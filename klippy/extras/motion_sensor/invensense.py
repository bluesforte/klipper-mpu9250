# Support for reading acceleration data from an mpu9250 chip
#
# Copyright (C) 2022  Harry Beyel <harry3b9@gmail.com>
# Copyright (C) 2020-2021 Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging, time, collections, threading, multiprocessing, os

from motion_sensor import MotionSensorBase, FREEFALL_ACCEL
from .. import bus

MPU9250_ADDR =      0x68

MPU9250_DEV_ID =    0x73
MPU6050_DEV_ID =    0x68

# MPU9250 registers
REG_DEVID =         0x75
REG_FIFO_EN =       0x23
REG_SMPLRT_DIV =    0x19
REG_CONFIG =        0x1A
REG_ACCEL_CONFIG =  0x1C
REG_ACCEL_CONFIG2 = 0x1D
REG_USER_CTRL =     0x6A
REG_PWR_MGMT_1 =    0x6B
REG_PWR_MGMT_2 =    0x6C

SET_CONFIG =        0x01 # FIFO mode 'stream' style
SET_ACCEL_CONFIG =  0x10 # 8g full scale
SET_ACCEL_CONFIG2 = 0x08 # 1046Hz BW, 0.503ms delay 4kHz sample rate
SET_PWR_MGMT_1_WAKE =     0x00
SET_PWR_MGMT_1_SLEEP=     0x40
SET_PWR_MGMT_2_ACCEL_ON = 0x07
SET_PWR_MGMT_2_OFF  =     0x3F

# Helper method for getting the two's complement value of an unsigned int
def twos_complement(val, nbits):
    if (val & (1 << (nbits - 1))) != 0:
        val = val - (1 << nbits)
    return val

# Printer class that controls MPU9250 chip
class MPU9250 (MotionSensorBase):
    # SCALE = 1/4096 g/LSB @8g scale * Earth gravity in mm/s**2
    SCALE = 0.000244140625 * FREEFALL_ACCEL
    BYTES_PER_SAMPLE = 6
    SAMPLES_PER_BLOCK = 8
    FIFO_SIZE = 512 // BYTES_PER_SAMPLE
    SAMPLE_RATES = { 4000: 0x00 }

    def __init__(self, config):
        super(MPU9250, self).__init__(config)
        
        self.data_rate = config.getint('rate', 4000)
        if self.data_rate not in self.SAMPLE_RATES:
            raise config.error("Invalid rate parameter: %d" % (self.data_rate))
        self.mcu.register_response(self._handle_motion_sensor_data, 
                                    "mpu9250_data", self.oid)

    def _init_conn(self, config):
        logging.log(logging.INFO, "Setting up I2C connection for MPU9250")
        # Setup mcu sensor_mpu9250 bulk query code
        self.conn = bus.MCU_I2C_from_config(config,
                                           default_addr=MPU9250_ADDR,
                                           default_speed=400000)

    def _build_config(self):
        cmdqueue = self.conn.get_command_queue()
        self.mcu.add_config_cmd("config_mpu9250 oid=%d i2c_oid=%d"
                           % (self.oid, self.conn.get_oid()))
        self.mcu.add_config_cmd("query_mpu9250 oid=%d clock=0 rest_ticks=0"
                           % (self.oid,), on_restart=True)
        self.query_motion_sensor_cmd = self.mcu.lookup_command(
            "query_mpu9250 oid=%c clock=%u rest_ticks=%u", cq=cmdqueue)
        self.query_motion_sensor_end_cmd = self.mcu.lookup_query_command(
            "query_mpu9250 oid=%c clock=%u rest_ticks=%u",
            "mpu9250_status oid=%c clock=%u query_ticks=%u next_sequence=%hu"
            " buffered=%c fifo=%u limit_count=%hu", oid=self.oid, cq=cmdqueue)
        self.query_motion_sensor_status_cmd = self.mcu.lookup_query_command(
            "query_mpu9250_status oid=%c",
            "mpu9250_status oid=%c clock=%u query_ticks=%u next_sequence=%hu"
            " buffered=%c fifo=%u limit_count=%hu", oid=self.oid, cq=cmdqueue)

    def read_reg(self, reg):
        params = self.conn.i2c_read([reg], 1)
        return bytearray(params['response'])[0]

    def set_reg(self, reg, val, minclock=0):
        self.conn.i2c_write([reg, val & 0xFF], minclock=minclock)

    def _extract_samples(self, raw_samples):
        # Load variables to optimize inner loop below
        (x_pos, x_scale), (y_pos, y_scale), (z_pos, z_scale) = self.axes_map
        last_sequence = self.last_sequence
        time_base, chip_base, inv_freq = self.clock_sync.get_time_translation()
        # Process every message in raw_samples
        count = seq = 0
        samples = [None] * (len(raw_samples) * self.SAMPLES_PER_BLOCK)
        for params in raw_samples:
            seq_diff = (last_sequence - params['sequence']) & 0xffff
            seq_diff -= (seq_diff & 0x8000) << 1
            seq = last_sequence - seq_diff
            d = bytearray(params['data'])
            msg_cdiff = seq * self.SAMPLES_PER_BLOCK - chip_base

            for i in range(len(d) // self.BYTES_PER_SAMPLE):
                d_xyz = d[i*self.BYTES_PER_SAMPLE:(i+1)*self.BYTES_PER_SAMPLE]
                xhigh, xlow, yhigh, ylow, zhigh, zlow = d_xyz
                rx = twos_complement(xhigh << 8 | xlow, 16)
                ry = twos_complement(yhigh << 8 | ylow, 16)
                rz = twos_complement(zhigh << 8 | zlow, 16)
                raw_xyz = (rx, ry, rz)

                x = round(raw_xyz[x_pos] * x_scale, 6)
                y = round(raw_xyz[y_pos] * y_scale, 6)
                z = round(raw_xyz[z_pos] * z_scale, 6)
                ptime = round(time_base + (msg_cdiff + i) * inv_freq, 6)
                samples[count] = (ptime, x, y, z)
                count += 1
        self.clock_sync.set_last_chip_clock(seq * self.SAMPLES_PER_BLOCK + i)
        del samples[count:]
        return samples

    def _start_measurements(self):
        if self.is_measuring():
            return

        # In case of miswiring, testing MPU9250 device ID prevents treating
        # noise or wrong signal as a correctly initialized device
        dev_id = self.read_reg(REG_DEVID)
        if dev_id != MPU9250_DEV_ID and dev_id != MPU6050_DEV_ID:
            raise self.printer.command_error(
                "Invalid mpu9250/mpu6050 id (got %x).\n"
                "This is generally indicative of connection problems\n"
                "(e.g. faulty wiring) or a faulty chip."
                % (dev_id))
        # Wake up chip
        self.set_reg(REG_PWR_MGMT_1, SET_PWR_MGMT_1_WAKE)
        self.set_reg(REG_PWR_MGMT_2, SET_PWR_MGMT_2_ACCEL_ON)
        time.sleep(20. / 1000) # wait for wake up
        # Setup chip in requested query rate
        self.set_reg(REG_SMPLRT_DIV, self.SAMPLE_RATES[self.data_rate])
        self.set_reg(REG_CONFIG, SET_CONFIG)
        self.set_reg(REG_ACCEL_CONFIG, SET_ACCEL_CONFIG)
        self.set_reg(REG_ACCEL_CONFIG2, SET_ACCEL_CONFIG2)
        super(MPU9250, self)._start_measurements()
        
    def _finish_measurements(self):
        super(MPU9250, self)._finish_measurements()
        # put device to sleep and turn off sensors
        self.set_reg(REG_PWR_MGMT_1, SET_PWR_MGMT_1_SLEEP)
        self.set_reg(REG_PWR_MGMT_2, SET_PWR_MGMT_2_OFF)