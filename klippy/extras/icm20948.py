# Support for reading acceleration data from an icm20948 chip
#
# Copyright (C) 2022  Harry Beyel <harry3b9@gmail.com>
# Copyright (C) 2020-2021 Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging, time, collections, threading
from . import bus, motion_report, adxl345

ICM20948_ADDR =      0x68

ICM20948_DEV_ID =    0xEA

# ICM20948 registers
REG_DEVID =         0x00
REG_SMPLRT_DIV_1 =  0x10
REG_SMPLRT_DIV_2 =  0x11
REG_FIFO_EN_2 =     0x67
REG_ACCEL_CONFIG =  0x14
REG_ACCEL_CONFIG2 = 0x15
REG_USER_CTRL =     0x03
REG_PWR_MGMT_1 =    0x06
REG_PWR_MGMT_2 =    0x07

SAMPLE_RATE_DIVS = { 4500:0x00 }

SET_FIFO_EN_2_ACCEL_ON = 0x10 # Output accel data to fifo
SET_ACCEL_CONFIG =  0x04 # 8g full scale, disable DLPF
SET_ACCEL_CONFIG2 = 0x00 # Disable sample averaging
SET_PWR_MGMT_1_WAKE =     0x01 # wake, auto clk select
SET_PWR_MGMT_1_SLEEP=     0x41 # auto clk select
SET_PWR_MGMT_2_ACCEL_ON = 0x07 # enable only accel
SET_PWR_MGMT_2_OFF  =     0x3F # disable accel and gyro

FREEFALL_ACCEL = 9.80665 * 1000.
# SCALE = 1/4096 g/LSB @8g scale * Earth gravity in mm/s**2
SCALE = 0.000244140625 * FREEFALL_ACCEL

FIFO_SIZE = 512

Accel_Measurement = collections.namedtuple(
    'Accel_Measurement', ('time', 'accel_x', 'accel_y', 'accel_z'))

MIN_MSG_TIME = 0.100

BYTES_PER_SAMPLE = 6
SAMPLES_PER_BLOCK = 8

# Printer class that controls ICM20948 chip
class ICM20948:
    def __init__(self, config):
        self.printer = config.get_printer()
        adxl345.AccelCommandHelper(config, self)
        self.query_rate = 0
        am = {'x': (0, SCALE), 'y': (1, SCALE), 'z': (2, SCALE),
              '-x': (0, -SCALE), '-y': (1, -SCALE), '-z': (2, -SCALE)}
        axes_map = config.getlist('axes_map', ('x','y','z'), count=3)
        if any([a not in am for a in axes_map]):
            raise config.error("Invalid icm20948 axes_map parameter")
        self.axes_map = [am[a.strip()] for a in axes_map]
        self.data_rate = config.getint('rate', 4500)
        if self.data_rate not in SAMPLE_RATE_DIVS:
            raise config.error("Invalid rate parameter: %d" % self.data_rate)
        # Measurement storage (accessed from background thread)
        self.lock = threading.Lock()
        self.raw_samples = []
        # Setup mcu sensor_icm20948 bulk query code
        self.i2c = bus.MCU_I2C_from_config(config,
                                           default_addr=ICM20948_ADDR,
                                           default_speed=400000)
        self.mcu = mcu = self.i2c.get_mcu()
        self.oid = oid = mcu.create_oid()
        self.query_icm20948_cmd = self.query_icm20948_end_cmd = None
        self.query_icm20948_status_cmd = None
        mcu.register_config_callback(self._build_config)
        mcu.register_response(self._handle_icm20948_data, "icm20948_data", oid)
        # Clock tracking
        self.last_sequence = self.max_query_duration = 0
        self.last_limit_count = self.last_error_count = 0
        self.clock_sync = adxl345.ClockSyncRegression(self.mcu, 640)
        # API server endpoints
        self.api_dump = motion_report.APIDumpHelper(
            self.printer, self._api_update, self._api_startstop, 0.100)
        self.name = config.get_name().split()[-1]
        wh = self.printer.lookup_object('webhooks')
        wh.register_mux_endpoint("icm20948/dump_icm20948", "sensor", self.name,
                                 self._handle_dump_icm20948)
    def _build_config(self):
        cmdqueue = self.i2c.get_command_queue()
        self.mcu.add_config_cmd("config_icm20948 oid=%d i2c_oid=%d"
                           % (self.oid, self.i2c.get_oid()))
        self.mcu.add_config_cmd("query_icm20948 oid=%d clock=0 rest_ticks=0"
                           % (self.oid,), on_restart=True)
        self.query_icm20948_cmd = self.mcu.lookup_command(
            "query_icm20948 oid=%c clock=%u rest_ticks=%u", cq=cmdqueue)
        self.query_icm20948_end_cmd = self.mcu.lookup_query_command(
            "query_icm20948 oid=%c clock=%u rest_ticks=%u",
            "icm20948_status oid=%c clock=%u query_ticks=%u next_sequence=%hu"
            " buffered=%c fifo=%u limit_count=%hu", oid=self.oid, cq=cmdqueue)
        self.query_icm20948_status_cmd = self.mcu.lookup_query_command(
            "query_icm20948_status oid=%c",
            "icm20948_status oid=%c clock=%u query_ticks=%u next_sequence=%hu"
            " buffered=%c fifo=%u limit_count=%hu", oid=self.oid, cq=cmdqueue)
    def read_reg(self, reg):
        params = self.i2c.i2c_read([reg], 1)
        return bytearray(params['response'])[0]

    def set_reg(self, reg, val, minclock=0):
        self.i2c.i2c_write([reg, val & 0xFF], minclock=minclock)

    # Measurement collection
    def is_measuring(self):
        return self.query_rate > 0
    def _handle_icm20948_data(self, params):
        with self.lock:
            self.raw_samples.append(params)
    def _extract_samples(self, raw_samples):
        # Load variables to optimize inner loop below
        (x_pos, x_scale), (y_pos, y_scale), (z_pos, z_scale) = self.axes_map
        last_sequence = self.last_sequence
        time_base, chip_base, inv_freq = self.clock_sync.get_time_translation()
        # Process every message in raw_samples
        count = seq = 0
        samples = [None] * (len(raw_samples) * SAMPLES_PER_BLOCK)
        for params in raw_samples:
            seq_diff = (last_sequence - params['sequence']) & 0xffff
            seq_diff -= (seq_diff & 0x8000) << 1
            seq = last_sequence - seq_diff
            d = bytearray(params['data'])
            msg_cdiff = seq * SAMPLES_PER_BLOCK - chip_base

            for i in range(len(d) // BYTES_PER_SAMPLE):
                d_xyz = d[i*BYTES_PER_SAMPLE:(i+1)*BYTES_PER_SAMPLE]
                xhigh, xlow, yhigh, ylow, zhigh, zlow = d_xyz
                # Merge and perform twos-complement
                rx = ((xhigh << 8) | xlow) - ((xhigh & 0x80) << 9)
                ry = ((yhigh << 8) | ylow) - ((yhigh & 0x80) << 9)
                rz = ((zhigh << 8) | zlow) - ((zhigh & 0x80) << 9)

                raw_xyz = (rx, ry, rz)
                x = round(raw_xyz[x_pos] * x_scale, 6)
                y = round(raw_xyz[y_pos] * y_scale, 6)
                z = round(raw_xyz[z_pos] * z_scale, 6)
                ptime = round(time_base + (msg_cdiff + i) * inv_freq, 6)
                samples[count] = (ptime, x, y, z)
                count += 1
        self.clock_sync.set_last_chip_clock(seq * SAMPLES_PER_BLOCK + i)
        del samples[count:]
        return samples

    def _update_clock(self, minclock=0):
        # Query current state
        for retry in range(5):
            params = self.query_icm20948_status_cmd.send([self.oid],
                                                        minclock=minclock)
            fifo = params['fifo'] & 0x1fff
            if fifo <= FIFO_SIZE:
                break
        else:
            raise self.printer.command_error("Unable to query icm20948 fifo")
        mcu_clock = self.mcu.clock32_to_clock64(params['clock'])
        sequence = (self.last_sequence & ~0xffff) | params['next_sequence']
        if sequence < self.last_sequence:
            sequence += 0x10000
        self.last_sequence = sequence
        buffered = params['buffered']
        limit_count = (self.last_limit_count & ~0xffff) | params['limit_count']
        if limit_count < self.last_limit_count:
            limit_count += 0x10000
        self.last_limit_count = limit_count
        duration = params['query_ticks']
        if duration > self.max_query_duration:
            # Skip measurement as a high query time could skew clock tracking
            self.max_query_duration = max(2 * self.max_query_duration,
                                          self.mcu.seconds_to_clock(.000005))
            return
        self.max_query_duration = 2 * duration
        msg_count = (sequence * SAMPLES_PER_BLOCK
                     + buffered // BYTES_PER_SAMPLE + fifo)
        # The "chip clock" is the message counter plus .5 for average
        # inaccuracy of query responses and plus .5 for assumed offset
        # of icm20948 hw processing time.
        chip_clock = msg_count + 1
        self.clock_sync.update(mcu_clock + duration // 2, chip_clock)
    def _start_measurements(self):
        if self.is_measuring():
            return
        # In case of miswiring, testing ICM20948 device ID prevents treating
        # noise or wrong signal as a correctly initialized device
        dev_id = self.read_reg(REG_DEVID)
        if dev_id != ICM20948_DEV_ID:
            raise self.printer.command_error(
                "Invalid ICM20948 id (got %x).\n"
                "This is generally indicative of connection problems\n"
                "(e.g. faulty wiring) or a faulty chip."
                % (dev_id))
        # Setup chip in requested query rate
        self.set_reg(REG_PWR_MGMT_1, SET_PWR_MGMT_1_WAKE)
        self.set_reg(REG_PWR_MGMT_2, SET_PWR_MGMT_2_ACCEL_ON)
        time.sleep(20. / 1000) # wait for accelerometer chip wake up
        self.set_reg(REG_SMPLRT_DIV_2, SAMPLE_RATE_DIVS[self.data_rate])
        self.set_reg(REG_FIFO_EN_2, SET_FIFO_EN_2_ACCEL_ON)
        self.set_reg(REG_ACCEL_CONFIG, SET_ACCEL_CONFIG)
        self.set_reg(REG_ACCEL_CONFIG2, SET_ACCEL_CONFIG2)

        # Setup samples
        with self.lock:
            self.raw_samples = []
        # Start bulk reading
        systime = self.printer.get_reactor().monotonic()
        print_time = self.mcu.estimated_print_time(systime) + MIN_MSG_TIME
        reqclock = self.mcu.print_time_to_clock(print_time)
        rest_ticks = self.mcu.seconds_to_clock(1. / self.data_rate)
        self.query_rate = self.data_rate
        self.query_icm20948_cmd.send([self.oid, reqclock, rest_ticks],
                                    reqclock=reqclock)
        logging.info("ICM20948 starting '%s' measurements", self.name)
        # Initialize clock tracking
        self.last_sequence = 0
        self.last_limit_count = self.last_error_count = 0
        self.clock_sync.reset(reqclock, 0)
        self.max_query_duration = 1 << 31
        self._update_clock(minclock=reqclock)
        self.max_query_duration = 1 << 31
    def _finish_measurements(self):
        if not self.is_measuring():
            return
        # Halt bulk reading
        params = self.query_icm20948_end_cmd.send([self.oid, 0, 0])
        self.query_rate = 0
        with self.lock:
            self.raw_samples = []
        logging.info("ICM20948 finished '%s' measurements", self.name)
        self.set_reg(REG_PWR_MGMT_1, SET_PWR_MGMT_1_SLEEP)
        self.set_reg(REG_PWR_MGMT_2, SET_PWR_MGMT_2_OFF)

    # API interface
    def _api_update(self, eventtime):
        self._update_clock()
        with self.lock:
            raw_samples = self.raw_samples
            self.raw_samples = []
        if not raw_samples:
            return {}
        samples = self._extract_samples(raw_samples)
        if not samples:
            return {}
        return {'data': samples, 'errors': self.last_error_count,
                'overflows': self.last_limit_count}
    def _api_startstop(self, is_start):
        if is_start:
            self._start_measurements()
        else:
            self._finish_measurements()
    def _handle_dump_icm20948(self, web_request):
        self.api_dump.add_client(web_request)
        hdr = ('time', 'x_acceleration', 'y_acceleration', 'z_acceleration')
        web_request.send({'header': hdr})
    def start_internal_client(self):
        cconn = self.api_dump.add_internal_client()
        return adxl345.AccelQueryHelper(self.printer, cconn)

def load_config(config):
    return ICM20948(config)

def load_config_prefix(config):
    return ICM20948(config)
