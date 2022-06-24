// Support for gathering acceleration data from icm20948 chip
//
// Copyright (C) 2022 Harry Beyel <harry3b9@gmail.com>
// Copyright (C) 2020-2021 Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <string.h> // memcpy
#include "board/irq.h" // irq_disable
#include "board/misc.h" // timer_read_time
#include "basecmd.h" // oid_alloc
#include "command.h" // DECL_COMMAND
#include "sched.h" // DECL_TASK
#include "board/gpio.h" // i2c_read
#include "i2ccmds.h" // i2cdev_oid_lookup

// Chip registers
#define AR_FIFO_SIZE 512

#define AR_FIFO_EN_2    0x67
#define AR_FIFO_RST     0x68
#define AR_USER_CTRL    0x03
#define AR_FIFO_COUNT_H 0x70
#define AR_FIFO_R_W     0x72

#define SET_FIFO_EN_2_ACCEL_ON  0x10
#define SET_FIFO_EN_2_ALL_OFF   0x00
#define SET_FIFO_RST_RESET      0x1F
#define SET_FIFO_RST_NORMAL     0x00
#define SET_USER_CTRL_FIFO_EN   0x40

#define SET_PWR_SLEEP   0x40
#define SET_PWR_WAKE    0x00
#define SET_PWR_2_ACCEL 0x07 // only enable accelerometers
#define SET_PWR_2_NONE  0x3F // disable all sensors

#define BYTES_PER_FIFO_ENTRY 6

struct icm20948 {
    struct timer timer;
    uint32_t rest_ticks;
    struct i2cdev_s *i2c;
    uint16_t sequence, limit_count;
    uint8_t flags, data_count;
    // data size must be <= 255 due to i2c api
    // = SAMPLES_PER_BLOCK (from icm20948.py) * BYTES_PER_FIFO_ENTRY + 1
    uint8_t data[48];
};

enum {
    AX_HAVE_START = 1<<0, AX_RUNNING = 1<<1, AX_PENDING = 1<<2,
};

static struct task_wake icm20948_wake;

// Reads the fifo byte count from the device.
uint16_t
get_fifo_status_icm (struct icm20948 *mp)
{
    uint8_t regs[] = {AR_FIFO_COUNT_H};
    uint8_t msg[2];
    i2c_read(mp->i2c->i2c_config, sizeof(regs), regs, 2, msg);
    msg[0] = 0x1F & msg[0]; // discard 3 MSB per datasheet
    return (((uint16_t)msg[0]) << 8 | msg[1]);
}

// Event handler that wakes icm20948_task() periodically
static uint_fast8_t
icm20948_event(struct timer *timer)
{
    struct icm20948 *ax = container_of(timer, struct icm20948, timer);
    ax->flags |= AX_PENDING;
    sched_wake_task(&icm20948_wake);
    return SF_DONE;
}

void
command_config_icm20948(uint32_t *args)
{
    struct icm20948 *mp = oid_alloc(args[0], command_config_icm20948
                                   , sizeof(*mp));
    mp->timer.func = icm20948_event;
    mp->i2c = i2cdev_oid_lookup(args[1]);
}
DECL_COMMAND(command_config_icm20948, "config_icm20948 oid=%c i2c_oid=%c");

// Report local measurement buffer
static void
icm20948_report(struct icm20948 *mp, uint8_t oid)
{
    sendf("icm20948_data oid=%c sequence=%hu data=%*s"
          , oid, mp->sequence, mp->data_count, mp->data);
    mp->data_count = 0;
    mp->sequence++;
}

// Report buffer and fifo status
static void
icm20948_status(struct icm20948 *mp, uint_fast8_t oid
            , uint32_t time1, uint32_t time2, uint16_t fifo)
{
    sendf("icm20948_status oid=%c clock=%u query_ticks=%u next_sequence=%hu"
          " buffered=%c fifo=%u limit_count=%hu"
          , oid, time1, time2-time1, mp->sequence
          , mp->data_count, fifo, mp->limit_count);
}

// Helper code to reschedule the icm20948_event() timer
static void
icm20948_reschedule_timer(struct icm20948 *mp)
{
    irq_disable();
    mp->timer.waketime = timer_read_time() + mp->rest_ticks;
    sched_add_timer(&mp->timer);
    irq_enable();
}

// Query accelerometer data
static void
icm20948_query(struct icm20948 *mp, uint8_t oid)
{
    // Check fifo status
    uint16_t fifo_bytes = get_fifo_status_icm(mp);
    if (fifo_bytes >= AR_FIFO_SIZE - BYTES_PER_FIFO_ENTRY)
        mp->limit_count++;

    // Read data
    // FIFO data are: [Xh, Xl, Yh, Yl, Zh, Zl]
    uint8_t reg = AR_FIFO_R_W;
    uint8_t bytes_to_read = fifo_bytes < sizeof(mp->data) - mp->data_count ?
                                    fifo_bytes & 0xFF :
                                    (sizeof(mp->data) - mp->data_count) & 0xFF;

    // round down to nearest full packet of data
    bytes_to_read = bytes_to_read / BYTES_PER_FIFO_ENTRY * BYTES_PER_FIFO_ENTRY;

    // Extract x, y, z measurements into data holder and report
    if (bytes_to_read > 0) {
        i2c_read(mp->i2c->i2c_config, sizeof(reg), &reg,
                bytes_to_read, &mp->data[mp->data_count]);
        mp->data_count += bytes_to_read;

        // report data when buffer is full
        if (mp->data_count + BYTES_PER_FIFO_ENTRY > sizeof(mp->data)) {
            icm20948_report(mp, oid);
        }
    }

    // check if we need to run the task again (more packets in fifo?)
    if ( bytes_to_read > 0 &&
            bytes_to_read / BYTES_PER_FIFO_ENTRY <
            fifo_bytes / BYTES_PER_FIFO_ENTRY) {
        // more data still ready in the fifo buffer
        sched_wake_task(&icm20948_wake);
    }
    else if (mp->flags & AX_RUNNING) {
        // No more fifo data, but actively running. Sleep until next check
        sched_del_timer(&mp->timer);
        mp->flags &= ~AX_PENDING;
        icm20948_reschedule_timer(mp);
    }
}

// Startup measurements
static void
icm20948_start(struct icm20948 *mp, uint8_t oid)
{
    sched_del_timer(&mp->timer);
    mp->flags = AX_RUNNING;
    uint8_t msg[2];

    msg[0] = AR_FIFO_EN_2;
    msg[1] = SET_FIFO_EN_2_ALL_OFF; // disable FIFO
    i2c_write(mp->i2c->i2c_config, sizeof(msg), msg);

    msg[0] = AR_FIFO_RST;
    msg[1] = SET_FIFO_RST_RESET; // reset FIFO buffer
    i2c_write(mp->i2c->i2c_config, sizeof(msg), msg);

    msg[0] = AR_USER_CTRL;
    msg[1] = SET_USER_CTRL_FIFO_EN; // enable FIFO buffer access
    i2c_write(mp->i2c->i2c_config, sizeof(msg), msg);

    msg[0] = AR_FIFO_EN_2;
    msg[1] = SET_FIFO_EN_2_ACCEL_ON; // enable accel output to FIFO
    i2c_write(mp->i2c->i2c_config, sizeof(msg), msg);

    msg[0] = AR_FIFO_RST;
    msg[1] = SET_FIFO_RST_NORMAL; // un-reset FIFO buffer
    i2c_write(mp->i2c->i2c_config, sizeof(msg), msg);

    icm20948_reschedule_timer(mp);
}

// End measurements
static void
icm20948_stop(struct icm20948 *mp, uint8_t oid)
{
    // Disable measurements
    sched_del_timer(&mp->timer);
    mp->flags = 0;

    // disable accel FIFO
    uint8_t msg[2] = { AR_FIFO_EN_2, SET_FIFO_EN_2_ALL_OFF };
    uint32_t end1_time = timer_read_time();
    i2c_write(mp->i2c->i2c_config, sizeof(msg), msg);
    uint32_t end2_time = timer_read_time();

    // Drain any measurements still in fifo
    uint16_t fifo_bytes = get_fifo_status_icm(mp);
    while (fifo_bytes >= BYTES_PER_FIFO_ENTRY) {
        icm20948_query(mp, oid);
        fifo_bytes = get_fifo_status_icm(mp);
    }

    // Report final data
    if (mp->data_count > 0)
        icm20948_report(mp, oid);
    icm20948_status(mp, oid, end1_time, end2_time,
                    fifo_bytes / BYTES_PER_FIFO_ENTRY);
}

void
command_query_icm20948(uint32_t *args)
{
    struct icm20948 *mp = oid_lookup(args[0], command_config_icm20948);

    if (!args[2]) {
        // End measurements
        icm20948_stop(mp, args[0]);
        return;
    }
    // Start new measurements query
    sched_del_timer(&mp->timer);
    mp->timer.waketime = args[1];
    mp->rest_ticks = args[2];
    mp->flags = AX_HAVE_START;
    mp->sequence = mp->limit_count = 0;
    mp->data_count = 0;
    sched_add_timer(&mp->timer);
}
DECL_COMMAND(command_query_icm20948,
             "query_icm20948 oid=%c clock=%u rest_ticks=%u");

void
command_query_icm20948_status(uint32_t *args)
{
    struct icm20948 *mp = oid_lookup(args[0], command_config_icm20948);
    uint8_t msg[2];
    uint32_t time1 = timer_read_time();
    uint8_t regs[] = {AR_FIFO_COUNT_H};
    i2c_read(mp->i2c->i2c_config, 1, regs, 2, msg);
    uint32_t time2 = timer_read_time();
    msg[0] = 0x1F & msg[0]; // discard 3 MSB
    uint16_t fifo_bytes = (((uint16_t)msg[0]) << 8) | msg[1];
    icm20948_status(mp, args[0], time1, time2, fifo_bytes / BYTES_PER_FIFO_ENTRY);
}
DECL_COMMAND(command_query_icm20948_status, "query_icm20948_status oid=%c");

void
icm20948_task(void)
{
    if (!sched_check_wake(&icm20948_wake))
        return;
    uint8_t oid;
    struct icm20948 *mp;
    foreach_oid(oid, mp, command_config_icm20948) {
        uint_fast8_t flags = mp->flags;
        if (!(flags & AX_PENDING)) {
            continue;
        }
        if (flags & AX_HAVE_START) {
            icm20948_start(mp, oid);
        }
        else {
            icm20948_query(mp, oid);
        }
    }
}
DECL_TASK(icm20948_task);
