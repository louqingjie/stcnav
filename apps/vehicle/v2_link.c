/*
 * V2 上位机链路层实现。
 *
 * 职责：
 *   - UART2 中断收发的上层轮询（流解码、按类型序列/uptime 校验）；
 *   - 心跳与状态帧调度（心跳 10 Hz，0x82/0x85/0x88 各 50 Hz）；
 *   - 上位机命令新鲜度与运动授权判定；
 *   - 上电 boot_id 持久化（app 区最后 512 B 保留扇区）。
 *
 * 消息对象由 v2_codec 持有（XDATA 直接寻址），这里只写字段/读字段。
 */

#include "v2_link.h"
#include "v2_codec.h"
#include "config.h"
#include "common.h"
#include "imu.h"
#include "timebase.h"
#include "stc32g_iap.h"
#include "stc32g_uart2.h"

#define V2_DEG_TO_RAD 0.0174532925f
#define V2_PI_F 3.14159265358979323846f
#define V2_TWO_PI_F 6.28318530717958647692f

#define V2_BOOT_RECORD_LENGTH 8u
#define V2_RX_BUDGET 512u

#define V2_HOST_SEQ_COMMAND 0u
#define V2_HOST_SEQ_POSTURE 1u

static v2_diag_t diag;

/* UART2 中断服务程序（向量 8，使能位 IE2.0/ES2）。
 * 放在本文件而不是 main.c：build.py 会按源码文本生成中断声明，
 * 而 vehicle_debug 不链接本文件也不启用 UART2 中断。 */
void uart2_isr(void) __interrupt (8)
{
    uart2_irq_handler();
}

static unsigned long boot_id;
static unsigned char ready_flag;

static uint16_t tx_seq_heartbeat;
static uint16_t tx_seq_gimbal;
static uint16_t tx_seq_heading;
static uint16_t tx_seq_attitude;

static unsigned int last_heartbeat_ms;
static unsigned int last_state_ms;

static unsigned long host_boot_id;
static unsigned char host_boot_known;
static unsigned char host_online;
static unsigned int host_last_ms;
static uint16_t host_heartbeat_seq;
static unsigned char host_heartbeat_seq_valid;
static uint16_t host_seq[2];
static unsigned char host_seq_valid[2];
static uint32_t host_last_uptime;
static unsigned char host_uptime_valid;

static float command_vx_mps;
static float command_vy_mps;
static float command_wz_rps;
static unsigned char command_enabled;
static unsigned int command_ms;
static unsigned char command_valid;

static float clampf(float value, float limit)
{
    if (value > limit)
        return limit;
    if (value < -limit)
        return -limit;
    return value;
}

static void send_heartbeat_frame(void);
static void send_state_frames(void);
static void handle_heartbeat(unsigned int now_ms);
static void handle_chassis_command(unsigned int now_ms);
static uint8_t next_host_seq(uint8_t slot, uint16_t sequence);

static float wrap180_degrees(float angle)
{
    while (angle > 180.0f)
        angle -= 360.0f;
    while (angle < -180.0f)
        angle += 360.0f;
    return angle;
}

static float degrees_to_pi_radians(float degrees)
{
    float radians = degrees * V2_DEG_TO_RAD;

    while (radians >= V2_PI_F)
        radians -= V2_TWO_PI_F;
    while (radians < -V2_PI_F)
        radians += V2_TWO_PI_F;
    return radians;
}

/* ---------- boot_id 持久化 ---------- */

static unsigned char boot_record_read(unsigned long *counter)
{
    unsigned char record[V2_BOOT_RECORD_LENGTH];
    unsigned char index;

    iap_begin();
    for (index = 0; index < V2_BOOT_RECORD_LENGTH; index++)
        record[index] = iap_read_byte(V2_BOOT_SECTOR_ADDR + (unsigned long)index);
    iap_end();

    if (record[0] != (unsigned char)'V' || record[1] != (unsigned char)'B' ||
        record[2] != (unsigned char)'I' || record[3] != (unsigned char)'D')
        return 0;

    *counter = (unsigned long)record[4] |
               ((unsigned long)record[5] << 8) |
               ((unsigned long)record[6] << 16) |
               ((unsigned long)record[7] << 24);

    return (unsigned char)(*counter != 0UL);
}

static unsigned char boot_record_write(unsigned long counter)
{
    unsigned char record[V2_BOOT_RECORD_LENGTH];
    unsigned char index;
    unsigned char failed = 0;

    record[0] = (unsigned char)'V';
    record[1] = (unsigned char)'B';
    record[2] = (unsigned char)'I';
    record[3] = (unsigned char)'D';
    record[4] = (unsigned char)(counter & 0xFFUL);
    record[5] = (unsigned char)((counter >> 8) & 0xFFUL);
    record[6] = (unsigned char)((counter >> 16) & 0xFFUL);
    record[7] = (unsigned char)((counter >> 24) & 0xFFUL);

    iap_begin();
    if (iap_erase_sector(V2_BOOT_SECTOR_ADDR))
        failed = 1;
    if (!failed)
    {
        for (index = 0; index < V2_BOOT_RECORD_LENGTH; index++)
        {
            if (iap_write_byte(V2_BOOT_SECTOR_ADDR + (unsigned long)index,
                               record[index]))
            {
                failed = 1;
                break;
            }
        }
    }
    if (iap_failed())
        failed = 1;
    iap_end();

    return (unsigned char)(failed ? 0u : 1u);
}

unsigned long v2_boot_id_load(void)
{
    unsigned long counter = 0UL;
    unsigned long next;

    if (boot_record_read(&counter))
        next = counter + 1UL;
    else
        next = 1UL;

    if (next == 0UL)
        next = 1UL;

    if (!boot_record_write(next))
        diag.boot_persist_fault = 1;

    return next;
}

/* ---------- 初始化与轮询 ---------- */

void v2_link_init(unsigned long id, unsigned int now_ms)
{
    boot_id = (id != 0UL) ? id : 1UL;
    ready_flag = 0;

    tx_seq_heartbeat = 0;
    tx_seq_gimbal = 0;
    tx_seq_heading = 0;
    tx_seq_attitude = 0;

    last_heartbeat_ms = now_ms;
    last_state_ms = now_ms;

    v2_decoder_reset();

    host_boot_known = 0;
    host_online = 0;
    host_heartbeat_seq_valid = 0;
    host_seq_valid[0] = 0;
    host_seq_valid[1] = 0;
    host_uptime_valid = 0;

    command_valid = 0;
    command_enabled = 0;
    command_vx_mps = 0.0f;
    command_vy_mps = 0.0f;
    command_wz_rps = 0.0f;
    command_ms = now_ms;

    diag.boot_id = boot_id;
    diag.ready = 0;

    uart2_init_async(V2_UART_BAUD);
    send_heartbeat_frame();
    last_heartbeat_ms = now_ms;
}

void v2_link_poll(unsigned int now_ms, unsigned char imu_ok)
{
    unsigned char result;
    unsigned char rx_byte;
    unsigned int budget = 0;

    while (budget < V2_RX_BUDGET && uart2_rx_pop(&rx_byte))
    {
        budget++;
        v2_decoder_feed(rx_byte);

        for (;;)
        {
            result = v2_decoder_next();
            if (result == V2_DECODE_NEED_MORE)
                break;
            if (result == V2_DECODE_INVALID)
                continue;
            if (result == V2_DECODE_UNKNOWN)
            {
                diag.rx_unknown_types++;
                diag.rx_frames++;
                continue;
            }
            if (result == V2_DECODE_BAD_PAYLOAD)
            {
                diag.rx_bad_payload++;
                continue;
            }

            diag.rx_frames++;
            if (v2_rx_type == V2_TYPE_HEARTBEAT)
            {
                handle_heartbeat(now_ms);
            }
            else if (v2_rx_type == V2_TYPE_CHASSIS_COMMAND)
            {
                handle_chassis_command(now_ms);
            }
            else if (host_boot_known &&
                     next_host_seq(V2_HOST_SEQ_POSTURE, v2_rx_sequence))
            {
                diag.posture_requests++;
            }
            else
            {
                diag.rx_rejected++;
            }
        }
    }

    if (host_online && (unsigned int)(now_ms - host_last_ms) > V2_HOST_TIMEOUT_MS)
    {
        host_online = 0;
        command_valid = 0;
        diag.rx_host_timeouts++;
    }

    if ((unsigned int)(now_ms - last_heartbeat_ms) >= V2_HEARTBEAT_PERIOD_MS)
    {
        last_heartbeat_ms = now_ms;
        send_heartbeat_frame();
    }

    if (ready_flag && imu_ok &&
        (unsigned int)(now_ms - last_state_ms) >= V2_STATE_PERIOD_MS)
    {
        last_state_ms = now_ms;
        send_state_frames();
    }

    diag.rx_crc_errors = v2_decoder_crc_errors;
    diag.rx_framing_errors = v2_decoder_framing_errors;
    diag.tx_overflows = uart2_tx_overflow_count();
    diag.host_online = host_online;
    diag.ready = ready_flag;
    diag.command_enabled = command_enabled;
    diag.command_fresh = (unsigned char)(command_valid &&
        ((unsigned int)(now_ms - command_ms) <= V2_CMD_TIMEOUT_MS));
}

void v2_link_set_ready(unsigned char ready)
{
    ready_flag = (unsigned char)(ready ? 1u : 0u);
    diag.ready = ready_flag;
}

unsigned char v2_motion_take(unsigned int now_ms, unsigned char imu_ok,
                             float *vx_mps, float *vy_mps, float *wz_rad_s)
{
#if V2_ENABLE_MOTION
    if (!imu_ok || !host_online || !command_valid || !command_enabled)
        return 0;
    if ((unsigned int)(now_ms - host_last_ms) > V2_HOST_TIMEOUT_MS)
        return 0;
    if ((unsigned int)(now_ms - command_ms) > V2_CMD_TIMEOUT_MS)
        return 0;

    *vx_mps = command_vx_mps;
    *vy_mps = command_vy_mps;
    *wz_rad_s = command_wz_rps;
    return 1;
#else
    (void)now_ms;
    (void)imu_ok;
    (void)vx_mps;
    (void)vy_mps;
    (void)wz_rad_s;
    return 0;
#endif
}

const v2_diag_t *v2_link_diagnostics(void)
{
    return &diag;
}

/* ---------- 发送 ---------- */

static void send_heartbeat_frame(void)
{
    uint16_t size;

    v2_tx_heartbeat.uptime_ms = (uint32_t)millis32();
    v2_tx_heartbeat.boot_id = (uint32_t)boot_id;
    v2_tx_heartbeat.capabilities = (uint32_t)V2_CAPABILITIES;
    v2_tx_heartbeat.ready = ready_flag;

    size = v2_encode_heartbeat(tx_seq_heartbeat);
    if (size == 0u)
    {
        diag.tx_drops++;
        return;
    }
    if (uart2_tx_push(v2_tx_frame, (unsigned int)size))
    {
        tx_seq_heartbeat++;
        diag.tx_frames++;
    }
    else
    {
        diag.tx_drops++;
    }
}

static void send_state_frames(void)
{
    uint16_t size;

    /* 0x82 GimbalState：相对 yaw / 相对 yaw 速率。 */
    v2_tx_gimbal.relative_yaw_rad = degrees_to_pi_radians(
        wrap180_degrees(imu_get_gimbal_yaw() - imu_get_chassis_yaw()));
    v2_tx_gimbal.yaw_rate_rad_s =
        (imu_get_gimbal_rate() - imu_get_chassis_rate()) * V2_DEG_TO_RAD;
    v2_tx_gimbal.sample_sequence = (uint32_t)imu_snapshot.gimbal_sample_sequence;
    v2_tx_gimbal.mcu_time_ms = (uint32_t)imu_snapshot.sample_time_ms;
    v2_tx_gimbal.valid = 1u;
    v2_tx_gimbal.online = 1u;

    size = v2_encode_gimbal_state(tx_seq_gimbal);
    if (size != 0u && uart2_tx_push(v2_tx_frame, (unsigned int)size))
    {
        tx_seq_gimbal++;
        diag.tx_frames++;
    }
    else
    {
        diag.tx_drops++;
    }

    /* 0x85 ChassisHeadingState。 */
    v2_tx_heading.yaw_rad = degrees_to_pi_radians(imu_get_chassis_yaw());
    v2_tx_heading.yaw_rate_rad_s = imu_get_chassis_rate() * V2_DEG_TO_RAD;
    v2_tx_heading.sample_sequence = (uint32_t)imu_snapshot.chassis_sample_sequence;
    v2_tx_heading.mcu_time_ms = (uint32_t)imu_snapshot.sample_time_ms;
    v2_tx_heading.reset_counter = (uint16_t)imu_snapshot.epoch;
    v2_tx_heading.valid = 1u;
    v2_tx_heading.online = 1u;

    size = v2_encode_chassis_heading_state(tx_seq_heading);
    if (size != 0u && uart2_tx_push(v2_tx_frame, (unsigned int)size))
    {
        tx_seq_heading++;
        diag.tx_frames++;
    }
    else
    {
        diag.tx_drops++;
    }

    /* 0x88 ChassisAttitudeState：quaternion/gyro/acc 由 imu.c 直接写入
       v2_tx_attitude，这里只补生命周期字段。 */
    v2_tx_attitude.sample_sequence = (uint32_t)imu_snapshot.chassis_sample_sequence;
    v2_tx_attitude.mcu_time_ms = (uint32_t)imu_snapshot.sample_time_ms;
    v2_tx_attitude.source_boot_id = (uint32_t)boot_id;
    v2_tx_attitude.reset_counter = (uint16_t)imu_snapshot.epoch;
    v2_tx_attitude.orientation_valid = 1u;
    v2_tx_attitude.angular_velocity_valid = 1u;
    v2_tx_attitude.linear_acceleration_valid = 1u;
    v2_tx_attitude.online = 1u;

    size = v2_encode_chassis_attitude_state(tx_seq_attitude);
    if (size != 0u && uart2_tx_push(v2_tx_frame, (unsigned int)size))
    {
        tx_seq_attitude++;
        diag.tx_frames++;
    }
    else
    {
        diag.tx_drops++;
    }
}

/* ---------- 接收处理 ---------- */

static uint8_t next_host_seq(uint8_t slot, uint16_t sequence)
{
    if (host_seq_valid[slot])
    {
        if (!v2_sequence_is_newer(sequence, host_seq[slot]))
            return 0;
    }
    host_seq_valid[slot] = 1u;
    host_seq[slot] = sequence;
    return 1u;
}

static void handle_heartbeat(unsigned int now_ms)
{
    if (v2_rx_heartbeat.boot_id == 0UL)
    {
        diag.rx_bad_payload++;
        return;
    }

    if (!host_boot_known || v2_rx_heartbeat.boot_id != host_boot_id)
    {
        host_boot_known = 1;
        host_boot_id = v2_rx_heartbeat.boot_id;
        host_uptime_valid = 0;
        host_heartbeat_seq_valid = 0;
        host_seq_valid[0] = 0;
        host_seq_valid[1] = 0;

        command_valid = 0;
        command_enabled = 0;
        command_vx_mps = 0.0f;
        command_vy_mps = 0.0f;
        command_wz_rps = 0.0f;

        diag.rx_host_boot_changes++;
    }

    if (host_heartbeat_seq_valid)
    {
        if (!v2_sequence_is_newer(v2_rx_sequence, host_heartbeat_seq))
        {
            diag.rx_rejected++;
            return;
        }
    }
    host_heartbeat_seq = v2_rx_sequence;
    host_heartbeat_seq_valid = 1;

    if (host_uptime_valid)
    {
        if ((uint32_t)(v2_rx_heartbeat.uptime_ms - host_last_uptime) >= 0x80000000UL)
        {
            diag.rx_bad_payload++;
            return;
        }
    }
    host_last_uptime = v2_rx_heartbeat.uptime_ms;
    host_uptime_valid = 1;

    host_last_ms = now_ms;
    host_online = 1;
}

static void handle_chassis_command(unsigned int now_ms)
{
    if (!host_boot_known || !next_host_seq(V2_HOST_SEQ_COMMAND, v2_rx_sequence))
    {
        diag.rx_rejected++;
        return;
    }

    command_vx_mps = clampf(v2_rx_chassis_command.vx_mps, V2_MAX_LINEAR_MPS);
    command_vy_mps = clampf(v2_rx_chassis_command.vy_mps, V2_MAX_LINEAR_MPS);
    command_wz_rps = clampf(v2_rx_chassis_command.wz_rad_s, V2_MAX_WZ_RPS);
    command_enabled = v2_rx_chassis_command.enabled;
    command_ms = now_ms;
    command_valid = 1;
}
