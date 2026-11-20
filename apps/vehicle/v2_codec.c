/*
 * RM2027 V2 wire 编解码实现（STM 侧裁剪子集，紧凑版）。
 *
 * 见 v2_codec.h 与 apps/vehicle/docs/stm_v2_protocol_reference.md。
 * 注意 SDCC C251 约束：
 *   - MCS-251 为大端 ABI，float/uint32 用 union 取位后按小端逐字节写出；
 *   - 不用 size_t 表示 wire 长度，不用 double；
 *   - 大结构体/帧缓冲用 XDATA 全局量 + 常量偏移，避免通用指针与函数调用开销。
 */

#include "v2_codec.h"

#define V2_PI_F 3.14159265358979323846f

#if defined(__SDCC_mcs251)
#define V2_DATA __data
#else
#define V2_DATA
#endif

/* TX 侧消息：调用方（v2_link）直接写字段，本模块负责编码。 */
v2_heartbeat_t V2_XDATA v2_tx_heartbeat;
v2_gimbal_state_t V2_XDATA v2_tx_gimbal;
v2_chassis_heading_state_t V2_XDATA v2_tx_heading;
v2_chassis_attitude_state_t V2_XDATA v2_tx_attitude;
uint8_t V2_XDATA v2_tx_frame[V2_MAX_FRAME_SIZE];

/* RX 侧解码结果。 */
v2_chassis_command_t V2_XDATA v2_rx_chassis_command;
v2_heartbeat_t V2_XDATA v2_rx_heartbeat;
uint8_t V2_XDATA v2_rx_type;
uint16_t V2_XDATA v2_rx_sequence;

unsigned int v2_decoder_crc_errors;
unsigned int v2_decoder_framing_errors;

static uint8_t V2_XDATA v2_rx_buffer[V2_DECODER_BUFFER_SIZE];
static V2_XDATA uint16_t v2_rx_head;
static V2_XDATA uint16_t v2_rx_tail;

/* 4 字节 union：放 EDATA，float 位操作走直接寻址，代码远小于 XDATA。 */
static V2_DATA union
{
    float f;
    unsigned long u;
} f32_scratch;

/* 解码中间量（EDATA，避免硬件栈消耗）。 */
static V2_DATA uint16_t rx_scratch_u16;

#define V2_FINITE(value) \
    ((f32_scratch.f = (value)), (((f32_scratch.u >> 23) & 0xFFUL) != 0xFFUL))

/* 小端写出（MCS251 大端：按字节倒序拷贝；宿主机小端：顺序拷贝）。 */
#if defined(__SDCC_mcs251)
#define V2_F32_B0 3u
#define V2_F32_B1 2u
#define V2_F32_B2 1u
#define V2_F32_B3 0u
#define V2_U16_B0 1u
#define V2_U16_B1 0u
#else
#define V2_F32_B0 0u
#define V2_F32_B1 1u
#define V2_F32_B2 2u
#define V2_F32_B3 3u
#define V2_U16_B0 0u
#define V2_U16_B1 1u
#endif

/* wire(小端) -> 目标变量的内存字节序（target 大端 / host 小端通用）。 */
static void copy_u16_le_xdata(const uint8_t V2_XDATA *src, uint8_t V2_XDATA *dest)
{
    dest[0] = src[V2_U16_B0];
    dest[1] = src[V2_U16_B1];
}

static void copy_u16_le_data(const uint8_t V2_XDATA *src, uint8_t V2_DATA *dest)
{
    dest[0] = src[V2_U16_B0];
    dest[1] = src[V2_U16_B1];
}

static void copy_u32_le_xdata(const uint8_t V2_XDATA *src, uint8_t V2_XDATA *dest)
{
    dest[0] = src[V2_F32_B0];
    dest[1] = src[V2_F32_B1];
    dest[2] = src[V2_F32_B2];
    dest[3] = src[V2_F32_B3];
}

static uint8_t copy_f32_le_xdata(const uint8_t V2_XDATA *src, float V2_XDATA *dest)
{
    uint8_t V2_DATA *sink = (uint8_t V2_DATA *)&f32_scratch;

    sink[0] = src[V2_F32_B0];
    sink[1] = src[V2_F32_B1];
    sink[2] = src[V2_F32_B2];
    sink[3] = src[V2_F32_B3];

    if (((f32_scratch.u >> 23) & 0xFFUL) == 0xFFUL)
        return 0;

    *dest = f32_scratch.f;
    return 1;
}

static void put_f32_xdata(uint8_t offset, const float V2_XDATA *value)
{
    const uint8_t V2_XDATA *bytes = (const uint8_t V2_XDATA *)value;

    v2_tx_frame[offset] = bytes[V2_F32_B0];
    v2_tx_frame[offset + 1u] = bytes[V2_F32_B1];
    v2_tx_frame[offset + 2u] = bytes[V2_F32_B2];
    v2_tx_frame[offset + 3u] = bytes[V2_F32_B3];
}

static void put_f32_value(uint8_t offset, float value)
{
    f32_scratch.f = value;
    v2_tx_frame[offset] = (uint8_t)(f32_scratch.u & 0xFFUL);
    v2_tx_frame[offset + 1u] = (uint8_t)((f32_scratch.u >> 8) & 0xFFUL);
    v2_tx_frame[offset + 2u] = (uint8_t)((f32_scratch.u >> 16) & 0xFFUL);
    v2_tx_frame[offset + 3u] = (uint8_t)((f32_scratch.u >> 24) & 0xFFUL);
}

#define V2_PUT_U8(offset, value) (v2_tx_frame[(offset)] = (uint8_t)(value))

#define V2_PUT_U16(offset, value)                    \
    do                                               \
    {                                                \
        unsigned int v_ = (unsigned int)(value);     \
        v2_tx_frame[(offset)] = (uint8_t)v_;         \
        v2_tx_frame[(offset) + 1u] = (uint8_t)(v_ >> 8); \
    } while (0)

#define V2_PUT_U32(offset, value)                          \
    do                                                     \
    {                                                      \
        unsigned long v_ = (unsigned long)(value);         \
        v2_tx_frame[(offset)] = (uint8_t)v_;               \
        v2_tx_frame[(offset) + 1u] = (uint8_t)(v_ >> 8);   \
        v2_tx_frame[(offset) + 2u] = (uint8_t)(v_ >> 16);  \
        v2_tx_frame[(offset) + 3u] = (uint8_t)(v_ >> 24);  \
    } while (0)

uint16_t v2_crc16_modbus(const uint8_t V2_XDATA *data, uint16_t size)
{
    uint16_t crc = 0xFFFFu;
    uint16_t index;
    uint8_t bit;

    for (index = 0; index < size; index++)
    {
        crc ^= data[index];
        for (bit = 0; bit < 8u; bit++)
        {
            if ((crc & 1u) != 0u)
                crc = (uint16_t)((crc >> 1) ^ 0xA001u);
            else
                crc = (uint16_t)(crc >> 1);
        }
    }
    return crc;
}

uint8_t v2_posture_is_valid(uint8_t posture)
{
    return (uint8_t)(posture <= 5u);
}

uint8_t v2_sequence_is_newer(uint16_t candidate, uint16_t reference)
{
    uint16_t delta = (uint16_t)(candidate - reference);

    return (uint8_t)(delta != 0u && delta < 0x8000u);
}

uint8_t v2_sequence32_is_newer(uint32_t candidate, uint32_t reference)
{
    uint32_t delta = candidate - reference;

    return (uint8_t)(delta != 0u && delta < 0x80000000UL);
}

static uint16_t finish_frame(uint8_t type, uint16_t sequence, uint16_t payload_length)
{
    uint16_t crc;
    uint16_t index;

    v2_tx_frame[0] = V2_MAGIC0;
    v2_tx_frame[1] = V2_MAGIC1;
    v2_tx_frame[2] = V2_VERSION;
    v2_tx_frame[3] = type;
    v2_tx_frame[4] = (uint8_t)(payload_length & 0xFFu);
    v2_tx_frame[5] = (uint8_t)((payload_length >> 8) & 0xFFu);
    v2_tx_frame[6] = (uint8_t)(sequence & 0xFFu);
    v2_tx_frame[7] = (uint8_t)((sequence >> 8) & 0xFFu);

    crc = v2_crc16_modbus(v2_tx_frame + 2, (uint16_t)(6u + payload_length));

    index = (uint16_t)(V2_HEADER_SIZE + payload_length);
    v2_tx_frame[index] = (uint8_t)(crc & 0xFFu);
    v2_tx_frame[index + 1u] = (uint8_t)((crc >> 8) & 0xFFu);

    return (uint16_t)(index + V2_CRC_SIZE);
}

uint16_t v2_encode_heartbeat(uint16_t sequence)
{
    if (v2_tx_heartbeat.boot_id == 0UL)
        return 0;

    V2_PUT_U32(8, v2_tx_heartbeat.uptime_ms);
    V2_PUT_U32(12, v2_tx_heartbeat.boot_id);
    V2_PUT_U32(16, v2_tx_heartbeat.capabilities);
    V2_PUT_U8(20, (uint8_t)(v2_tx_heartbeat.ready ? 1u : 0u));

    return finish_frame(V2_TYPE_HEARTBEAT, sequence, V2_PAYLOAD_HEARTBEAT);
}

uint16_t v2_encode_gimbal_state(uint16_t sequence)
{
    /* 局部变量 static：EDATA 栈只有 89 B，编码路径处于 UART2 中断可抢占的
       深层调用里，必须避免占用栈。 */
    static float yaw;
    static float rate;

    yaw = v2_tx_gimbal.relative_yaw_rad;
    if (!V2_FINITE(yaw) || yaw < -V2_PI_F || yaw >= V2_PI_F)
        return 0;
    rate = v2_tx_gimbal.yaw_rate_rad_s;
    if (!V2_FINITE(rate))
        return 0;

    put_f32_value(8, yaw);
    put_f32_value(12, rate);
    V2_PUT_U32(16, v2_tx_gimbal.sample_sequence);
    V2_PUT_U32(20, v2_tx_gimbal.mcu_time_ms);
    V2_PUT_U8(24, (uint8_t)((v2_tx_gimbal.valid ? 1u : 0u) |
                            (v2_tx_gimbal.online ? 2u : 0u)));

    return finish_frame(V2_TYPE_GIMBAL_STATE, sequence, V2_PAYLOAD_GIMBAL_STATE);
}

uint16_t v2_encode_chassis_heading_state(uint16_t sequence)
{
    static float yaw;
    static float rate;

    yaw = v2_tx_heading.yaw_rad;
    if (!V2_FINITE(yaw) || yaw < -V2_PI_F || yaw >= V2_PI_F)
        return 0;
    rate = v2_tx_heading.yaw_rate_rad_s;
    if (!V2_FINITE(rate))
        return 0;

    put_f32_value(8, yaw);
    put_f32_value(12, rate);
    V2_PUT_U32(16, v2_tx_heading.sample_sequence);
    V2_PUT_U32(20, v2_tx_heading.mcu_time_ms);
    V2_PUT_U16(24, v2_tx_heading.reset_counter);
    V2_PUT_U8(26, (uint8_t)((v2_tx_heading.valid ? 1u : 0u) |
                            (v2_tx_heading.online ? 2u : 0u)));

    return finish_frame(V2_TYPE_CHASSIS_HEADING_STATE, sequence,
                        V2_PAYLOAD_CHASSIS_HEADING_STATE);
}

uint16_t v2_encode_chassis_attitude_state(uint16_t sequence)
{
    static float qx;
    static float qy;
    static float qz;
    static float qw;
    static uint8_t flags;

    if (v2_tx_attitude.source_boot_id == 0UL)
        return 0;

    /* 四元数做有限性与范数校验；gyro/acc 来自 int16 换算，必为有限值。 */
    qx = v2_tx_attitude.orientation_x;
    qy = v2_tx_attitude.orientation_y;
    qz = v2_tx_attitude.orientation_z;
    qw = v2_tx_attitude.orientation_w;
    if (!V2_FINITE(qx) || !V2_FINITE(qy) || !V2_FINITE(qz) || !V2_FINITE(qw))
        return 0;

    if (v2_tx_attitude.orientation_valid)
    {
        /* |norm-1| <= 1e-3 等价于 norm^2 落在 [0.998001, 1.002001]。 */
        f32_scratch.f = qx * qx + qy * qy + qz * qz + qw * qw;
        if (!V2_FINITE(f32_scratch.f) || f32_scratch.f < 0.998f ||
            f32_scratch.f > 1.002f)
            return 0;
    }

    flags = 0;
    if (v2_tx_attitude.orientation_valid)
        flags |= 1u;
    if (v2_tx_attitude.angular_velocity_valid)
        flags |= 2u;
    if (v2_tx_attitude.linear_acceleration_valid)
        flags |= 4u;
    if (v2_tx_attitude.online)
        flags |= 8u;

    put_f32_value(8, qx);
    put_f32_value(12, qy);
    put_f32_value(16, qz);
    put_f32_value(20, qw);
    put_f32_xdata(24, &v2_tx_attitude.angular_velocity_x_rad_s);
    put_f32_xdata(28, &v2_tx_attitude.angular_velocity_y_rad_s);
    put_f32_xdata(32, &v2_tx_attitude.angular_velocity_z_rad_s);
    put_f32_xdata(36, &v2_tx_attitude.linear_acceleration_x_mps2);
    put_f32_xdata(40, &v2_tx_attitude.linear_acceleration_y_mps2);
    put_f32_xdata(44, &v2_tx_attitude.linear_acceleration_z_mps2);
    V2_PUT_U32(48, v2_tx_attitude.sample_sequence);
    V2_PUT_U32(52, v2_tx_attitude.mcu_time_ms);
    V2_PUT_U32(56, v2_tx_attitude.source_boot_id);
    V2_PUT_U16(60, v2_tx_attitude.reset_counter);
    V2_PUT_U8(62, flags);

    return finish_frame(V2_TYPE_CHASSIS_ATTITUDE_STATE, sequence,
                        V2_PAYLOAD_CHASSIS_ATTITUDE_STATE);
}

/* ---------- 流解码器 ---------- */

void v2_decoder_reset(void)
{
    v2_rx_head = 0;
    v2_rx_tail = 0;
    v2_decoder_crc_errors = 0;
    v2_decoder_framing_errors = 0;
}

static void decoder_compact(void)
{
    uint16_t index = 0;

    while (v2_rx_head < v2_rx_tail)
    {
        v2_rx_buffer[index] = v2_rx_buffer[v2_rx_head];
        v2_rx_head++;
        index++;
    }
    v2_rx_head = 0;
    v2_rx_tail = index;
}

void v2_decoder_feed(uint8_t byte)
{
    if (v2_rx_tail >= V2_DECODER_BUFFER_SIZE)
        decoder_compact();

    if (v2_rx_tail >= V2_DECODER_BUFFER_SIZE)
    {
        v2_decoder_framing_errors++;
        return;
    }

    v2_rx_buffer[v2_rx_tail] = byte;
    v2_rx_tail++;
}

uint8_t v2_decoder_next(void)
{
    const uint8_t V2_XDATA *cursor;
    uint16_t length;
    uint16_t search;
    uint16_t keep_from;
    uint16_t drop_count;
    uint16_t payload_length;
    uint16_t total;
    uint16_t expected_crc;

    length = (uint16_t)(v2_rx_tail - v2_rx_head);
    if (length < 2u)
        return V2_DECODE_NEED_MORE;

    search = v2_rx_head;
    while ((uint16_t)(search + 1u) < v2_rx_tail)
    {
        if (v2_rx_buffer[search] == V2_MAGIC0 &&
            v2_rx_buffer[search + 1u] == V2_MAGIC1)
            break;
        search++;
    }

    if ((uint16_t)(search + 1u) >= v2_rx_tail)
    {
        /* 找不到相邻 52 4d：只保留末尾可能是前导 0x52 的一个字节。 */
        keep_from = v2_rx_tail;
        if (v2_rx_buffer[v2_rx_tail - 1u] == V2_MAGIC0)
            keep_from = (uint16_t)(v2_rx_tail - 1u);

        drop_count = (uint16_t)(keep_from - v2_rx_head);
        v2_rx_head = keep_from;
        v2_decoder_framing_errors += drop_count;
        return (drop_count != 0u) ? V2_DECODE_INVALID : V2_DECODE_NEED_MORE;
    }

    if (search > v2_rx_head)
    {
        v2_decoder_framing_errors += (uint16_t)(search - v2_rx_head);
        v2_rx_head = search;
    }

    length = (uint16_t)(v2_rx_tail - v2_rx_head);
    if (length < V2_HEADER_SIZE)
        return V2_DECODE_NEED_MORE;

    cursor = v2_rx_buffer + v2_rx_head;

    if (cursor[2] != V2_VERSION)
    {
        v2_rx_head++;
        v2_decoder_framing_errors++;
        return V2_DECODE_INVALID;
    }

    copy_u16_le_data(cursor + 4, (uint8_t V2_DATA *)&rx_scratch_u16);
    payload_length = rx_scratch_u16;
    if (payload_length > V2_MAX_PAYLOAD_SIZE)
    {
        v2_rx_head++;
        v2_decoder_framing_errors++;
        return V2_DECODE_INVALID;
    }

    total = (uint16_t)(V2_HEADER_SIZE + payload_length + V2_CRC_SIZE);
    if (length < total)
        return V2_DECODE_NEED_MORE;

    expected_crc = v2_crc16_modbus(cursor + 2, (uint16_t)(6u + payload_length));
    copy_u16_le_data(cursor + V2_HEADER_SIZE + payload_length,
                     (uint8_t V2_DATA *)&rx_scratch_u16);
    if (expected_crc != rx_scratch_u16)
    {
        v2_rx_head++;
        v2_decoder_crc_errors++;
        return V2_DECODE_INVALID;
    }

    v2_rx_type = cursor[3];
    copy_u16_le_xdata(cursor + 6, (uint8_t V2_XDATA *)&v2_rx_sequence);

    /* 完整消费本帧，再解码到全局量。 */
    v2_rx_head = (uint16_t)(v2_rx_head + total);
    cursor += V2_HEADER_SIZE;

    switch (v2_rx_type)
    {
    case V2_TYPE_HEARTBEAT:
        if (payload_length != V2_PAYLOAD_HEARTBEAT)
            return V2_DECODE_BAD_PAYLOAD;
        copy_u32_le_xdata(cursor, (uint8_t V2_XDATA *)&v2_rx_heartbeat.uptime_ms);
        copy_u32_le_xdata(cursor + 4, (uint8_t V2_XDATA *)&v2_rx_heartbeat.boot_id);
        copy_u32_le_xdata(cursor + 8, (uint8_t V2_XDATA *)&v2_rx_heartbeat.capabilities);
        v2_rx_heartbeat.ready = (uint8_t)(cursor[12] & 1u);
        return V2_DECODE_OK;

    case V2_TYPE_CHASSIS_COMMAND:
        if (payload_length != V2_PAYLOAD_CHASSIS_COMMAND)
            return V2_DECODE_BAD_PAYLOAD;
        if (!copy_f32_le_xdata(cursor, &v2_rx_chassis_command.vx_mps))
            return V2_DECODE_BAD_PAYLOAD;
        if (!copy_f32_le_xdata(cursor + 4, &v2_rx_chassis_command.vy_mps))
            return V2_DECODE_BAD_PAYLOAD;
        if (!copy_f32_le_xdata(cursor + 8, &v2_rx_chassis_command.wz_rad_s))
            return V2_DECODE_BAD_PAYLOAD;
        v2_rx_chassis_command.enabled = (uint8_t)(cursor[12] & 1u);
        return V2_DECODE_OK;

    case V2_TYPE_POSTURE_REQUEST:
        if (payload_length != V2_PAYLOAD_POSTURE_REQUEST)
            return V2_DECODE_BAD_PAYLOAD;
        /* 无姿态执行器：只做合法性校验并计数（link 层处理），不保存字段。 */
        if (!v2_posture_is_valid(cursor[8]))
            return V2_DECODE_BAD_PAYLOAD;
        return V2_DECODE_OK;

    default:
        return V2_DECODE_UNKNOWN;
    }
}
