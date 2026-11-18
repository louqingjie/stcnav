/*
 * V2 协议自检：golden vector、帧/CRC、流重同步、拒绝路径。
 *
 * golden 来源：apps/vehicle/docs/stm_v2_protocol_reference.md 第 10 节，
 * 以及 docs/test_competition_v2_reference.cpp 中的 ChassisCommand 向量。
 *
 * 注意：const 数组在 MCS251 上位于 code 空间，只能直接下标访问；
 * 不要把它们的地址转成通用指针（通用指针不携带地址空间标签）。
 */

#include "v2_selftest.h"
#include "v2_codec.h"

/* 0x01 ChassisCommand：seq=0x1234，vx=1.0，vy=-2.0，wz=0.5，enabled=1。 */
static const uint8_t golden_chassis_command[23] = {
    0x52, 0x4d, 0x02, 0x01, 0x0d, 0x00, 0x34, 0x12,
    0x00, 0x00, 0x80, 0x3f,
    0x00, 0x00, 0x00, 0xc0,
    0x00, 0x00, 0x00, 0x3f,
    0x01, 0xe6, 0xcb};

/* 0x88 ChassisAttitudeState golden（文档第 10 节，完整 65 字节）。 */
static const uint8_t golden_attitude[65] = {
    0x52, 0x4d, 0x02, 0x88, 0x37, 0x00, 0x44, 0x33,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3f,
    0x00, 0x00, 0x00, 0x3e, 0x00, 0x00, 0x80, 0xbe,
    0x00, 0x00, 0x00, 0x3f, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x0a, 0xe8, 0x1c, 0x41,
    0x04, 0x03, 0x02, 0x01, 0xe8, 0x03, 0x00, 0x00,
    0x01, 0x00, 0x27, 0x20, 0x02, 0x00, 0x0f, 0x90,
    0xbe};

static uint8_t frame_equal(const uint8_t golden[], uint16_t length)
{
    uint16_t index;

    for (index = 0; index < length; index++)
    {
        if (v2_tx_frame[index] != golden[index])
            return 0;
    }
    return 1;
}

static uint8_t payload_equal(const uint8_t expected[], uint16_t length)
{
    uint16_t index;

    for (index = 0; index < length; index++)
    {
        if (v2_tx_frame[V2_HEADER_SIZE + index] != expected[index])
            return 0;
    }
    return 1;
}

static uint8_t feed_frame(const uint8_t frame[], uint16_t length)
{
    uint16_t index;

    v2_decoder_reset();
    for (index = 0; index < length; index++)
        v2_decoder_feed(frame[index]);
    return v2_decoder_next();
}

unsigned char v2_selftest(void)
{
    uint16_t size;
    uint16_t index;

    /* 1. 解码 0x01 golden：先给 7 字节应 NEED_MORE，再补齐后应成功。 */
    v2_decoder_reset();
    for (index = 0; index < (V2_HEADER_SIZE - 1u); index++)
        v2_decoder_feed(golden_chassis_command[index]);
    if (v2_decoder_next() != V2_DECODE_NEED_MORE)
        return 1;
    for (; index < 23u; index++)
        v2_decoder_feed(golden_chassis_command[index]);
    if (v2_decoder_next() != V2_DECODE_OK)
        return 1;
    if (v2_rx_type != V2_TYPE_CHASSIS_COMMAND || v2_rx_sequence != 0x1234u)
        return 1;
    if (v2_rx_chassis_command.vx_mps != 1.0f ||
        v2_rx_chassis_command.vy_mps != -2.0f ||
        v2_rx_chassis_command.wz_rad_s != 0.5f ||
        v2_rx_chassis_command.enabled != 1u)
        return 1;

    /* 2. 0x88 golden 编码必须逐字节一致。 */
    v2_tx_attitude.orientation_x = 0.0f;
    v2_tx_attitude.orientation_y = 0.0f;
    v2_tx_attitude.orientation_z = 0.0f;
    v2_tx_attitude.orientation_w = 1.0f;
    v2_tx_attitude.angular_velocity_x_rad_s = 0.125f;
    v2_tx_attitude.angular_velocity_y_rad_s = -0.25f;
    v2_tx_attitude.angular_velocity_z_rad_s = 0.5f;
    v2_tx_attitude.linear_acceleration_x_mps2 = 0.0f;
    v2_tx_attitude.linear_acceleration_y_mps2 = 0.0f;
    v2_tx_attitude.linear_acceleration_z_mps2 = 9.80665f;
    v2_tx_attitude.sample_sequence = 0x01020304UL;
    v2_tx_attitude.mcu_time_ms = 1000UL;
    v2_tx_attitude.source_boot_id = 0x20270001UL;
    v2_tx_attitude.reset_counter = 2u;
    v2_tx_attitude.orientation_valid = 1u;
    v2_tx_attitude.angular_velocity_valid = 1u;
    v2_tx_attitude.linear_acceleration_valid = 1u;
    v2_tx_attitude.online = 1u;

    size = v2_encode_chassis_attitude_state(0x3344u);
    if (size != 65u)
        return 2;
    if (!frame_equal(golden_attitude, 65u))
        return 2;

    /* 3. 非法 0x88：四元数范数超差 / boot=0 / NaN 必须拒绝。 */
    v2_tx_attitude.orientation_w = 0.5f;
    if (v2_encode_chassis_attitude_state(0u) != 0u)
        return 3;
    v2_tx_attitude.orientation_w = 1.0f;
    v2_tx_attitude.source_boot_id = 0UL;
    if (v2_encode_chassis_attitude_state(0u) != 0u)
        return 3;
    v2_tx_attitude.source_boot_id = 0x20270001UL;
    {
        union
        {
            float f;
            unsigned long u;
        } not_a_number;

        /* 四元数出现 NaN：范数校验必须拒绝（gyro/acc 由 int16 换算，必为有限值）。 */
        not_a_number.u = 0x7FC00000UL;
        v2_tx_attitude.orientation_w = not_a_number.f;
        if (v2_encode_chassis_attitude_state(0u) != 0u)
            return 3;
        v2_tx_attitude.orientation_w = 1.0f;
    }

    /* 4. 心跳往返（23 字节帧）。 */
    v2_tx_heartbeat.uptime_ms = 99UL;
    v2_tx_heartbeat.boot_id = 0xABCDEF01UL;
    v2_tx_heartbeat.capabilities = 0x00000125UL;
    v2_tx_heartbeat.ready = 1u;
    size = v2_encode_heartbeat(7u);
    if (size != (V2_HEADER_SIZE + V2_PAYLOAD_HEARTBEAT + V2_CRC_SIZE))
        return 4;
    if (feed_frame(v2_tx_frame, size) != V2_DECODE_OK)
        return 4;
    if (v2_rx_type != V2_TYPE_HEARTBEAT || v2_rx_sequence != 7u)
        return 4;
    if (v2_rx_heartbeat.uptime_ms != 99UL || v2_rx_heartbeat.boot_id != 0xABCDEF01UL ||
        v2_rx_heartbeat.capabilities != 0x00000125UL || v2_rx_heartbeat.ready != 1u)
        return 4;

    /* 5. 0x82 云台帧 payload 字节级校验。 */
    v2_tx_gimbal.relative_yaw_rad = 0.5f;
    v2_tx_gimbal.yaw_rate_rad_s = -0.25f;
    v2_tx_gimbal.sample_sequence = 0x01020304UL;
    v2_tx_gimbal.mcu_time_ms = 1000UL;
    v2_tx_gimbal.valid = 1u;
    v2_tx_gimbal.online = 1u;
    size = v2_encode_gimbal_state(3u);
    if (size != (V2_HEADER_SIZE + V2_PAYLOAD_GIMBAL_STATE + V2_CRC_SIZE))
        return 5;
    {
        static const uint8_t expected[V2_PAYLOAD_GIMBAL_STATE] = {
            0x00, 0x00, 0x00, 0x3f,
            0x00, 0x00, 0x80, 0xbe,
            0x04, 0x03, 0x02, 0x01,
            0xe8, 0x03, 0x00, 0x00,
            0x03};
        if (!payload_equal(expected, V2_PAYLOAD_GIMBAL_STATE))
            return 5;
        if (feed_frame(v2_tx_frame, size) != V2_DECODE_UNKNOWN)
            return 5;
        if (v2_rx_type != V2_TYPE_GIMBAL_STATE || v2_rx_sequence != 3u)
            return 5;
    }

    /* 6. 0x85 航向帧 payload 字节级校验。 */
    v2_tx_heading.yaw_rad = 0.5f;
    v2_tx_heading.yaw_rate_rad_s = -1.25f;
    v2_tx_heading.sample_sequence = 0x10203040UL;
    v2_tx_heading.mcu_time_ms = 4321UL;
    v2_tx_heading.reset_counter = 17u;
    v2_tx_heading.valid = 1u;
    v2_tx_heading.online = 1u;
    size = v2_encode_chassis_heading_state(4u);
    if (size != (V2_HEADER_SIZE + V2_PAYLOAD_CHASSIS_HEADING_STATE + V2_CRC_SIZE))
        return 6;
    {
        static const uint8_t expected[V2_PAYLOAD_CHASSIS_HEADING_STATE] = {
            0x00, 0x00, 0x00, 0x3f,
            0x00, 0x00, 0xa0, 0xbf,
            0x40, 0x30, 0x20, 0x10,
            0xe1, 0x10, 0x00, 0x00,
            0x11, 0x00,
            0x03};
        if (!payload_equal(expected, V2_PAYLOAD_CHASSIS_HEADING_STATE))
            return 6;
        if (feed_frame(v2_tx_frame, size) != V2_DECODE_UNKNOWN)
            return 6;
        if (v2_rx_type != V2_TYPE_CHASSIS_HEADING_STATE || v2_rx_sequence != 4u)
            return 6;
    }

    /* 7. yaw 范围与非法输入拒绝。 */
    v2_tx_gimbal.relative_yaw_rad = 3.14159265358979323846f;
    if (v2_encode_gimbal_state(0u) != 0u)
        return 7;
    v2_tx_gimbal.relative_yaw_rad = 0.0f;
    v2_tx_heading.yaw_rad = -4.0f;
    if (v2_encode_chassis_heading_state(0u) != 0u)
        return 7;
    v2_tx_heartbeat.boot_id = 0UL;
    if (v2_encode_heartbeat(0u) != 0u)
        return 7;
    v2_tx_heartbeat.boot_id = 0xABCDEF01UL;

    /* 8. CRC 破坏：INVALID 且只丢 1 字节，随后有效帧仍可同步。 */
    size = v2_encode_heartbeat(8u);
    if (size == 0u)
        return 8;
    v2_tx_frame[size - 1u] = (uint8_t)(v2_tx_frame[size - 1u] ^ 0x80u);
    v2_decoder_reset();
    for (index = 0; index < size; index++)
        v2_decoder_feed(v2_tx_frame[index]);
    if (v2_decoder_next() != V2_DECODE_INVALID)
        return 8;
    if (v2_decoder_crc_errors != 1u)
        return 8;
    v2_tx_frame[size - 1u] = (uint8_t)(v2_tx_frame[size - 1u] ^ 0x80u);
    for (index = 0; index < size; index++)
        v2_decoder_feed(v2_tx_frame[index]);
    if (v2_decoder_next() != V2_DECODE_OK)
        return 8;

    /* 9. 未知但 CRC 正确的类型：完整消费并返回类型。 */
    size = v2_encode_heartbeat(9u);
    v2_tx_frame[3] = 0x55u;
    {
        uint16_t crc;

        crc = v2_crc16_modbus(v2_tx_frame + 2, 6u + V2_PAYLOAD_HEARTBEAT);
        v2_tx_frame[size - 2u] = (uint8_t)(crc & 0xFFu);
        v2_tx_frame[size - 1u] = (uint8_t)((crc >> 8) & 0xFFu);
    }
    if (feed_frame(v2_tx_frame, size) != V2_DECODE_UNKNOWN)
        return 9;
    if (v2_rx_type != 0x55u)
        return 9;

    /* 10. 纯噪声：帧同步失败计数。 */
    v2_decoder_reset();
    v2_decoder_feed(0x00u);
    v2_decoder_feed(0x11u);
    if (v2_decoder_next() != V2_DECODE_INVALID)
        return 10;
    if (v2_decoder_framing_errors != 2u)
        return 10;

    /* 11. 序号比较与 posture 取值。 */
    if (!v2_sequence_is_newer(0u, 0xFFFFu))
        return 11;
    if (v2_sequence_is_newer(0xFFFFu, 0u))
        return 11;
    if (!v2_sequence32_is_newer(0u, 0xFFFFFFFFUL))
        return 11;
    if (v2_sequence32_is_newer(0xFFFFFFFFUL, 0u))
        return 11;
    for (index = 0; index <= 5u; index++)
    {
        if (!v2_posture_is_valid((uint8_t)index))
            return 11;
    }
    if (v2_posture_is_valid(6u))
        return 11;

    return 0;
}
