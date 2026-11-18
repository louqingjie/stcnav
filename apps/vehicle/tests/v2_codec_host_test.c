/*
 * 宿主机（gcc）侧 V2 codec 测试：复用固件自检，并补充边界用例。
 *
 * 运行：
 *   cc -std=c11 -O2 -Wall -Wextra -Iapps/vehicle \
 *      apps/vehicle/v2_codec.c apps/vehicle/v2_selftest.c \
 *      apps/vehicle/tests/v2_codec_host_test.c -o /tmp/v2_codec_test
 *   /tmp/v2_codec_test
 */

#include <stdio.h>
#include <string.h>

#include "v2_codec.h"
#include "v2_selftest.h"

static int failures;

static void expect(int condition, const char *message)
{
    if (!condition)
    {
        failures++;
        printf("FAIL: %s\n", message);
    }
}

int main(void)
{
    unsigned char step;
    unsigned char result;
    v2_heartbeat_t heartbeat;
    v2_chassis_heading_state_t heading;
    v2_chassis_command_t command;
    uint8_t buffer[V2_MAX_FRAME_SIZE];
    uint16_t size;
    uint16_t index;

    step = v2_selftest();
    if (step != 0u)
    {
        printf("FAIL: golden self-test step %u\n", (unsigned int)step);
        return 1;
    }
    printf("PASS: golden self-test\n");

    /* 边界：yaw == -PI 合法，+PI 非法（wire 要求 [-PI, PI)）。 */
    heading.yaw_rad = -3.14159265358979323846f;
    heading.yaw_rate_rad_s = 0.0f;
    heading.sample_sequence = 1UL;
    heading.mcu_time_ms = 2UL;
    heading.reset_counter = 3u;
    heading.valid = 1u;
    heading.online = 1u;
    v2_tx_heading = heading;
    expect(v2_encode_chassis_heading_state(1u) != 0u, "-PI yaw should encode");

    heading.yaw_rad = 3.14159265358979323846f;
    v2_tx_heading = heading;
    expect(v2_encode_chassis_heading_state(1u) == 0u, "+PI yaw should be rejected");

    /* 逐字节喂入心跳帧，检查流解码器分片行为。 */
    heartbeat.uptime_ms = 12345UL;
    heartbeat.boot_id = 0x01020304UL;
    heartbeat.capabilities = 0x125UL;
    heartbeat.ready = 1u;
    v2_tx_heartbeat = heartbeat;
    size = v2_encode_heartbeat(0xABCDu);
    expect(size == 23u, "heartbeat frame size");
    memcpy(buffer, v2_tx_frame, size);

    v2_decoder_reset();
    for (index = 0; index < size; index++)
    {
        v2_decoder_feed(buffer[index]);
        result = v2_decoder_next();
        if (index + 1u < size)
            expect(result == V2_DECODE_NEED_MORE, "fragmented frame should need more");
        else
            expect(result == V2_DECODE_OK, "decoder yields frame");
    }
    expect(v2_rx_type == V2_TYPE_HEARTBEAT, "heartbeat type");
    expect(v2_rx_sequence == 0xABCDu, "heartbeat sequence");
    expect(v2_rx_heartbeat.uptime_ms == 12345UL, "heartbeat uptime");
    expect(v2_rx_heartbeat.boot_id == 0x01020304UL, "heartbeat boot");
    expect(v2_rx_heartbeat.capabilities == 0x125UL, "heartbeat capabilities");
    expect(v2_rx_heartbeat.ready == 1u, "heartbeat ready");

    /* 手工构造 NaN 的 0x01 帧：解码器必须报 BAD_PAYLOAD。 */
    {
        const uint8_t nan_frame[23] = {
            0x52, 0x4d, 0x02, 0x01, 0x0d, 0x00, 0x01, 0x00,
            0x00, 0x00, 0xc0, 0x7f,
            0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00,
            0x01};
        uint16_t crc;

        memcpy(buffer, nan_frame, 21);
        crc = v2_crc16_modbus(buffer + 2, 19u);
        buffer[21] = (uint8_t)(crc & 0xFFu);
        buffer[22] = (uint8_t)(crc >> 8);
        v2_decoder_reset();
        for (index = 0; index < 23u; index++)
            v2_decoder_feed(buffer[index]);
        expect(v2_decoder_next() == V2_DECODE_BAD_PAYLOAD, "NaN command rejected");
    }

    /* 非法帧长直接拒绝。 */
    {
        const uint8_t short_frame[22] = {
            0x52, 0x4d, 0x02, 0x03, 0x0c, 0x00, 0x07, 0x00,
            0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
            0x09, 0x0a, 0x0b, 0x0c, 0x00, 0x00};
        uint16_t crc;

        memcpy(buffer, short_frame, 22);
        crc = v2_crc16_modbus(buffer + 2, 18u);
        buffer[20] = (uint8_t)(crc & 0xFFu);
        buffer[21] = (uint8_t)(crc >> 8);
        v2_decoder_reset();
        for (index = 0; index < 22u; index++)
            v2_decoder_feed(buffer[index]);
        expect(v2_decoder_next() == V2_DECODE_BAD_PAYLOAD, "12-byte heartbeat rejected");
    }

    /* 未知但 CRC 正确的类型：完整消费并返回 UNKNOWN。 */
    v2_tx_heartbeat = heartbeat;
    size = v2_encode_heartbeat(9u);
    memcpy(buffer, v2_tx_frame, size);
    buffer[3] = 0x55u;
    {
        uint16_t crc = v2_crc16_modbus(buffer + 2, 19u);

        buffer[size - 2u] = (uint8_t)(crc & 0xFFu);
        buffer[size - 1u] = (uint8_t)(crc >> 8);
    }
    v2_decoder_reset();
    for (index = 0; index < size; index++)
        v2_decoder_feed(buffer[index]);
    expect(v2_decoder_next() == V2_DECODE_UNKNOWN, "unknown type consumed");
    expect(v2_rx_type == 0x55u, "unknown type value");

    /* 手工构造合法 0x01 帧并解码。 */
    {
        const uint8_t command_frame[23] = {
            0x52, 0x4d, 0x02, 0x01, 0x0d, 0x00, 0x34, 0x12,
            0x00, 0x00, 0x80, 0x3f,
            0x00, 0x00, 0x00, 0xc0,
            0x00, 0x00, 0x00, 0x3f,
            0x01, 0xe6, 0xcb};

        v2_decoder_reset();
        for (index = 0; index < 23u; index++)
            v2_decoder_feed(command_frame[index]);
        expect(v2_decoder_next() == V2_DECODE_OK, "command decode ok");
        command = v2_rx_chassis_command;
        expect(command.vx_mps == 1.0f, "command vx");
        expect(command.vy_mps == -2.0f, "command vy");
        expect(command.wz_rad_s == 0.5f, "command wz");
        expect(command.enabled == 1u, "command enabled");
    }

    if (failures != 0)
    {
        printf("FAIL: %d host-side checks\n", failures);
        return 1;
    }

    printf("PASS: host codec tests\n");
    return 0;
}
