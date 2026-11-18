/*
 * RM2027 STM <-> Host V2 协议（Host Freeze v1）wire 编解码 —— 紧凑实现。
 *
 * 语义依据：apps/vehicle/docs/stm_v2_protocol_reference.md 与
 * apps/vehicle/docs/competition_v2_codec.{h,c}（Host 侧 C11 参考实现）。
 *
 * 本文件只实现整车固件实际需要的子集：
 *   RX: 0x01 ChassisCommand、0x02 PostureRequest、0x03 Heartbeat
 *   TX: 0x03 Heartbeat、0x82 GimbalState、0x85 ChassisHeadingState、
 *       0x88 ChassisAttitudeState
 *
 * MCS251 代码生成特性：通过 XDATA 全局量 + 常量偏移读写，比把大结构体
 * 通过通用指针传入函数小得多。因此：
 *   - TX 消息由本模块持有（v2_tx_*），调用方直接写字段；
 *   - RX 解码结果写入 v2_rx_* 全局量；
 *   - 帧缓冲 v2_tx_frame / v2_rx_buffer 固定在本模块。
 *
 * 该文件不依赖任何 MCU 头文件，可被宿主机 gcc 直接编译用于 golden vector 测试。
 */

#ifndef STCNAV_VEHICLE_V2_CODEC_H
#define STCNAV_VEHICLE_V2_CODEC_H

#include <stdint.h>

#if defined(__SDCC_mcs251)
#define V2_XDATA __xdata
#else
#define V2_XDATA
#endif

#define V2_MAGIC0 0x52u
#define V2_MAGIC1 0x4Du
#define V2_VERSION 2u

#define V2_HEADER_SIZE 8u
#define V2_CRC_SIZE 2u
#define V2_MAX_PAYLOAD_SIZE 128u
#define V2_MAX_FRAME_SIZE (V2_HEADER_SIZE + V2_MAX_PAYLOAD_SIZE + V2_CRC_SIZE)
#define V2_DECODER_BUFFER_SIZE 144u

#define V2_TYPE_CHASSIS_COMMAND 0x01u
#define V2_TYPE_POSTURE_REQUEST 0x02u
#define V2_TYPE_HEARTBEAT 0x03u
#define V2_TYPE_GIMBAL_STATE 0x82u
#define V2_TYPE_CHASSIS_HEADING_STATE 0x85u
#define V2_TYPE_CHASSIS_ATTITUDE_STATE 0x88u

#define V2_PAYLOAD_CHASSIS_COMMAND 13u
#define V2_PAYLOAD_POSTURE_REQUEST 11u
#define V2_PAYLOAD_HEARTBEAT 13u
#define V2_PAYLOAD_GIMBAL_STATE 17u
#define V2_PAYLOAD_CHASSIS_HEADING_STATE 19u
#define V2_PAYLOAD_CHASSIS_ATTITUDE_STATE 55u

/* v2_decoder_next() 返回值 */
#define V2_DECODE_NEED_MORE 0u
#define V2_DECODE_OK 1u
#define V2_DECODE_INVALID 2u      /* 帧层错误，已丢弃 1 字节 */
#define V2_DECODE_UNKNOWN 3u      /* CRC 正确的未知类型，已完整消费 */
#define V2_DECODE_BAD_PAYLOAD 4u  /* 已知类型但载荷无效，已完整消费 */

typedef struct
{
    float vx_mps;
    float vy_mps;
    float wz_rad_s;
    uint8_t enabled;
} v2_chassis_command_t;

typedef struct
{
    uint32_t uptime_ms;
    uint32_t boot_id;
    uint32_t capabilities;
    uint8_t ready;
} v2_heartbeat_t;

typedef struct
{
    float relative_yaw_rad;
    float yaw_rate_rad_s;
    uint32_t sample_sequence;
    uint32_t mcu_time_ms;
    uint8_t valid;
    uint8_t online;
} v2_gimbal_state_t;

typedef struct
{
    float yaw_rad;
    float yaw_rate_rad_s;
    uint32_t sample_sequence;
    uint32_t mcu_time_ms;
    uint16_t reset_counter;
    uint8_t valid;
    uint8_t online;
} v2_chassis_heading_state_t;

typedef struct
{
    float orientation_x;
    float orientation_y;
    float orientation_z;
    float orientation_w;
    float angular_velocity_x_rad_s;
    float angular_velocity_y_rad_s;
    float angular_velocity_z_rad_s;
    float linear_acceleration_x_mps2;
    float linear_acceleration_y_mps2;
    float linear_acceleration_z_mps2;
    uint32_t sample_sequence;
    uint32_t mcu_time_ms;
    uint32_t source_boot_id;
    uint16_t reset_counter;
    uint8_t orientation_valid;
    uint8_t angular_velocity_valid;
    uint8_t linear_acceleration_valid;
    uint8_t online;
} v2_chassis_attitude_state_t;

/* ---------- TX ---------- */
extern v2_heartbeat_t V2_XDATA v2_tx_heartbeat;
extern v2_gimbal_state_t V2_XDATA v2_tx_gimbal;
extern v2_chassis_heading_state_t V2_XDATA v2_tx_heading;
extern v2_chassis_attitude_state_t V2_XDATA v2_tx_attitude;
extern uint8_t V2_XDATA v2_tx_frame[V2_MAX_FRAME_SIZE];

/* 返回完整帧长度（写入 v2_tx_frame），0 表示字段非法被拒绝。 */
uint16_t v2_encode_heartbeat(uint16_t sequence);
uint16_t v2_encode_gimbal_state(uint16_t sequence);
uint16_t v2_encode_chassis_heading_state(uint16_t sequence);
uint16_t v2_encode_chassis_attitude_state(uint16_t sequence);

/* ---------- RX ---------- */
extern v2_chassis_command_t V2_XDATA v2_rx_chassis_command;
extern v2_heartbeat_t V2_XDATA v2_rx_heartbeat;
extern uint8_t V2_XDATA v2_rx_type;
extern uint16_t V2_XDATA v2_rx_sequence;
extern unsigned int v2_decoder_crc_errors;
extern unsigned int v2_decoder_framing_errors;

void v2_decoder_reset(void);
void v2_decoder_feed(uint8_t byte);
uint8_t v2_decoder_next(void);

uint8_t v2_posture_is_valid(uint8_t posture);
uint8_t v2_sequence_is_newer(uint16_t candidate, uint16_t reference);
uint8_t v2_sequence32_is_newer(uint32_t candidate, uint32_t reference);
uint16_t v2_crc16_modbus(const uint8_t V2_XDATA *data, uint16_t size);

#endif
