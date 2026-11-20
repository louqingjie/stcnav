/*
 * V2 上位机链路层：UART2 收发、帧调度、宿主命令/心跳校验、运动授权。
 *
 * 接线：P1.0 = RxD2（接上位机 TX），P1.1 = TxD2（接上位机 RX），230400 8N1。
 * 与 UART1（拓展板/调参）共用 Timer2 波特率发生器。
 *
 * 线程模型：主循环调用 v2_link_poll()；UART2 中断只做字节搬运。
 */

#ifndef STCNAV_VEHICLE_V2_LINK_H
#define STCNAV_VEHICLE_V2_LINK_H

typedef struct
{
    unsigned long boot_id;
    unsigned int rx_frames;
    unsigned int rx_crc_errors;
    unsigned int rx_framing_errors;
    unsigned int rx_bad_payload;
    unsigned int rx_rejected;
    unsigned int rx_unknown_types;
    unsigned int rx_host_boot_changes;
    unsigned int rx_host_timeouts;
    unsigned int posture_requests;
    unsigned int tx_frames;
    unsigned int tx_drops;
    unsigned int tx_overflows;
    unsigned int tx_stalls;
    unsigned char boot_persist_fault;
    unsigned char host_online;
    unsigned char command_fresh;
    unsigned char command_enabled;
    unsigned char ready;
} v2_diag_t;

/* 上电时读取并递增保留扇区里的 boot 计数；返回非零 boot_id。 */
unsigned long v2_boot_id_load(void);

void v2_link_init(unsigned long boot_id, unsigned int now_ms);
void v2_link_poll(unsigned int now_ms, unsigned char imu_ok);

void v2_link_set_ready(unsigned char ready);

/*
 * V2 运动授权：仅当 IMU 正常、上位机心跳新鲜、命令新鲜且 enabled 时返回 1。
 * vx/vy 为 base 系 m/s（+x 前、+y 左），wz 为绕 +z 的 rad/s。
 */
unsigned char v2_motion_take(unsigned int now_ms, unsigned char imu_ok,
                             float *vx_mps, float *vy_mps, float *wz_rad_s);

const v2_diag_t *v2_link_diagnostics(void);

#endif
