# RM2027 STM V2 协议 —— 整车固件实现说明（STM 侧）

状态：**STM 侧已实现 / Host 联签与真机 Phase 1A 证据仍待补齐**

wire 语义以 `stm_v2_protocol_reference.md`（Host Freeze v1）与
`competition_v2_codec.{h,c}` / `competition_v2_protocol.{hpp,cpp}` 为权威来源；
本文只描述本仓库 `apps/vehicle` 的 STM 侧实现、约定与限制，不修改 Host 合同。

---

## 1. 接线与物理层

| 项目 | 值 |
| --- | --- |
| STM 串口 | UART2 |
| RX | **P1.0（RxD2，高阻输入）** ← 上位机 TX |
| TX | **P1.1（TxD2，推挽输出）** → 上位机 RX |
| 波特率 | **230400 8N1**（`config.h: V2_UART_BAUD`） |
| GND | 必须共地 |
| 电平 | 3.3 V TTL（不要直接接 RS-232） |

约束：UART2 的波特率由 **Timer2** 产生，而 UART1（拓展板 / 调参）同样使用
Timer2，因此 **UART1 与 UART2 必须保持同一波特率 230400**；修改
`V2_UART_BAUD` 时必须同步修改拓展板/调参波特率。

## 2. 实现的消息矩阵与 capability

`0x03 Heartbeat` payload 中的 `capabilities = 0x00000125`：

| bit | 含义 | 本固件 |
| --- | --- | --- |
| 0 | `0x01 ChassisCommand` | 置位（解析并在授权条件下驱动底盘） |
| 1 | posture | **不置位**（整车没有姿态执行器） |
| 2 | `0x82 GimbalState` | 置位 |
| 3 | referee | 不置位（无裁判系统接口） |
| 4 | operator navigation target | 不置位（无操作手目标源） |
| 5 | `0x85 ChassisHeadingState` | 置位 |
| 8 | `0x88 ChassisAttitudeState` | 置位（六轴姿态，见第 5 节） |

未实现的类型：`0x81 PostureState`、`0x83 RefereeState`、`0x84 OperatorNavigationTarget`、
`0x86/0x87`（radar / operator target 候选）——不发送、不置 capability；
收到 CRC 正确的这类帧时完整消费并计数（`rx_unknown_types`）。

接收侧实现：`0x01`、`0x02`（仅解码/合法性校验并计数，见 6.3）、`0x03`。

## 3. 发送调度与带宽

| 消息 | 周期 | 频率 | 帧长 |
| --- | --- | --- | --- |
| `0x03 Heartbeat` | 100 ms | 10 Hz | 23 B |
| `0x82 GimbalState` | 20 ms | 50 Hz | 27 B |
| `0x85 ChassisHeadingState` | 20 ms | 50 Hz | 29 B |
| `0x88 ChassisAttitudeState` | 20 ms | 50 Hz | 65 B |

总带宽约 7.5 KB/s（230400 上限约 23 KB/s）。状态帧仅在 `ready=1` 且本周期 IMU
读取成功时发送；heartbeat 始终发送。发送使用 UART2 中断 + 256 B TX/RX 环形缓冲，
整帧入队失败则丢弃并计数（`tx_drops`），**不阻塞控制循环**，心跳优先。

每种消息类型使用独立的 16 位 frame sequence，成功入队后才递增。

## 4. 接收校验与生命周期

- 帧层：`52 4D 02 + type + len + seq + payload + CRC16/Modbus(LE)`；版本/长度/CRC
  错误时丢弃 1 字节并重扫相邻 `52 4D`（与 Host 参考实现一致）。
- `0x03`（Host heartbeat）：`boot_id != 0`；同 boot 时 frame sequence 必须更新；
  `uptime_ms` 按模 2^32 不得倒退（相等允许）；Host boot 变化会清空命令有效性、
  序号基线与在线状态，并计入 `rx_host_boot_changes`。500 ms 无新 heartbeat 视为
  离线并撤销 V2 运动授权（`rx_host_timeouts`）。
- `0x01`：要求已经收到 Host heartbeat；13 B；三个 float 必须有限；按类型序号
  去重/丢弃乱序；`vx/vy` 夹到 `±2.0 m/s`、`wz` 夹到 `±6.283 rad/s`；命令新鲜度
  **200 ms**。`enabled` 为 bit0。
- `0x02`：11 B、posture 取值 0..5；本固件没有姿态执行器，因此只做合法性校验与
  计数（`posture_requests`），不发送 `0x81`、不置 capability bit1。
- `0x82/0x85/0x88` 等 STM→Host 类型若被 Host 误发，按未知类型消费计数。
- `sample_sequence` 每个真实新样本递增，`mcu_time_ms = millis32()`（与 heartbeat
  `uptime_ms` 同一时钟域）。IMU 重新初始化时 `reset_counter`（= IMU epoch）递增；
  `sample_sequence` 保持单调不回退。

## 5. `0x88` 姿态语义（重要）

- 参考系：上电时 yaw=0、**重力对齐**的本地系（非 ROS map/odom）；
  四元数为 `R_reference_from_imu`，Hamilton 约定，wire 顺序 `x,y,z,w`（scalar-last）。
- 姿态生成（`imu.c`，100 Hz）：
  1. 加速度计 14 字节 burst 读取，低通（α=0.35）后用**最短弧四元数**把重力方向
     对齐到参考系 +z（`normalize((ay,-ax,0,1+az))`），得到倾角；`|a|` 超出
     0.8–1.2 g 时冻结倾角更新（抑制线加速度污染）。
  2. 陀螺 z（启动时 200 次采样去零偏）在参考系 z 轴的投影积分出 yaw。
  3. 输出四元数 = 航向四元数 ⊗ 倾角四元数。
  - 没有磁力计：**yaw 会缓慢漂移**；倾角存在加速度噪声（低通后的残差）。
  - 协方差 / 时间同步 / `T_base_from_chassis_imu` 均未提供，Host 不得把该姿态
    当作高精度或绝对航向；是否发布 Imu 由 Host 的 `0x88` 门限决定。
- 角速度为 IMU 系原始测量（x/y 未做零偏补偿，z 去零偏），单位 rad/s；
  线加速度为 specific force（m/s²，保留重力，静止 +z 朝上约 +9.81）。
- 0x88 flags = `0x0F`（orientation/gyro/acc/online 均有效）；IMU 读取失败时
  本周期不发状态帧（Host 侧超时后标记 invalid）。
- `0x85` 的 yaw 与控制共用同一条 yaw（度→弧度并归一到 `[-π,π)`）；`0x82` 的
  relative yaw = `wrap(gimbal_yaw − chassis_yaw)`，相对 yaw 速率 = 两 IMU
  z 速率之差；云台 yaw 仍是自身陀螺积分，姿态滤波不作用于云台 IMU。

## 6. 运动授权与仲裁（与 Host 文档 §8 的偏差）

`config.h: V2_ENABLE_MOTION = 1`：**V2 ChassisCommand 默认可以驱动底盘**。
这是本固件的显式取舍（宿主已确认），但与 Host 文档 §8 “Phase 1A 物理禁能 /
无正向 motion owner”的措辞不一致，必须注意：

- 所有权优先级：**NRF 遥控连接 > V2 命令 > 停车**；遥控器连接时 V2 命令只解析
  不执行。
- V2 运动生效条件（全部满足）：IMU 正常、Host heartbeat ≤500 ms、命令 ≤200 ms、
  `enabled=1`。任一条件失效立即硬停车。
- 命令语义：`vx/vy` 为 **base 系**（+x 前、+y 左，m/s），`wz` 绕 +z 右手（rad/s）；
  `vx/vy` 按 `V2_MAX_LINEAR_MPS=2.0` 线性映射到 `MAX_TRANSLATE_DUTY`，`wz` 按
  `ROT_GAIN_DUTY_PER_DPS` 映射；这些系数与电机/减速比相关，**需台架标定**。
- V2 模式下云台保持进入时的目标（无云台命令），PID 继续工作；小陀螺模式只在
  遥控模式生效。
- 台架测试前必须架空车轮或物理禁能电机；Host 联签前不得整车运行。
  彻底禁用 V2 运动：`V2_ENABLE_MOTION=0` 重新编译（命令仍解析、状态仍上报）。

## 7. `boot_id` 与 flash 布局

- **vehicle / vehicle_debug 的 app 区为 0xFE0400–0xFEFDFF**。基址必须是 0xFE0400 而不是
  0xFE0000：STC32G12K128 的 EEPROM 固定从 `FE:0000` 开始、STC-ISP 最小设 1K
  （`FE:0000–FE:03FF`，手册 21.3.1），程序区从 `0xFE0400` 起；代码落在 EEPROM 区
  会导致下载后跑飞/不运行。
- 该布局不再兼容 HID bootloader（0xFE0000–0xFE3DFF）；请用 `tools/flash.py`（ROM ISP）
  或 STC-ISP 烧录。若必须恢复 HID bootloader 流程，需要重新划分 app 区并同步裁剪固件。
- `boot_id` 计数放在 **EEPROM 首扇区 0xFE0000–0xFE01FF**（`V2_BOOT_SECTOR_ADDR`，
  IAP 偏移 0x0000）：上电时 IAP 读取 `'V''B''I''D'` + uint32 counter，counter+1 后
  整扇区擦除并写回，返回非零 `boot_id`。
  * STC-ISP 下载时 “EEPROM 大小” 设为 **1K**（最小即可，正好覆盖 FE:0000–FE:03FF）；
    不必再设 64K/全片。若 IAP 失败仍返回非零 fallback，`boot_persist_fault=1`。
  * 重新烧录会擦除该扇区，计数从 1 重新开始；Host 若保留退役 boot 表，重刷后
    请重启 Host 会话。
- 代码/常量上限 `code_end = 0xFEFDFF`，由 `tools/check_layout.py` 强制；当前
  vehicle 使用约 63.3 KB（从 0xFE0400 起算，余量约 0.7 KB，新增代码需先做尺寸评估）。
- XRAM 约 2.0 KB / 8 KB；EDATA 栈约 89 B（`--stack-auto`），新代码禁止大局部变量。

## 8. 构建、烧录与自检

```bash
python3 tools/build.py --app vehicle            # 生产固件（含 V2 链路）
python3 tools/build.py --app vehicle_debug      # P4.3/P4.4 230400 调试 CSV + golden 自检
python3 tools/build.py --app vehicle_v2test --qemu   # 纯 codec golden 自检（QEMU/CI）
python3 tools/build.py --app vehicle_tune       # 云台调参（UART2 轮询模式）
```

- `vehicle_debug` **不运行 V2 链路**（UART2 保持空闲、由调参/调试工具使用），
  但会链接 codec 并在启动时打印 `V2SELFTEST PASS/FAIL`。
- `vehicle_v2test` 只链接 codec + 自检，在 UART1（P3.0/P3.1，115200）打印
  `PASS` / `FAIL <step>`，用于 QEMU 与 CI 验证 SDCC 大端 ABI 下的 float 序列化。
- 真机 V2 链路观测请用 `tools/v2_probe.py`（见第 9 节）。

## 9. 验证方法

1. **宿主机单测**（无需硬件）：
   ```bash
   cc -std=c11 -O2 -Wall -Wextra -Iapps/vehicle \
      apps/vehicle/v2_codec.c apps/vehicle/v2_selftest.c \
      apps/vehicle/tests/v2_codec_host_test.c -o /tmp/v2_codec_test
   /tmp/v2_codec_test
   ```
   覆盖：§10 golden 向量、0x82/0x85 payload 字节序、NaN/长度/CRC/重同步、
   序号回绕、yaw 的 ±π 边界。
2. **QEMU**：`python3 tools/build.py --app vehicle_v2test --qemu` 串口输出 `PASS`。
3. **上位机探测**（真机）：
   ```bash
   python tools/v2_probe.py --port COM5            # 只监听
   python tools/v2_probe.py --golden                # 本地校验 golden 向量
   python tools/v2_probe.py --port COM5 --chassis 0.1 0 0   # 台架运动（先架空！）
   ```
   期望：heartbeat 10 Hz、状态各 50 Hz、`crc_err=0`、`att_norm_bad=0`、
   `boot` 每次上电递增、`caps=0x125`。
4. **回归**：`vehicle_tune`、`vehicle_tune_hid`、`mpu6050_*`、`bootloader` 等
   app 构建不受影响；`bootloader` 仍占用 0xFE0000 区，不能与 vehicle 同时存在。

## 10. 已知限制 / 未闭环项

- posture（`0x81`）、referee（`0x83`）、operator target（`0x84`）、radar/operator
  target（`0x86/0x87`）未实现；Host 的姿态/publish 门限与 ACQ 链路未在本仓库验证。
- 姿态为“加速度倾角 + 陀螺 yaw”六轴估计：yaw 无磁力计会漂移；倾角对线加速度
  敏感；未提供协方差、延迟补偿、`T_base_from_chassis_imu` 标定。
- 云台/航向状态没有 per-sample boot 绑定（wire 合同限制）；V2 模式与遥控切换
  瞬间存在一次目标冻结/恢复动作。
- V2 运动映射系数（m/s→duty、rad/s→duty）未台架标定。
- ROM ISP 高区（≥0xFF1000）未使用；`vehicle` 不能用 HID bootloader 现场升级。
- 与 Host 的联签、原始抓包、reset/re-zero/延迟旧包测试仍需 Host 侧配合完成。
