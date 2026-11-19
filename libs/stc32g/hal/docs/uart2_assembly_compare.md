# UART2（P1.0/P1.1）汇编对比：本仓库 SDCC 固件 vs Keil + STC32G-SOFTWARE-LIB

日期：2026-11-19
目的：用 STC 官方库 + Keil C251 实现 P1.0(RxD2)/P1.1(TxD2) 串口实验，并与本仓库
SDCC 构建的 UART2 驱动做汇编级对比，定位"P1.1 没有 UART 波形"的根因。

---

## 1. 实验工程

| 项目 | 值 |
| --- | --- |
| 官方例程 | `STC32G-SOFTWARE-LIB/独立程序/06-串口2中断模式与电脑收发测试` |
| 工作副本 | `C:\Users\louqi\Desktop\program\keil_uart2_p10p11` |
| 相对官方例程的改动 | 1) `UART2_SW(UART2_SW_P10_P11)`（默认是 P4.6/P4.7）<br>2) P1.0 高阻输入 / P1.1 推挽输出<br>3) `config.h` 的 `MAIN_Fosc` 改为 35000000L<br>4) 每 500 ms 追加一行 `U2 HB KEIL` 便于观察 |
| 工具链 | Keil µVision V5.25.3，PK251 5.60（C251/A251/L251） |
| 产物 | `list/UART2.hex`（地址范围 `0xFF0000-0xFF0CD6`） |
| 烧录 | STC-ISP，芯片 STC32G12K128，**IRC 频率 35 MHz**，EEPROM 1K |
| 观测 | 115200 8N1：上电打印 `U2 READY KEIL P1.0/P1.1 baud=115200 fosc=35000000`，每 500 ms 一行 `U2 HB KEIL`，收到的字节原样回显 |

实验工程编译命令：

```bat
UV4.exe -b UART2.uvproj -j0 -o build.log
:: 生成汇编：SRC(...) 输出 .SRC，A251 的 PRINT(...) 输出带机器码的 .lst
C251.EXE STC32G_UART.c  XSMALL INTR2 SRC(.\list\STC32G_UART.src)
A251.EXE .\list\STC32G_UART.src PRINT(.\list\STC32G_UART_asm.lst)
```

本仓库 SDCC 侧对照物：`tools/build.py --app uart2_echo` 生成的
`build/uart2_echo/libs_stc32g_hal_src_stc32g_uart2_c.lst`（SDCC 自带机器码）。

---

## 2. 结论（先说结果）

| # | 差异 | 本仓库 SDCC（修复前） | Keil + STC 官方库 | 影响 |
| --- | --- | --- | --- | --- |
| 1 | **S2CON 模式位** | `75 9A 10` → `S2CON=0x10`（S2SM0=0,S2SM1=0） | `S2CON=(S2CON&0x3F)|0x40`，再 `SETB S2REN` → **0x50** | **根因**：0x10 是模式0"同步移位串口"，P1.1 输出的是移位时钟而不是 UART 帧 |
| 2 | S2TI/S2RI 访问 | 旧代码 `__sbit __at(0x9A/0x9B)` → `clr 0x9B` 等绝对位地址，别名到 `SCON(0x98).2/.3`（UART1 RB8/TB8） | `A9 D1 9A`(SETB S2CON.1) / `A9 31 9A`(JNB S2CON.1) 等 MCS251 位操作指令 | 轮询发送死等、ISR 清不掉 S2 标志（已在上一提交修复为字节掩码） |
| 3 | UART2 中断使能 | 旧代码 `IE2 |= 0x02`（= ESPI/SPI） | STC 头文件 `ES2 = IE2^0` | UART2 中断从未打开（已修复为 `IE2 |= 0x01`） |
| 4 | 波特率重装值 | `65536-(FOSC/baud+2)/4` @35MHz/115200 → 76 → 115132 baud（-0.06%） | `65536-(FOSC/4)/baud` @35MHz/115200 → 75 → 116667 baud（+1.27%） | 都能被 CH340 正确解码；官方例程默认 22.1184 MHz 时是整除（48→115200 精确） |
| 5 | 代码布局 | app `0xFE4000` + 向量区 `0xFF0000`（叹跳 EJMP） | 全部在 `0xFF0000-0xFFFFFF`（近调用） | 都能运行（本仓库 SDCC 复位向量 `02 00 0F` → `0xFF000F` → `ejmp 0xFE4000`） |
| 6 | 引脚切换 / 引脚模式 | `53 BA FE`（`P_SW2 &= ~0x01`）、P1M1/P1M0 写 P1.0 高阻 + P1.1 推挽 | `53 BA FE`（`P_SW2 &= ~0x01`）、`GPIO_Inilize` 同样效果 | **完全一致** |

**根因是第 1 条**：STC32G 的 UART2 是 4 模式结构（官方 `STC32G.H` 里
`S2SM0=S2CON^7`、`S2SM1=S2CON^6`，库函数 `UART_8bit_BRTx=(1<<6)`）：

```
S2SM0 S2SM1
  0     0    模式0  同步移位串行方式      <-- 本仓库原代码 0x10 落在这里
  0     1    模式1  8位UART，可变波特率   <-- 官方库 0x50 / S2CON=0x50
  1     0    模式2  9位UART，固定波特率
  1     1    模式3  9位UART，可变波特率
```

模式0 下 P1.1(TxD2) 输出的是同步移位时钟，P1.0(RxD2) 是数据，CH340/上位机只能看到
高频噪声，因此"板子有电、程序在跑、但 P1.1 收不到 UART 数据"。

---

## 3. 逐条汇编对照

### 3.1 UART2 初始化 —— Keil（官方库，`?PR?UART_CONFIGURATION`）

`main.src` 里传入 `UART_8bit_BRTx`：

```
000000 7440            MOV      A,#040H              ; UART_Mode = 0x40 (S2SM1=1)
```

`STC32G_UART_asm.lst`（`UART_Configuration` 的 UART2 分支，函数位于 `0xFF03E9`）：

```
000134 7E7BA0          MOV      R10,@DR28            ; R10 = UART_Mode = 0x40
000137 E59A            MOV      A,S2CON
000139 543F            ANL      A,#03FH
00013B 4CBA            ORL      R11,R10
00013D F59A            MOV      S2CON,A              ; S2CON = 0x40  模式1 8位UART
...
00016F A9C48E          CLR      T2R                  ; AUXR.4 = 0
000172 A9C38E          CLR      T2_CT                ; AUXR.3 = 0
000175 A9D28E          SETB     T2x12                ; AUXR.2 = 1  Timer2 1T
000178 7CB6            MOV      R11,R6
00017A F5D6            MOV      T2H,A                ; 重装值高字节
00017C 7CB7            MOV      R11,R7
00017E F5D7            MOV      T2L,A                ; 重装值低字节
000180 A9D48E          SETB     T2R                  ; AUXR.4 = 1  启动 Timer2
000195 A9D49A          SETB     S2REN                ; S2CON |= 0x10 -> 0x50
```

### 3.2 UART2 初始化 —— 本仓库 SDCC

修复前（`build/.../stc32g_uart2_c.lst`）：

```
00009C 75 9A 10        mov      _S2CON,#0x10         ; S2SM0=0,S2SM1=0 -> 模式0 同步移位（错误）
```

修复后：

```
...  mov  T2L/T2H、AUXR |= 0x14
00009C 75 9A 50        mov      _S2CON,#0x50         ; 模式1 8位UART + S2REN
```

### 3.3 S2TI / S2RI 访问

Keil/A251 对"不可位寻址 SFR 的位"使用 MCS251 的 `A9` 扩展位操作指令，操作数
明确是 `0x9A`（S2CON），不会落到 8051 的绝对位地址：

```
; UART2_ISR_Handler (0xFF097F)
000006 A9309A00 F      JNB      S2RI,?C0009          ; JNB  S2CON.0
00000A A9C09A          CLR      S2RI                 ; CLR  S2CON.0
00003C 7EA19B          MOV      R10,S2BUF
000056 A9319A00 F      JNB      S2TI,?C0013          ; JNB  S2CON.1
00005A A9C19A          CLR      S2TI                 ; CLR  S2CON.1
000071 F59B            MOV      S2BUF,A

; TX2_write2buff 队列模式触发发送中断
000027 A9D19A          SETB     S2TI                 ; SETB S2CON.1
```

本仓库 SDCC 旧写法（`__sbit __at (0x9A) S2RI; __sbit __at (0x9B) S2TI;`）：

```
C2 9B                  clr      0x9B                 ; 位地址 0x9B = SCON(0x98).3 = UART1 TB8 !!
20 9B                  jb       0x9B, ...            ; 同上
C2 9A                  clr      0x9A                 ; 位地址 0x9A = SCON(0x98).2 = UART1 RB8
```

修复后（字节掩码，语义等价于官方库）：

```
E5 9A                  mov      a,_S2CON             ; 取 S2CON
53 9A FD               anl      _S2CON,#0xFD         ; 清 S2TI
53 9A FE               anl      _S2CON,#0xFE         ; 清 S2RI
53 9A FC               anl      _S2CON,#0xFC         ; 清 S2RI+S2TI
43 AF 01               orl      _IE2,#0x01           ; ES2 = IE2.0（修复前是 0x02=ESPI）
```

### 3.4 引脚切换与引脚模式（两边一致）

```
; Keil main.src
000029 53BAFE          ANL      P_SW2,#0FEH          ; S2_S=0 -> UART2 在 P1.0/P1.1

; SDCC stc32g_uart2.c
53 BA FE               anl      _P_SW2,#0xFE
```

引脚模式两边都是：P1.0 高阻输入（RxD2），P1.1 推挽输出（TxD2）。

### 3.5 波特率公式差异（非根因，但值得记录）

| 实现 | 公式（1T） | 35 MHz / 115200 | 实际波特率 | 误差 |
| --- | --- | --- | --- | --- |
| 本仓库 SDCC | `reload = 65536 - (FOSC/baud + 2)/4` | 76 | 115132 | -0.06% |
| STC 官方库 | `reload = 65536 - (FOSC/4)/baud` | 75 | 116667 | +1.27% |

官方例程默认 `MAIN_Fosc=22118400`（`(22118400/4)/115200 = 48` 整除），本实验按
35 MHz 编译，误差 +1.27%，仍在 CH340/上位机容差内；若要严格 115200，可把
`config.h` 改回 `22118400L` 并在 STC-ISP 选 22.1184 MHz。

---

## 4. 修复

`libs/stc32g/hal/src/stc32g_uart2.c`：

```c
/* 串口2 模式 1：8 位数据、可变波特率，并允许接收（S2REN=1）。 */
S2CON = 0x50;
```

加上此前两个修复（同一驱动）：

1. `S2CON(0x9A)` 不可位寻址，`S2RI/S2TI` 改用 `S2CON & 0x01 / 0x02` 掩码；
2. UART2 中断使能 `IE2.0`（ES2），不是 `IE2.1`（ESPI）。

## 5. 硬件验证方法

1. STC-ISP 烧 `C:\Users\louqi\Desktop\program\keil_uart2_p10p11\list\UART2.hex`，
   **IRC=35 MHz**、EEPROM=1K；
2. USB-TTL 交叉接 P1.1（TXD）→ CH340 RXD、CH340 TXD → P1.0、GND 共地，115200 8N1；
3. 串口助手应看到：
   ```
   U2 READY KEIL P1.0/P1.1 baud=115200 fosc=35000000
   U2 HB KEIL
   ```
   向 MCU 发送任意字节应被原样回显。
