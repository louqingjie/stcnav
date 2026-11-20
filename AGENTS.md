请自动提交Git并推送

## 一、HAL 库编写强制规则（教训：2026-11-19 UART2 排查）

任何 HAL 库文件（`libs/stc32g/hal/**`、`libs/stc32g/include/STC32Gxx.h`、`libs/stc32g/startup/**`）的
编写或修改，必须有下面两份依据，缺一不可：

1. **芯片手册**：`docs/STC32G.pdf`（STC32G 系列技术手册；本机 67 MB，不入库，需要时从 STC 官网下载放到该路径）。
   寄存器地址、位定义、外设模式表、初始化顺序一律以手册为准。
2. **官方库**：`C:\Users\louqi\Desktop\program\STC32G-SOFTWARE-LIB`（STC32G-SOFTWARE-LIB，只读参考，不修改）。
   同名外设的初始化/收发实现和宏定义（如 `UART2_SW`、`UART_8bit_BRTx`）以它为准；
   与手册冲突时先查手册，再对照官方库例程。

具体要求：

- 每个寄存器写入/位操作都要能指到手册的寄存器表或章节；初始化顺序、模式位取值要与官方库一致。
- **禁止**凭 STC8/STC15 或其它 8051 的经验类推 STC32G 的寄存器（UART2 模式位就是反例）。
- 提交信息或代码注释中写明依据：手册章节/寄存器名 + 官方库对应文件/函数。
- 行为不明的外设，必须用 Keil C251 编译官方例程并对比汇编/寄存器序列
  （示例：`libs/stc32g/hal/docs/uart2_assembly_compare.md`，
  对照工程 `C:\Users\louqi\Desktop\program\keil_uart2_p10p11`，由官方 06 号 UART2 例程改 P1.0/P1.1 得到）。
- HAL 修改后重新构建受影响的 app（至少 `hello_uart` 和目标 app），确认编译与布局校验通过。

## 二、已经踩过的坑（禁止重演）

- **`S2CON(0x9A)` 不可位寻址**：地址不是 8 的整数倍就不能位寻址。SDCC 的
  `__sbit __at (0x9A/0x9B)` 实际会别名到 `SCON(0x98).2/.3`（UART1 的 RB8/TB8），
  `S2RI/S2TI` 必须用 `S2CON & 0x01`、`S2CON & 0x02` 掩码访问。
- **STC32G 的 UART2 是 4 模式结构**：`S2SM0=S2CON^7`、`S2SM1=S2CON^6`（官方 `STC32G.H`），
  官方库 `UART_8bit_BRTx=(1<<6)`，初始化后 `S2CON=0x50`。
  写成 `S2CON=0x10`（S2SM1=0）会落到模式 0 同步移位串口，P1.1 输出的是移位时钟而不是 UART 帧。
- **UART2 中断使能是 `ES2=IE2.0`**；`IE2.1` 是 ESPI（SPI），写错后 UART2 中断永远不会进。
