/*
 * IAP（EEPROM/代码 Flash 自编程）与软复位。
 *
 * STC32G12K128 的 IAP 地址从 0xFE0000 起算：
 *   物理 0xFE0000 -> IAP 0x000000
 *   物理 0xFF0000 -> IAP 0x010000
 * 前提是 ISP 选项里的 EEPROM 大小覆盖目标区域（整片 128K 时设为 128K）。
 *
 * 所有函数都使用 24 位物理地址，内部减去 0xFE0000。
 */

#ifndef STCNAV_STC32G_IAP_H
#define STCNAV_STC32G_IAP_H

#define IAP_BASE 0xFE0000UL
#define IAP_SECTOR_SIZE 512UL

/* 打开 IAP，设置 IAP_TPS；一批操作前后分别调用 begin/end。 */
void iap_begin(void);
void iap_end(void);

/* 单字节读/写；返回 0 成功，1 失败。擦除粒度为 512 字节。 */
unsigned char iap_read_byte(unsigned long address);
unsigned char iap_write_byte(unsigned long address, unsigned char value);
unsigned char iap_erase_sector(unsigned long address);

/* 自上次 begin 以来是否发生过 CMD_FAIL。 */
unsigned char iap_failed(void);

/* 写 IAP_CONTR=0x60，复位后进入系统区等待 USB ISP 下载。 */
void iap_reset_to_isp(void);

/*
 * 应用 -> 自研 Bootloader 的软复位握手。
 * 魔数放在 XRAM 末尾，软复位不会清掉；Bootloader 读取后清零。
 */
void iap_request_bootloader(void);
unsigned char iap_take_boot_request(void);

#endif
