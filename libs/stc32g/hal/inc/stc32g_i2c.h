/*
 * 软件 I2C 主机：SCL=P1.5，SDA=P1.4。
 *
 * 引脚配置为开漏，依赖总线上的上拉电阻（GY-521 板载 4.7k）。
 * i2c_write 返回 0=ACK、1=NACK；
 * 寄存器读写函数返回 0=成功、1=失败。
 */

#ifndef STCNAV_STC32G_I2C_H
#define STCNAV_STC32G_I2C_H

void i2c_init(void);
void i2c_start(void);
void i2c_stop(void);

/* 发送一个字节，返回 0=ACK，1=NACK。 */
unsigned char i2c_write(unsigned char value);

/* 读取一个字节；ack 非 0 时回 ACK，否则回 NACK。 */
unsigned char i2c_read(unsigned char ack);

/* 7 位从机地址（不含读写位）。 */
unsigned char i2c_write_reg(unsigned char address, unsigned char reg, unsigned char value);
unsigned char i2c_read_bytes(unsigned char address, unsigned char reg,
                                    unsigned char *buffer, unsigned char length);

#endif
