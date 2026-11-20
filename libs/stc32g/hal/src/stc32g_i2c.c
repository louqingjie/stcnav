/*
 * 软件 I2C 主机。
 *
 * 使用开漏输出：写 1 释放总线，由上拉电阻拉高；写 0 拉低。
 * 总线速率由 i2c_delay() 控制，当前约 100 kHz 量级，MPU6050 完全兼容。
 */

#include "stc32g_i2c.h"
#include "common.h"

#define I2C_SCL 0x20
#define I2C_SDA 0x10

#define SCL_HIGH() (P1 |= I2C_SCL)
#define SCL_LOW() (P1 &= (unsigned char)~I2C_SCL)
#define SDA_HIGH() (P1 |= I2C_SDA)
#define SDA_LOW() (P1 &= (unsigned char)~I2C_SDA)
#define SDA_READ() ((P1 & I2C_SDA) ? 1 : 0)

static void i2c_delay(void)
{
    us_delay(3);
}

void i2c_init(void)
{
    /* P1.4/P1.5 开漏，先把锁存器置 1 释放总线。 */
    P1M1 |= (unsigned char)(I2C_SCL | I2C_SDA);
    P1M0 |= (unsigned char)(I2C_SCL | I2C_SDA);
    P1 |= (unsigned char)(I2C_SCL | I2C_SDA);
}

void i2c_start(void)
{
    SDA_HIGH();
    SCL_HIGH();
    i2c_delay();
    SDA_LOW();
    i2c_delay();
    SCL_LOW();
    i2c_delay();
}

void i2c_stop(void)
{
    SDA_LOW();
    i2c_delay();
    SCL_HIGH();
    i2c_delay();
    SDA_HIGH();
    i2c_delay();
}

static void i2c_write_bit(unsigned char value)
{
    if (value)
        SDA_HIGH();
    else
        SDA_LOW();
    i2c_delay();
    SCL_HIGH();
    i2c_delay();
    SCL_LOW();
    i2c_delay();
}

static unsigned char i2c_read_bit(void)
{
    static unsigned char value; /* EDATA 栈紧张：热路径局部变量一律 static */

    SDA_HIGH();
    i2c_delay();
    SCL_HIGH();
    i2c_delay();
    value = SDA_READ();
    SCL_LOW();
    i2c_delay();

    return value;
}

unsigned char i2c_write(unsigned char value)
{
    static unsigned char index;
    static unsigned char ack;

    for (index = 0; index < 8; index++)
    {
        i2c_write_bit((unsigned char)(value & 0x80));
        value <<= 1;
    }

    SDA_HIGH();
    i2c_delay();
    SCL_HIGH();
    i2c_delay();
    ack = SDA_READ();
    SCL_LOW();
    i2c_delay();

    return ack;
}

unsigned char i2c_read(unsigned char ack)
{
    static unsigned char index;
    static unsigned char value;
    value = 0;

    SDA_HIGH();
    for (index = 0; index < 8; index++)
    {
        value <<= 1;
        value |= i2c_read_bit();
    }

    if (ack)
        SDA_LOW();
    else
        SDA_HIGH();
    i2c_delay();
    SCL_HIGH();
    i2c_delay();
    SCL_LOW();
    i2c_delay();
    SDA_HIGH();

    return value;
}

unsigned char i2c_write_reg(unsigned char address, unsigned char reg, unsigned char value)
{
    unsigned char status = 0;

    i2c_start();
    if (i2c_write((unsigned char)(address << 1)) != 0)
        status = 1;
    else if (i2c_write(reg) != 0)
        status = 1;
    else if (i2c_write(value) != 0)
        status = 1;
    i2c_stop();

    return status;
}

unsigned char i2c_read_bytes(unsigned char address, unsigned char reg,
                                    unsigned char *buffer, unsigned char length)
{
    static unsigned char index;
    static unsigned char status;
    status = 0;

    if (length == 0)
        return 1;

    i2c_start();
    if (i2c_write((unsigned char)(address << 1)) != 0)
        status = 1;
    else if (i2c_write(reg) != 0)
        status = 1;
    else
    {
        i2c_start();
        if (i2c_write((unsigned char)((address << 1) | 0x01)) != 0)
            status = 1;
        else
        {
            for (index = 0; index < length; index++)
                buffer[index] = i2c_read((unsigned char)(index + 1 < length));
        }
    }
    i2c_stop();

    return status;
}
