/*
 * GPIO 最小驱动。
 * 端口号 0-7，引脚号 0-7；模式编码见 STC32Gxx.h 注释。
 */

#ifndef STCNAV_STC32G_GPIO_H
#define STCNAV_STC32G_GPIO_H

typedef enum
{
    GPIO_MODE_QUASI = 0,
    GPIO_MODE_PUSH_PULL = 1,
    GPIO_MODE_HIGH_Z = 2,
    GPIO_MODE_OPEN_DRAIN = 3
} gpio_mode_t;

void gpio_set_mode(unsigned char port, unsigned char pin, gpio_mode_t mode);
void gpio_write(unsigned char port, unsigned char pin, unsigned char level);
unsigned char gpio_read(unsigned char port, unsigned char pin);

#endif
