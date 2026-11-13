/*
 * GPIO 最小驱动。
 * 端口号 0-7，引脚号 0-7；模式编码见 STC32Gxx.h 注释。
 */

#ifndef STCNAV_STC32G_GPIO_H
#define STCNAV_STC32G_GPIO_H

typedef enum
{
    STC32G_GPIO_MODE_QUASI = 0,
    STC32G_GPIO_MODE_PUSH_PULL = 1,
    STC32G_GPIO_MODE_HIGH_Z = 2,
    STC32G_GPIO_MODE_OPEN_DRAIN = 3
} stc32g_gpio_mode_t;

void stc32g_gpio_set_mode(unsigned char port, unsigned char pin, stc32g_gpio_mode_t mode);
void stc32g_gpio_write(unsigned char port, unsigned char pin, unsigned char level);
unsigned char stc32g_gpio_read(unsigned char port, unsigned char pin);

#endif
