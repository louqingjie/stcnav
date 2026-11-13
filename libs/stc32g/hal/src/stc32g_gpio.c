/*
 * GPIO 最小驱动。
 * SFR 不能通过指针间接访问，因此每个端口用 switch 直接寻址。
 */

#include "stc32g_gpio.h"
#include "STC32Gxx.h"

#define SET_PORT_MODE(port)                          \
    do                                               \
    {                                                \
        if (m1)                                      \
            port##M1 |= mask;                        \
        else                                         \
            port##M1 &= (unsigned char)~mask;        \
        if (m0)                                      \
            port##M0 |= mask;                        \
        else                                         \
            port##M0 &= (unsigned char)~mask;        \
    } while (0)

#define SET_PORT_LEVEL(port)                         \
    do                                               \
    {                                                \
        if (level)                                   \
            port |= mask;                            \
        else                                         \
            port &= (unsigned char)~mask;            \
    } while (0)

void stc32g_gpio_set_mode(unsigned char port, unsigned char pin, stc32g_gpio_mode_t mode)
{
    unsigned char mask;
    unsigned char m1;
    unsigned char m0;

    if (port > 7 || pin > 7)
        return;

    mask = (unsigned char)(1u << pin);
    m1 = (unsigned char)(((unsigned char)mode >> 1) & 0x01);
    m0 = (unsigned char)((unsigned char)mode & 0x01);

    switch (port)
    {
    case 0: SET_PORT_MODE(P0); break;
    case 1: SET_PORT_MODE(P1); break;
    case 2: SET_PORT_MODE(P2); break;
    case 3: SET_PORT_MODE(P3); break;
    case 4: SET_PORT_MODE(P4); break;
    case 5: SET_PORT_MODE(P5); break;
    case 6: SET_PORT_MODE(P6); break;
    case 7: SET_PORT_MODE(P7); break;
    default: break;
    }
}

void stc32g_gpio_write(unsigned char port, unsigned char pin, unsigned char level)
{
    unsigned char mask;

    if (port > 7 || pin > 7)
        return;

    mask = (unsigned char)(1u << pin);

    switch (port)
    {
    case 0: SET_PORT_LEVEL(P0); break;
    case 1: SET_PORT_LEVEL(P1); break;
    case 2: SET_PORT_LEVEL(P2); break;
    case 3: SET_PORT_LEVEL(P3); break;
    case 4: SET_PORT_LEVEL(P4); break;
    case 5: SET_PORT_LEVEL(P5); break;
    case 6: SET_PORT_LEVEL(P6); break;
    case 7: SET_PORT_LEVEL(P7); break;
    default: break;
    }
}

unsigned char stc32g_gpio_read(unsigned char port, unsigned char pin)
{
    unsigned char mask;

    if (port > 7 || pin > 7)
        return 0;

    mask = (unsigned char)(1u << pin);

    switch (port)
    {
    case 0: return (P0 & mask) ? 1 : 0;
    case 1: return (P1 & mask) ? 1 : 0;
    case 2: return (P2 & mask) ? 1 : 0;
    case 3: return (P3 & mask) ? 1 : 0;
    case 4: return (P4 & mask) ? 1 : 0;
    case 5: return (P5 & mask) ? 1 : 0;
    case 6: return (P6 & mask) ? 1 : 0;
    case 7: return (P7 & mask) ? 1 : 0;
    default: return 0;
    }
}
