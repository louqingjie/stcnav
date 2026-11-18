/*
 * PIE-Block 拓展板控制协议。
 *
 * UART1 P3.0/P3.1，230400 8N1。
 * 21 字节帧：
 *   AB BC cmd P60 P62 P64 P66 P74 P75 P76 P77 CD DE
 * 每个 16 位字大端；连续帧间隔 >= 5 ms。
 */

#include "expansion.h"
#include "config.h"
#include "common.h"
#include "stc32g_uart.h"
#include "timebase.h"

#define EXP_HEAD1 0xAB
#define EXP_HEAD2 0xBC
#define EXP_END1 0xCD
#define EXP_END2 0xDE
#define EXP_CMD_INIT 0xAA
#define EXP_CMD_DUTY 0xBB
#define EXP_CMD_DIR 0xDD

#define EXP_FRAME_GAP_MS 5

static unsigned char last_dir[8];
static unsigned char dir_valid;
static unsigned char frame[21];

static void expansion_send_frame(unsigned char command,
                                 unsigned short p60, unsigned short p62,
                                 unsigned short p64, unsigned short p66,
                                 unsigned short p74, unsigned short p75,
                                 unsigned short p76, unsigned short p77)
{
    unsigned char index;

    frame[0] = EXP_HEAD1;
    frame[1] = EXP_HEAD2;
    frame[2] = command;

    /* SDCC C251 对“values[] 数组 + 3+i*2 变址”的写法会取错字节，
     * 必须像 PIE-Block 的 ExpansionBoradControl 一样从形参逐路展开。 */
    frame[3] = (unsigned char)((p60 >> 8) & 0xFF);
    frame[4] = (unsigned char)(p60 & 0xFF);
    frame[5] = (unsigned char)((p62 >> 8) & 0xFF);
    frame[6] = (unsigned char)(p62 & 0xFF);
    frame[7] = (unsigned char)((p64 >> 8) & 0xFF);
    frame[8] = (unsigned char)(p64 & 0xFF);
    frame[9] = (unsigned char)((p66 >> 8) & 0xFF);
    frame[10] = (unsigned char)(p66 & 0xFF);
    frame[11] = (unsigned char)((p74 >> 8) & 0xFF);
    frame[12] = (unsigned char)(p74 & 0xFF);
    frame[13] = (unsigned char)((p75 >> 8) & 0xFF);
    frame[14] = (unsigned char)(p75 & 0xFF);
    frame[15] = (unsigned char)((p76 >> 8) & 0xFF);
    frame[16] = (unsigned char)(p76 & 0xFF);
    frame[17] = (unsigned char)((p77 >> 8) & 0xFF);
    frame[18] = (unsigned char)(p77 & 0xFF);

    frame[19] = EXP_END1;
    frame[20] = EXP_END2;

    for (index = 0; index < 21; index++)
        uart1_write_byte(frame[index]);

    delay_ms(EXP_FRAME_GAP_MS);
}

static void expansion_send_dir(const unsigned char dir[8])
{
    expansion_send_frame(EXP_CMD_DIR,
                         (unsigned short)dir[0], (unsigned short)dir[1],
                         (unsigned short)dir[2], (unsigned short)dir[3],
                         (unsigned short)dir[4], (unsigned short)dir[5],
                         (unsigned short)dir[6], (unsigned short)dir[7]);
}

static void expansion_send_duty(const unsigned short duty[8])
{
    expansion_send_frame(EXP_CMD_DUTY,
                         duty[0], duty[1], duty[2], duty[3],
                         duty[4], duty[5], duty[6], duty[7]);
}

void expansion_init(void)
{
    unsigned char index;

    uart1_init(230400UL, 0);

    expansion_send_frame(EXP_CMD_INIT,
                         10000, 10000, 10000, 10000,
                         10000, 10000, 10000, 10000);

    delay_ms(100);

    expansion_send_frame(EXP_CMD_DUTY, 0, 0, 0, 0, 0, 0, 0, 0);

    for (index = 0; index < 8; index++)
        last_dir[index] = 0xFF;
    dir_valid = 0;
}

void expansion_apply(const unsigned char dir[8], const unsigned short duty[8])
{
    unsigned char index;
    unsigned char changed = 0;

    if (!dir_valid)
        changed = 1;
    else
    {
        for (index = 0; index < 8; index++)
        {
            if (last_dir[index] != dir[index])
            {
                changed = 1;
                break;
            }
        }
    }

    if (changed)
    {
        expansion_send_dir(dir);
        for (index = 0; index < 8; index++)
            last_dir[index] = dir[index];
        dir_valid = 1;
    }

    expansion_send_duty(duty);
}

void expansion_stop(void)
{
    unsigned char dir[8];
    unsigned short duty[8];
    unsigned char index;

    for (index = 0; index < 8; index++)
    {
        dir[index] = 1;
        duty[index] = 0;
    }

    expansion_apply(dir, duty);
}
