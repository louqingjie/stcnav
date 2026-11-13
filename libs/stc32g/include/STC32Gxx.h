/*
 * STC32G12K128 SFR definitions.
 *
 * 依据：STC32G 系列单片机技术参考手册（2024-07 版）第 11.3 节
 *       “特殊功能寄存器列表（SFR：0x80-0xFF）”。
 * 仅收录本项目当前使用的寄存器；新增寄存器时在此补充并注明手册页码。
 *
 * 注意：MCS-251 为大端 ABI，多字节寄存器一律按字节访问。
 */

#ifndef STCNAV_STC32GXX_H
#define STCNAV_STC32GXX_H

/* ---------- 端口 ---------- */
__sfr __at (0x80) P0;
__sfr __at (0x90) P1;
__sfr __at (0xA0) P2;
__sfr __at (0xB0) P3;
__sfr __at (0xC0) P4;
__sfr __at (0xC8) P5;
__sfr __at (0xE8) P6;
__sfr __at (0xF8) P7;

/* 端口模式：M1:M0 = 00 准双向, 01 推挽, 10 高阻输入, 11 开漏 */
__sfr __at (0x93) P0M1;
__sfr __at (0x94) P0M0;
__sfr __at (0x91) P1M1;
__sfr __at (0x92) P1M0;
__sfr __at (0x95) P2M1;
__sfr __at (0x96) P2M0;
__sfr __at (0xB1) P3M1;
__sfr __at (0xB2) P3M0;
__sfr __at (0xB3) P4M1;
__sfr __at (0xB4) P4M0;
__sfr __at (0xC9) P5M1;
__sfr __at (0xCA) P5M0;
__sfr __at (0xCB) P6M1;
__sfr __at (0xCC) P6M0;
__sfr __at (0xE1) P7M1;
__sfr __at (0xE2) P7M0;

/* ---------- 系统 ---------- */
__sfr __at (0x81) SP;
__sfr __at (0x9D) IRCBAND;
__sfr __at (0xC2) IAP_DATA;
__sfr __at (0xC3) IAP_ADDRH;
__sfr __at (0xC4) IAP_ADDRL;
__sfr __at (0xC5) IAP_CMD;
__sfr __at (0xC6) IAP_TRIG;
__sfr __at (0xC7) IAP_CONTR;
__sfr __at (0xF5) IAP_TPS;
__sfr __at (0xF6) IAP_ADDRE;
__sfr __at (0xDC) USBCLK;
__sfr __at (0xEC) USBDAT;
__sfr __at (0xF4) USBCON;
__sfr __at (0xFC) USBADR;
__sfr __at (0x82) DPL;
__sfr __at (0x83) DPH;
__sfr __at (0x84) DPXL;
__sfr __at (0x85) SPH;
__sfr __at (0x87) PCON;
__sfr __at (0x8E) AUXR;
__sfr __at (0x8F) INTCLKO;
__sfr __at (0xE9) WTST;
__sfr __at (0xEA) CKCON;
__sfr __at (0xEF) AUXINTIF;

/* ---------- 定时器 0/1 ---------- */
__sfr __at (0x88) TCON;
__sfr __at (0x89) TMOD;
__sfr __at (0x8A) TL0;
__sfr __at (0x8B) TL1;
__sfr __at (0x8C) TH0;
__sfr __at (0x8D) TH1;
__sfr __at (0xD6) T2H;
__sfr __at (0xD7) T2L;

/* ---------- 串口 1 ---------- */
__sfr __at (0x98) SCON;
__sfr __at (0x99) SBUF;
__sfr __at (0xA2) P_SW1;
__sfr __at (0xBA) P_SW2;

/* ---------- 中断 ---------- */
__sfr __at (0xA8) IE;
__sfr __at (0xAF) IE2;
__sfr __at (0xB8) IP;
__sfr __at (0xB5) IP2;

/* ---------- 其它 ---------- */
__sfr __at (0xD0) PSW;
__sfr __at (0xD1) PSW1;
__sfr __at (0xE0) ACC;
__sfr __at (0xF0) B;

/* ---------- 位定义（仅位寻址区） ---------- */
/* TCON 0x88 */
__sbit __at (0x8F) TF1;
__sbit __at (0x8E) TR1;
__sbit __at (0x8D) TF0;
__sbit __at (0x8C) TR0;
__sbit __at (0x8B) IE1;
__sbit __at (0x8A) IT1;
__sbit __at (0x89) IE0;
__sbit __at (0x88) IT0;

/* SCON 0x98 */
__sbit __at (0x99) TI;
__sbit __at (0x98) RI;

/* IE 0xA8 */
__sbit __at (0xAF) EA;
__sbit __at (0xAE) ELVD;
__sbit __at (0xAD) EADC;
__sbit __at (0xAC) ES;
__sbit __at (0xAB) ET1;
__sbit __at (0xAA) EX1;
__sbit __at (0xA9) ET0;
__sbit __at (0xA8) EX0;

/* IP 0xB8 */
__sbit __at (0xB9) PT0;
__sbit __at (0xBC) PS;

#endif
