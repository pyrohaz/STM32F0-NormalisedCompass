#ifndef UC1701X_H
#define UC1701X_H

#include <stm32f0xx_gpio.h>
#include <stm32f0xx_rcc.h>
#include <stm32f0xx_spi.h>

typedef enum WModesS{
	Dat,
	Reg,
} WModes;

//Main functions
void UC1701X_InitSetup();
void SB(uint8_t, uint8_t, uint8_t);
void PScrn(void);
void ClrBuf(void);
uint8_t BacklightIO(uint8_t);

void Delay(uint32_t);

#define Clk GPIO_Pin_5
#define DIn GPIO_Pin_7
#define DC GPIO_Pin_0
#define CE GPIO_Pin_1

#define ClkPS GPIO_PinSource5
#define DInPS GPIO_PinSource7

#define IOGPIO GPIOA

#define XPix 132
#define YPix 64
#define GBufS (XPix*YPix)/8

extern uint8_t Mode;
extern uint8_t GBuf[GBufS];

#endif
