#include "GFX.h"
#include <math.h>
#include <stm32f0xx_i2c.h>


/*
 * A simple normalised compass sketch using the HMC5883L magnetometer and a
 * UC1701X LCD, designed for the STM32F0 Discovery board.
 *
 * Author: Harris Shallcross
 * Year: 1/4/2015
 *
 * NOTE: PNumF is a bit dodgy and doesn't print floating point numbers properly
 * YOU HAVE BEEN WARNED!
 *
 *
 *Code and example descriptions can be found on my blog at:
 *www.hsel.co.uk
 *
 *The MIT License (MIT)
 *Copyright (c) 2015 Harris Shallcross
 *
 *Permission is hereby granted, free of charge, to any person obtaining a copy
 *of this software and associated documentation files (the "Software"), to deal
 *in the Software without restriction, including without limitation the rights
 *to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *copies of the Software, and to permit persons to whom the Software is
 *furnished to do so, subject to the following conditions:
 *
 *The above copyright notice and this permission notice shall be included in all
 *copies or substantial portions of the Software.
 *
 *THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *SOFTWARE.
 */

//HMC5883L defines!
#define I_SDA 		GPIO_Pin_11
#define I_SCL 		GPIO_Pin_10

#define I_SDAPS		GPIO_PinSource11
#define I_SCLPS		GPIO_PinSource10

#define I_GPIO		GPIOB
#define I_GPIOAF	GPIO_AF_1

#define I_I2C		I2C2

#define I_HMCADDR	(0x1E<<1)

//Predefined initializer values for minimum and maximum storage
//variables
#define H_MAXSTRT	-32767
#define H_MINSTRT	32767

uint8_t H_ReadReg(uint8_t Reg){
	uint8_t Dat;
	while(I2C_GetFlagStatus(I_I2C, I2C_FLAG_BUSY) == SET);

	I2C_TransferHandling(I_I2C, I_HMCADDR, 1, I2C_SoftEnd_Mode, I2C_Generate_Start_Write);
	while(I2C_GetFlagStatus(I_I2C, I2C_FLAG_TXIS) == RESET);

	I2C_SendData(I_I2C, Reg);
	while(I2C_GetFlagStatus(I_I2C, I2C_FLAG_TC) == RESET);

	I2C_TransferHandling(I_I2C, I_HMCADDR, 1, I2C_AutoEnd_Mode, I2C_Generate_Start_Read);
	while(I2C_GetFlagStatus(I_I2C, I2C_FLAG_RXNE) == RESET);

	Dat = I2C_ReceiveData(I_I2C);

	while(I2C_GetFlagStatus(I_I2C, I2C_FLAG_STOPF) == RESET);
	I2C_ClearFlag(I_I2C, I2C_FLAG_STOPF);

	return Dat;
}

void H_WriteReg(uint8_t Reg, uint8_t Dat){
	while(I2C_GetFlagStatus(I_I2C, I2C_FLAG_BUSY) == SET);

	I2C_TransferHandling(I_I2C, I_HMCADDR, 1, I2C_Reload_Mode, I2C_Generate_Start_Write);
	while(I2C_GetFlagStatus(I_I2C, I2C_FLAG_TXIS) == RESET);

	I2C_SendData(I_I2C, Reg);
	while(I2C_GetFlagStatus(I_I2C, I2C_FLAG_TCR) == RESET);

	I2C_TransferHandling(I_I2C, I_HMCADDR, 1, I2C_AutoEnd_Mode, I2C_No_StartStop);
	while(I2C_GetFlagStatus(I_I2C, I2C_FLAG_TXIS) == RESET);

	I2C_SendData(I_I2C, Dat);
	while(I2C_GetFlagStatus(I_I2C, I2C_FLAG_STOPF) == RESET);
	I2C_ClearFlag(I_I2C, I2C_FLAG_STOPF);
}

volatile uint32_t MSec = 0;

void SysTick_Handler(void){
	MSec++;
}

void Delay(uint32_t MS){
	uint32_t MSS = MSec;
	while((MSec-MSS)<MS);
}

GPIO_InitTypeDef G;
I2C_InitTypeDef I;

int main(void)
{
	SysTick_Config(SystemCoreClock/1000);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);

	G.GPIO_Pin = I_SDA | I_SCL;
	G.GPIO_Mode = GPIO_Mode_AF;
	G.GPIO_OType = GPIO_OType_PP;
	G.GPIO_PuPd = GPIO_PuPd_UP;
	G.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(I_GPIO, &G);

	GPIO_PinAFConfig(I_GPIO, I_SCLPS, I_GPIOAF);
	GPIO_PinAFConfig(I_GPIO, I_SDAPS, I_GPIOAF);

	I.I2C_Ack = I2C_Ack_Enable;
	I.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I.I2C_AnalogFilter = I2C_AnalogFilter_Enable;
	I.I2C_DigitalFilter = 0;
	I.I2C_Mode = I2C_Mode_I2C;
	I.I2C_OwnAddress1 = 0x01;
	I.I2C_Timing = 0x0010020A;
	I2C_Init(I_I2C, &I);
	I2C_Cmd(I_I2C, ENABLE);

	UC1701X_InitSetup();

	int16_t X, Y, Z, Ang;
	float XN = 0.0f, YN = 0.0f, ZN = 0.0f;
	int16_t XMx, XMn;
	int16_t YMx, YMn;
	int16_t ZMx, ZMn;

	XMx = YMx = ZMx = H_MAXSTRT;
	XMn = YMn = ZMn = H_MINSTRT;

	uint8_t Status, ID[3] = {0}, XPos;

	//Configure the module
	H_WriteReg(0, 0x18); //75Hz Data rate with no biasing
	H_WriteReg(1, 0x00); //MAXIMUM gain!
	H_WriteReg(2, 0x00); //Continuous measurement mode

	while(1)
	{

		//Read raw X, Y and Z values
		X = (H_ReadReg(3)<<8)|H_ReadReg(4);
		Y = (H_ReadReg(7)<<8)|H_ReadReg(8);
		Z = (H_ReadReg(5)<<8)|H_ReadReg(6);

		//Peak and trough detection from these values!
		if(X>XMx) XMx = X;
		if(X<XMn) XMn = X;

		if(Y>YMx) YMx = Y;
		if(Y<YMn) YMn = Y;

		if(Z>ZMx) ZMx = Z;
		if(Z<ZMn) ZMn = Z;

		//Print the raw X, Y and Z values with the corresponding
		//minimum and maximum detected values
		ClrBuf();
		PStr("X: ", 0, 0, 0, 0);
		PNum(X, 15, 0, 0, 0, 0);
		PNum(XMx, 40, 0, 0, 0, 0);
		PNum(XMn, 75, 0, 0, 0, 0);

		PStr("Y: ", 0, 8, 0, 0);
		PNum(Y, 15, 8, 0, 0, 0);
		PNum(YMx, 40, 8, 0, 0, 0);
		PNum(YMn, 75, 8, 0, 0, 0);

		PStr("Z: ", 0, 16, 0, 0);
		PNum(Z, 15, 16, 0, 0, 0);
		PNum(ZMx, 40, 16, 0, 0, 0);
		PNum(ZMn, 75, 16, 0, 0, 0);

		//Calculate the normalised X, Y and Z values
		if((XMx-XMn) != 0 && (YMx-YMn) != 0 && (ZMx-ZMn) != 0){
			XN = (float)(X-XMn)/(float)(XMx-XMn)-0.5f;
			YN = (float)(Y-YMn)/(float)(YMx-YMn)-0.5f;
			ZN = (float)(Z-ZMn)/(float)(ZMx-ZMn)-0.5f;
		}

		//Find the bearing
		if(YN>0){
			Ang = 90-atanf(XN/YN)*180.0f/M_PI;
		}
		else if(YN<0){
			Ang = 270-atanf(XN/YN)*180.0f/M_PI;
		}
		else if(YN==0.0f && XN<0.0f){
			Ang = 180;
		}
		else{
			Ang = 0;
		}

		//Print the normalised values
		XPos = PStr("NormX: ", 0, 24, 0, 0);
		PNumF(XN, XPos, 24, 3, 0, 0);

		XPos = PStr("NormY: ", 0, 32, 0, 0);
		PNumF(YN, XPos, 32, 3, 0, 0);

		XPos = PStr("NormZ: ", 0, 40, 0, 0);
		PNumF(ZN, XPos, 40, 3, 0, 0);

		//Print the vector magnitude (normalised)
		XPos = PStr("Mag: ", 0, 48, 0, 0);
		PNum(sqrtf(XN*XN+YN*YN)*1000, XPos, 48, 0, 0, 0);

		//Print the current bearing
		XPos = PStr("Ang: ", 0, 56, 0, 0);
		PNum(Ang, XPos, 56, 0, 0, 0);
		PScrn();

	}
}
