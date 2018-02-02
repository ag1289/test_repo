/*
 * Copyright (c) 2016, NXP Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
 
/**
 * @file    20170518.001_A_1 axis tracker controller.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include <fastmath.h>
#include <time.h>
#include "board.h"
#include "user.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MK22F12810.h"
#include "fsl_gpio.h"
#include "fsl_ftm.h"
#include "fsl_i2c.h"
#include "fsl_wdog.h"
#include "LoRa.h"
#include "LIS2HH12.h"
#include "MCP794X_Driver.h"
#include "SunPos.h"
#include "ModbusRTU.h"
#include "Analog.h"
#include "Control.h"
#include "LedFlash.h"


uint8_t Lora_TX[]="HopeRF RFM COBRFM95-S";
uint8_t Lora_RX[128];
wdog_config_t WatchDogCfg;

volatile struct tm system_DateTime;

int main(void)
{
	uint32_t timeUpdate_tm = 0;
	volatile bool set_datetime = false;


/*
	ftm_config_t ftm_PWM_PV;
	ftm_config_t ftm_PWM_MOTOR;

	// uint8_t PV_DutyCicle = 0, Motor_DutyCile = 0;
	ftm_chnl_pwm_signal_param_t ftmParam_PV;
	ftm_chnl_pwm_signal_param_t ftmParam_MOTOR[2];*/

	// Init board hardware.

    BOARD_InitBootPins();
    BOARD_InitBootClocks();
	BOARD_Init();
	InitSystick();
	InitTimer_us();

	vInitialize();	// RFM95 initialization
	vGoRx();

	I2C_Init();
	LIS2HH_Init();

	if(Init_RTC() == kStatus_Success)
	{
		if(set_datetime)
		{
			system_DateTime.tm_hour = 8;
			system_DateTime.tm_min = 18;
			system_DateTime.tm_sec = 0;

			system_DateTime.tm_wday = 1;
			system_DateTime.tm_mday = 21;
			system_DateTime.tm_mon = 10;	// 0: January, 11: December
			system_DateTime.tm_year = 118;	// Year since 1900
			fSetDate((struct tm *)&system_DateTime);
			fSetTime((struct tm *)&system_DateTime);
		}
		InitSunPos((struct tm *)&system_DateTime);
	}

	USART_Init();
	Analog_Init();
	PWM_Init();

	// testBCD = Dig8_Bin2BCD(87654321);
	// testBCD = fBCD2Bin(fBin2BCD(175));

	PID_Init();
	timeUpdate_tm = GetTickCount();

	fGetLed(0)->LedGPIO = (uint32_t)STATUSLED_GPIO;
	fGetLed(0)->LedPin = STATUSLED_GPIO_PIN;
	fGetLed(0)->InvertOut = 1;
	fGetLed(0)->Enable = 1;
	fSetLedPeriod(fGetLed(0), 80, 20);
	fStartLedFlash(fGetLed(0), 10);

	WDOG_GetDefaultConfig(&WatchDogCfg);
	WDOG_Init(WDOG, &WatchDogCfg);

	while(1)
    {
		if(TimeDiff(timeUpdate_tm, GetTickCount()) > 1000)	// Updated every second
		{
			if(fGetTime((struct tm *)&system_DateTime) == RTC_OK);
				if(fGetDate((struct tm *)&system_DateTime) == RTC_OK)
				{
					computeSunPos(1);
					// bSendMessage(Lora_TX, sizeof(Lora_TX) -1);
					timeUpdate_tm = GetTickCount();
				}
		}

		if(!computeSunPos(0))
		{
			float sun_angle = GetSunElevation();

			if(!isnan(sun_angle) && GetMotorDrive()->status.bits.Auto)
			{
				float delta = fabsf(sun_angle - GetMotorPIDCtrl()[0].value);
				if( delta > 1.0)
				{
					GetMotorPIDCtrl()[0].target = sun_angle;
					GetMotorDrive()->status.bits.update = 1;
				}
			}
		}

		AnalogRunTime();
		PID_RunTime();

		if(!LoRaCheckTXstatus() && bGetMessage(Lora_RX) > 7)
			Compute_MBUSRequest((uint8_t *)&Lora_RX, LORA_PORT, fGet_COM_Param()->Mbus_sID);

		fMbus_Engine();
		LedRunTime();
		WDOG_Refresh(WDOG);
    }
    return 0 ;
}
