/*
 * user.c
 *
 *  Created on: 7 nov. 2017
 *      Author: agarcia
 */

#include "MK22F12810.h"
#include <core_cm4.h>
#include "pin_mux.h"
#include "user.h"
#include "fsl_lptmr.h"
#include "fsl_i2c.h"
#include "fsl_port.h"
#include "fsl_gpio.h"
#include "fsl_ftm.h"
#include "fsl_clock.h"

volatile uint32_t TickCount = 0;

ftm_config_t ftm_PWM_PV;
ftm_config_t ftm_PWM_MOTOR;

ftm_chnl_pwm_signal_param_t ftmParam_PV;
ftm_chnl_pwm_signal_param_t ftmParam_MOTOR[2];


void SysTick_Handler(void)
{
	TickCount++;
}

uint32_t GetTickCount(void)
{
	return TickCount;
}

/*******************************************************************\
    *   unsigned short update_crc_16( unsigned short crc, char c );      *
 *   The function update_crc_16 calculates a  new  CRC-16  value  *
 *   based  on  the  previous value of the CRC and the next byte     *
 *   of the data to be checked.							*
 \*******************************************************************/

uint16_t crc16_update(uint16_t crc, uint8_t a)
{
	crc = crc ^ a;
	for (uint32_t i = 0; i < 8; ++i)
	{
		if (crc & 1)
			crc = (crc >> 1) ^ P_CRC16;
		else
			crc = (crc >> 1);
	}
	return crc;
}

uint16_t crc16(uint8_t *data, uint16_t length)
{
	uint16_t crc = INIT_CRC16;
	for (uint32_t x = 0; x < length; x++)
	{
		crc = crc16_update(crc, *(data + x));
	}
	return crc;
}


uint32_t TimeDiff(uint32_t time_start, uint32_t time_end)
{
	if (time_end >= time_start)
	{
		return time_end - time_start;
	}
	else
	{
		return UINT32_MAX - time_start + time_end + 1;
	}
}

void InitSystick(void)
{
	SysTick_Config(95977);		// Configure for interrupt every 96 * (1 /95977472) = 1 ms
}

void InitTimer_us(void)
{
	 lptmr_config_t lptmr_cfg={
		.timerMode = kLPTMR_TimerModeTimeCounter,
		.prescalerClockSource = kLPTMR_PrescalerClock_0,	// Set MCGIRCLK clock source
		.enableFreeRunning = false,
		.value = kLPTMR_Prescale_Glitch_1,
		.bypassPrescaler = false};
	LPTMR_Init(LPTMR0, &lptmr_cfg);
}

void delay_us(uint16_t time)
{
	LPTMR_SetTimerPeriod(LPTMR0, time);
	LPTMR_StartTimer(LPTMR0);
	while(!(LPTMR0->CSR & LPTMR_CSR_TCF_MASK));
	LPTMR_StopTimer(LPTMR0);
}

void delay_ms(uint32_t time)
{
	uint32_t start_tm = TickCount;
	while(TimeDiff(start_tm, TickCount)< time);
}

void I2C_ClkBurst(void)
{
	uint32_t statusFlags;

	gpio_pin_config_t gpio_config =
	{
		kGPIO_DigitalOutput,
	    1,
	};

	PORT_SetPinMux(I2C1_SCL_PORT, I2C1_SCL_GPIO_PIN, kPORT_MuxAsGpio);

	GPIO_PinInit(I2C1_SCL_GPIO, I2C1_SCL_GPIO_PIN, &gpio_config);

	for(uint32_t x = 0; x < 10; x++)
	{
	    statusFlags = I2C_MasterGetStatusFlags(I2C1);

		/* Return an error if the bus is already in use, but not by us. */
		if ((statusFlags & kI2C_BusBusyFlag) && ((I2C1->C1 & I2C_C1_MST_MASK) == 0))
		{
			GPIO_ClearPinsOutput(I2C1_SCL_GPIO, 1 << I2C1_SCL_GPIO_PIN);
			delay_us(5);
			GPIO_SetPinsOutput(I2C1_SCL_GPIO, 1 << I2C1_SCL_GPIO_PIN);
			delay_us(5);
		}
		else
			break;
	}

	PORT_SetPinMux(I2C1_SCL_PORT, I2C1_SCL_GPIO_PIN, kPORT_MuxAlt2);
}

void I2C_Init(void)
{
	i2c_master_config_t masterConfig;

	/*Default configuration for master. */

	masterConfig.baudRate_Bps = 100000;
	masterConfig.enableMaster = true;
	masterConfig.enableStopHold = false;
	masterConfig.glitchFilterWidth = 0;

	/* Init I2C master.*/
	I2C_MasterInit(I2C1, &masterConfig, CLOCK_GetFreq(kCLOCK_BusClk));
	// I2C1->C2 |= I2C_C2_HDRS_MASK;	// Enable high drive strength
}

int32_t I2C_XferBuffer(uint8_t Device_ID, uint32_t Reg_Addr, uint8_t Addr_nbytes, uint8_t direction, uint8_t *buffer, uint16_t Length)
{
	i2c_master_transfer_t xfer ={
		.flags = kI2C_TransferDefaultFlag,
		.direction = direction & 0x1,	// 0 for write , 1 for read
		.slaveAddress = Device_ID,
		.data = buffer,
		.dataSize = Length,
		.subaddress = Reg_Addr,
		.subaddressSize = Addr_nbytes
	};

	return I2C_MasterTransferBlocking(I2C1, &xfer);
}

/* 8 digits BIN to BCD converter */

uint32_t Dig8_Bin2BCD(uint32_t value)
{
	uint8_t Digit[8];
	uint32_t Shift[8] = {1, 10, 100, 1000, 10000, 100000, 1000000, 10000000};
	uint32_t result = 0;

	for(int32_t y = 7, x = Shift[7]; y >= 0; x = Shift[y], y--)
		for(Digit[y] = 0; value >= x; value -= x, Digit[y]++);

	for(uint32_t x = 0; x < 8; x++)
		result |= (Digit[x] << (4 * x));

    return result;
}

/* 3 digits BIN to BCD converter */

uint32_t fBin2BCD(uint8_t u)
{
	uint8_t c,d;

    for(c = 0; u >= 100; u -= 100, c++);
    for(d = 0; u >= 10; u -= 10, d++);
    return (uint32_t)((c << 8) | (d << 4) | u);
}

/* 3 digits BCD to BIN converter */

uint32_t fBCD2Bin(uint8_t n)
{
    return (uint32_t)((100 * (n & 0xf00) >>8) + (10 * (n & 0xf0) >>4) + (n & 0xf));
}

void PWM_Init(void)
{
	/* Configure ftm params with frequency 24kHZ*/

	ftmParam_PV.chnlNumber = kFTM_Chnl_2;	// FTM0 Channel 2 for PV_PWM
	ftmParam_PV.level = kFTM_LowTrue;
	ftmParam_PV.dutyCyclePercent = 0;
	ftmParam_PV.firstEdgeDelayPercent = 0;
	FTM_GetDefaultConfig(&ftm_PWM_PV);

	/* Initializes the FTM module.*/

	FTM_Init(PWM_PV_PERIPHERAL, &ftm_PWM_PV);
	FTM_SetupPwm(PWM_PV_PERIPHERAL, &ftmParam_PV, 1, kFTM_EdgeAlignedPwm, 100000, CLOCK_GetFreq(kCLOCK_BusClk));
	PWM_PV_PERIPHERAL->CONF |= FTM_CONF_BDMMODE(3);		// FTM operative when Halted on Debug mode

	FTM_StartTimer(PWM_PV_PERIPHERAL, kFTM_SystemClock);

	/* Configure ftm params with frequency 24kHZ*/

	ftmParam_MOTOR[0].chnlNumber = kFTM_Chnl_0;	// FTM0 Channel 0 for MOTOR BRIDGE 1
	ftmParam_MOTOR[0].level = kFTM_HighTrue;
	ftmParam_MOTOR[0].dutyCyclePercent = 10;
	ftmParam_MOTOR[0].firstEdgeDelayPercent = 0;

	ftmParam_MOTOR[1].chnlNumber = kFTM_Chnl_1;	// FTM0 Channel 1 for MOTOR BRIDGE 2
	ftmParam_MOTOR[1].level = kFTM_HighTrue;
	ftmParam_MOTOR[1].dutyCyclePercent = 0;
	ftmParam_MOTOR[1].firstEdgeDelayPercent = 0;

	FTM_GetDefaultConfig(&ftm_PWM_MOTOR);

	/* Initializes the FTM module.*/

	FTM_Init(IN_BRIDGE_1_PERIPHERAL, &ftm_PWM_MOTOR);
	FTM_SetupPwm(IN_BRIDGE_1_PERIPHERAL, ftmParam_MOTOR, 2, kFTM_EdgeAlignedPwm, 5000, CLOCK_GetFreq(kCLOCK_BusClk));

	IN_BRIDGE_1_PERIPHERAL->CONF |= FTM_CONF_BDMMODE(1);		// FTM output POLn when Halted on Debug mode
	FTM_StartTimer(IN_BRIDGE_1_PERIPHERAL, kFTM_SystemClock);
}
