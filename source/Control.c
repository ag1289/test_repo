/*
 * Control.c
 *
 *  Created on: 27 nov. 2017
 *      Author: agarcia
 */
#include <fastmath.h>
#include <stddef.h>
#include "MK22F12810.h"
#include "fsl_common.h"
#include "fsl_ftm.h"
#include "fsl_gpio.h"
#include "pin_mux.h"
#include "user.h"
#include "LIS2HH12.h"
#include "Control.h"
#include "Analog.h"
#include "LedFlash.h"

AxesRaw_t accel_data;
volatile float max_Vbat = MAX_VBAT - 2.0, k_pid = 1;

stPIDCtrl stPID_Ctrl[PID_COUNT];
stMotor stDrive = {.status = {.reg = 0x80}, .error = {.reg = 0}, .command = 0, .duty = 0, .limit_max = 90, .limit_min = 0, .nMov = 10};
stPVDrive stPV = {.status = {.reg = 0}, .error = {.reg = 0}, .command = 0, .duty = 0, .T_threshold = 45};

stPIDCtrl *GetMotorPIDCtrl(void)
{
	return (stPIDCtrl *)&stPID_Ctrl;
}

stMotor *GetMotorDrive(void)
{
	return (stMotor *)&stDrive;
}

stPVDrive *GetPVDrive(void)
{
	return (stPVDrive *)&stPV;
}

float PID_Compute(stPIDCtrl *pPID)
{
	float result;

	//	PID calculation

	pPID->error = pPID->target - pPID->value;
	pPID->derivative = pPID->error - pPID->l_error;
	pPID->integral = pPID->integral + pPID->error;
	pPID->l_error = pPID->error;

	// Integral accumulate value limiter

	if(pPID->integral > pPID->integral_max)
		pPID->integral = pPID->integral_max;
	else if (pPID->integral < pPID->integral_min)
		pPID->integral = pPID->integral_min;

	result = fabsf(pPID->Kp *pPID->error + pPID->Kd * pPID->derivative + pPID->Ki * pPID->integral);

	// Output value limiter

	if(result > pPID->value_max)
		result = pPID->value_max;
	else if(result < pPID->value_min)
		result = pPID->value_min;

	return result;
}

void ManControl(stPIDCtrl *pPID)
{
	if(!GPIO_ReadPinInput( KEYB6_GPIO, KEYB6_GPIO_PIN))
	{
		stDrive.status.bits.Auto = !stDrive.status.bits.Auto;
		stDrive.status.bits.KeybCtrl = !stDrive.status.bits.Auto ;
	}
	else if(stDrive.status.bits.KeybCtrl && !GPIO_ReadPinInput(KEYB4_GPIO, KEYB4_GPIO_PIN))
	{
		stDrive.command = stDrive.status.bits.direction = eDRVCMD_REVERSE;
		stDrive.duty = pPID->value_min;
	}
	else if(stDrive.status.bits.KeybCtrl && !GPIO_ReadPinInput( KEYB5_GPIO, KEYB5_GPIO_PIN))
	{
		stDrive.duty = pPID->value_min;
		stDrive.command = stDrive.status.bits.direction = eDRVCMD_FORWARD;
	}
		else if(stDrive.status.bits.KeybCtrl)
		stDrive.command = eDRVCMD_STOP;
}


void ComputePIDMotor(stPIDCtrl *pPID)
{
	uint8_t status;

	if((LIS2HH_GetStatusReg(&status) == kStatus_Success))
	{
		if(status & LIS2HH_STATUS_REG_ZYXDA)
		{
			if(LIS2HH_GetAccAxesRaw(&accel_data) == kStatus_Success)
			{
				float pAlpha = ComputePlaneAngle(&accel_data);

				/* Check if computed angle is valid value */

				if(!isnanf(pAlpha))
					pPID->value = (9 * pPID->value + pAlpha) / 10;

				stDrive.duty = PID_Compute(pPID);

				if(stDrive.status.bits.update)
				{
					if(pPID->error > pPID->error_max)
						stDrive.command = stDrive.status.bits.direction =  eDRVCMD_REVERSE ;
					else if(pPID->error < -pPID->error_max)
						stDrive.command = stDrive.status.bits.direction = eDRVCMD_FORWARD;
				}

				if((stDrive.status.bits.direction == eDRVCMD_REVERSE && pPID->value >= pPID->target) ||
						(stDrive.status.bits.direction == eDRVCMD_FORWARD && pPID->value <= pPID->target))
				{
						//pPID->integral = 0;
						stDrive.command = eDRVCMD_STOP;
				}
			}

			if(pPID->value > stDrive.limit_max && (stDrive.status.bits.direction == eDRVCMD_REVERSE))
			{
				stDrive.error.bits.top_west = 1;
				stDrive.command = eDRVCMD_STOP;
			}
			else if(pPID->value < stDrive.limit_min && (stDrive.status.bits.direction == eDRVCMD_FORWARD))
			{
				stDrive.error.bits.top_east = 1;
				stDrive.command = eDRVCMD_STOP;
			}
			else
				stDrive.error.bits.top_west = stDrive.error.bits.top_east = 0;

			ManControl(pPID);
			MotorControl(&stDrive);
		}
	}
}

float ComputePV_duty(stPIDCtrl *pPID)
{
	volatile float result = 0, error;

	error = (pPID->target - pPID->value) * k_pid;
	if(*stPV.V_batt < max_Vbat - 1)
		result = stPV.duty + error;
	else
		result = stPV.duty - 0.2;

	if(result > pPID->value_max)
		result = pPID->value_max;
	else if(result < pPID->value_min)
		result = 0;

	return result;
}

void ComputePID_PV(stPIDCtrl *pPID)
{
	float max_duty =(1.0 - (((CONV_PERFORMANCE) * (*stPV.V_PV)) / (*stPV.V_batt + DIODE_VOLT_FWD))) * 100.0;
	float vIbatt = ((*stPV.V_PV) * (*stPV.I_PV)) / *stPV.V_batt;

	// pPID->value = (9 * pPID->value + *stPV.I_batt) / 10;

	pPID->value_max = max_duty;
	pPID->value = vIbatt;
	pPID->value = (9 * pPID->value + *stPV.I_batt) / 10;

	stPV.duty = ComputePV_duty(pPID);  // PID_Compute(pPID);

	if(*stPV.T_batt > MAX_CH_TEMP)
		stPV.state = ePVDRV_STOP;

	switch(stPV.state)
	{
		// Check initial conditions to start charging

		case ePVDRV_STANDBY:
			if(*stPV.V_PV > MIN_PV_VOLT && *stPV.V_batt < max_Vbat)
				stPV.state = ePVDRV_START;
			break;

		// Check charging Temperature limits

		case ePVDRV_START:
			if(*stPV.T_batt < (MAX_CH_TEMP - 10.0) && *stPV.T_batt > MIN_CH_TEMP)
			{
				stPV.status.bits.floating = 0;
				stPV.status.bits.battfull = 1;
				stPV.error.bits.batt_Tout = 0;
				pPID->target = I_FLOAT_CH;
				stPV.duty = 0;
				stPV.charging_tm = GetTickCount();
				stPV.command = eDRVCMD_FORWARD;
				stPV.state = ePVDRV_ICONST;
			}
			else
			{
				stPV.error.bits.batt_Tout = 1;
				stPV.state = ePVDRV_STOP;
			}
			break;
		case ePVDRV_ICONST:
			/*if(stPV.status.bits.floating)
			{
				pPID->target = I_FLOAT_CH;
				stPV.state = ePVDRV_VCONST;
			}
			else if(*stPV.V_batt > MAX_VBAT - 0.5)
			{
				pPID->target = I_FLOAT_CH;
				stPV.status.bits.floating = 1;
				stPV.charging_tm = GetTickCount();
				stPV.state = ePVDRV_VCONST;
			}*/

			if(*stPV.T_batt > stPV.T_threshold)
			{
				stPV.duty += stPV.k_temp * (*stPV.T_batt - stPV.T_threshold);
				pPID->target = I_STD_CHBAT;
			}
			else if(*stPV.T_batt < stPV.T_threshold)
			{
					//pPID->target = I_MAX_CHBAT;
					pPID->target = 0.5;
			}
			stPV.command = eDRVCMD_FORWARD;
			break;
		case ePVDRV_VCONST:
			if(TimeDiff(stPV.charging_tm, GetTickCount()) > 28800)
			{
				stPV.status.bits.floating = 0;
				stPV.status.bits.battfull = 1;
				stPV.state = ePVDRV_STOP;
			}
			stPV.command = eDRVCMD_FORWARD;
			break;
		case ePVDRV_STOP:
			stPV.command = eDRVCMD_STOP;
			stPV.state = ePVDRV_STANDBY;
			break;

		default:
			stPV.command = eDRVCMD_STOP;
			stPV.state = ePVDRV_STOP;
			break;
	}

	PV_Control(&stPV);
}

// Set precision duty in float format

void PWM_SetDuty(FTM_Type *base, ftm_chnl_t chnlNumber, float dutyCyclePercent )
{
	uint16_t cnv, mod;

	mod = base->MOD;
	cnv = (uint16_t)(((float)mod * dutyCyclePercent) / 100.0);
	/* For 100% duty cycle */
	if (cnv >= mod)
		cnv = mod + 1;
	base->CONTROLS[chnlNumber].CnV = cnv;
}

void MotorControl(stMotor *pMotor)
{
	switch(pMotor->command)
	{
		case eDRVCMD_NOACTION:
			break;
		case eDRVCMD_FORWARD:
			FTM_UpdatePwmDutycycle(IN_BRIDGE_1_PERIPHERAL, kFTM_Chnl_0 ,kFTM_EdgeAlignedPwm, (uint32_t)pMotor->duty);
			FTM_UpdatePwmDutycycle(IN_BRIDGE_1_PERIPHERAL, kFTM_Chnl_1 ,kFTM_EdgeAlignedPwm, 0);
			FTM_SetSoftwareTrigger(IN_BRIDGE_1_PERIPHERAL, true);
			GPIO_WritePinOutput(INH_BRIDGE1_GPIO, INH_BRIDGE1_GPIO_PIN,1);
			fStartLedFlash(fGetLed(0), 1);
			pMotor->status.bits.direction = 1;
			pMotor->status.bits.run = 1;
			pMotor->command = eDRVCMD_NOACTION;
			break;
		case eDRVCMD_REVERSE:
			FTM_UpdatePwmDutycycle(IN_BRIDGE_1_PERIPHERAL, kFTM_Chnl_0 ,kFTM_EdgeAlignedPwm, 0);
			FTM_UpdatePwmDutycycle(IN_BRIDGE_2_PERIPHERAL, kFTM_Chnl_1 ,kFTM_EdgeAlignedPwm, (uint32_t)pMotor->duty);
			FTM_SetSoftwareTrigger(IN_BRIDGE_2_PERIPHERAL, true);
			GPIO_WritePinOutput(INH_BRIDGE1_GPIO, INH_BRIDGE1_GPIO_PIN,1);
			fStartLedFlash(fGetLed(0), 1);
			pMotor->status.bits.direction = 2;
			pMotor->status.bits.run = 1;
			pMotor->command = eDRVCMD_NOACTION;
			break;
		case eDRVCMD_STOP:
			FTM_UpdatePwmDutycycle(IN_BRIDGE_1_PERIPHERAL, kFTM_Chnl_0 ,kFTM_EdgeAlignedPwm, 0);
			FTM_UpdatePwmDutycycle(IN_BRIDGE_2_PERIPHERAL, kFTM_Chnl_1 ,kFTM_EdgeAlignedPwm, 0);
			GPIO_WritePinOutput(INH_BRIDGE1_GPIO, INH_BRIDGE1_GPIO_PIN,0);
			pMotor->status.bits.run = 0;
			stDrive.status.bits.update = 0;
			pMotor->command = eDRVCMD_NOACTION;
			break;
		default:
			pMotor->command = eDRVCMD_NOACTION;
			break;
	}
}

void PV_Control(stPVDrive *pPV)
{
	switch(pPV->command)
	{
		case eDRVCMD_NOACTION:
			break;
		case eDRVCMD_FORWARD:
			PWM_SetDuty(PWM_PV_PERIPHERAL, kFTM_Chnl_2, pPV->duty);
			FTM_SetSoftwareTrigger(PWM_PV_PERIPHERAL, true);
			pPV->status.bits.run = 1;
			pPV->command = eDRVCMD_NOACTION;
			break;
		case eDRVCMD_STOP:
			FTM_UpdatePwmDutycycle(PWM_PV_PERIPHERAL, kFTM_Chnl_2 ,kFTM_EdgeAlignedPwm, 0);
			FTM_SetSoftwareTrigger(PWM_PV_PERIPHERAL, true);
			pPV->status.bits.run = 0;
			stPV.duty = 0;
			pPV->command = eDRVCMD_NOACTION;
			break;
		default:
			pPV->command = eDRVCMD_NOACTION;
			break;
	}
}

void PID_Init(void)
{
	stPID_Ctrl[0].Kp = 0.5;
	stPID_Ctrl[0].Kd = 0.2;
	stPID_Ctrl[0].Ki = 0.05;
	stPID_Ctrl[0].error_min = 0.1;
	stPID_Ctrl[0].error_max = 1.0;
	stPID_Ctrl[0].integral_max = 200.0;
	stPID_Ctrl[0].integral_min = -200.0;
	stPID_Ctrl[0].value_max = 100.0;
	stPID_Ctrl[0].value_min = 10.0;
	stPID_Ctrl[0].sample_time =	100;
	stPID_Ctrl[0].target = 0;
	stPID_Ctrl[0].caclerr_handle = (void *)&ComputePIDMotor;

	memset((void *)&stPID_Ctrl[0].value, 0, sizeof(stPIDCtrl) - offsetof(stPIDCtrl, value));

	stPID_Ctrl[1].Kp = 25.00;
	stPID_Ctrl[1].Kd = 5.00;
	stPID_Ctrl[1].Ki = 0.25;
	stPID_Ctrl[1].error_min = 0.1;
	stPID_Ctrl[1].error_max = 100;
	stPID_Ctrl[1].integral_max = 100 / (2 * stPID_Ctrl[1].Ki);
	stPID_Ctrl[1].integral_min = -stPID_Ctrl[1].integral_max;
	stPID_Ctrl[1].value_max = 70;
	stPID_Ctrl[1].value_min = 10;
	stPID_Ctrl[1].sample_time =	100;
	stPID_Ctrl[1].target = 0;
	stPID_Ctrl[1].caclerr_handle = (void *)&ComputePID_PV;

	memset((void *)&stPID_Ctrl[1].value, 0, sizeof(stPIDCtrl) - offsetof(stPIDCtrl, value));

	stPV.I_batt = GetAnalogData(eADCCHN_I_BATT);
	stPV.V_batt = GetAnalogData(eADCCHN_V_BATT);
	stPV.T_batt = GetAnalogData(eADCCHN_T_BATT);
	stPV.V_PV = GetAnalogData(eADCCHN_V_PV);
	stPV.I_PV = GetAnalogData(eADCCHN_I_PV);
	stPV.k_temp = -2.0;

	stPID_Ctrl[0].sample_tm =stPID_Ctrl[1].sample_tm = GetTickCount();		// leave 5 seconds to stabilize meassures
}

void PID_RunTime(void)
{
	for(uint32_t x = 0; x < PID_COUNT; x++)
	{
		if(TimeDiff(stPID_Ctrl[x].sample_tm, GetTickCount()) > stPID_Ctrl[x].sample_time)
		{
			if(stPID_Ctrl[x].caclerr_handle != NULL)
				stPID_Ctrl[x].caclerr_handle((void *)&stPID_Ctrl[x]);

			stPID_Ctrl[x].sample_tm = GetTickCount();
		}
	}
}
