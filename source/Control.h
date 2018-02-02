/*
 * Control.h
 *
 *  Created on: 27 nov. 2017
 *      Author: agarcia
 */

#ifndef CONTROL_H_
#define CONTROL_H_

#include "fsl_ftm.h"

#define PID_COUNT 	2

#define DIODE_VOLT_FWD		0.35
#define CONV_PERFORMANCE	0.90
typedef enum tag_eDRVCMD
{
	eDRVCMD_NOACTION = 0,
	eDRVCMD_FORWARD,
	eDRVCMD_REVERSE,
	eDRVCMD_STOP = 0x04,
	eDRVCMD_MAX = 0xffff
}eDRVCMD;

typedef enum tag_ePVDRV
{
	ePVDRV_STANDBY = 0,
	ePVDRV_START,
	ePVDRV_ICONST,
	ePVDRV_VCONST,
	ePVDRV_STOP,
	ePVDRV_MAX = 0xffffffff
}ePVDRV;

typedef struct tag_stPIDCtrl
{
	float Kp;				// Proportional constant
	float Kd;				// Derivative constant
	float Ki;				// Integral constant
	float error_min;		// minimum error value to stop movement
	float error_max;		// maximum error value to start movement
	float integral_max;		// maximum accumulate integral value
	float integral_min;		// minimum accumulate integral value
	float value_max;		// maximum allowed output value
	float value_min;       	// minimum allowed output value
	uint32_t sample_time;	// Sample time interval
	float target;			// target value
	void (*caclerr_handle)(void *);	// function to calc error
	float value;			// present input variable value
	float error;			// present error value
	float l_error;			// last error value
	float derivative;		// present derivative value
	float integral;			// present integral value
	uint32_t sample_tm;		// runtime sample counter
}stPIDCtrl;

typedef struct tag_stMotor
{
	union
	{
		struct
		{
			uint8_t direction:2;
			uint8_t run:1;
			uint8_t update:1;
			uint8_t :2;
			uint8_t KeybCtrl:1;
			uint8_t Auto:1;
		}bits;
		uint8_t reg;
	}status;

	union
	{
		struct
		{
			uint8_t top_east:1;
			uint8_t top_west:1;
			uint8_t br1_overload:1;
			uint8_t br2_overload:1;
		}bits;
		uint8_t reg;
	}error;

	uint16_t command;
	float duty;
	float limit_max;
	float limit_min;
	uint32_t nMov;
}stMotor;

typedef struct tag_stPVDrive
{
	union
	{
		struct
		{
			uint8_t direction:2;
			uint8_t run:1;
			uint8_t floating:1;
			uint8_t battfull:1;
			uint8_t :3;
		}bits;
		uint8_t reg;
	}status;

	union
	{
		struct
		{
			uint8_t top_voltage:1;
			uint8_t top_current:1;
			uint8_t overvoltage:1;
			uint8_t overload:1;
			uint8_t batt_Tout:1;
		}bits;
		uint8_t reg;
	}error;

	uint16_t command;
	float target;
	float value;
	float duty;
	float duty_max;
	float duty_min;
	uint32_t charging_tm;
	float T_threshold;
	float k_temp;
	float *I_batt;
	float *V_batt;
	float *I_PV;
	float *V_PV;
	float *T_batt;
	uint32_t state;
}stPVDrive;


stPIDCtrl *GetMotorPIDCtrl(void);
stMotor *GetMotorDrive(void);
stPVDrive *GetPVDrive(void);
void PID_Init(void);
void ComputePIDMotor(stPIDCtrl *);
void ComputePID_PV(stPIDCtrl *);
float PID_Compute(stPIDCtrl *);
void PWM_SetDuty(FTM_Type *, ftm_chnl_t , float);
void MotorControl(stMotor *);
void PV_Control(stPVDrive *);
void PID_RunTime(void);

#endif /* CONTROL_H_ */
