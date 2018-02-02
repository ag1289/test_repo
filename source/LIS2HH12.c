/*
 * LIS2HH12.c
 *
 *  Created on: 9 nov. 2017
 *      Author: agarcia
 */

#include <fastmath.h>
#include "MK22F12810.h"
#include "fsl_common.h"
#include "user.h"
#include "fsl_i2c.h"
#include "fsl_lptmr.h"
#include "LIS2HH12.h"

volatile float Tilt_Angle[3] = {0.0, 0.0, 0.0};

//volatile AxesRaw_t Ref_Vector = {-1440, -430, 10605};
volatile AxesRaw_t Ref_Vector = {-16592, 656, 349};
volatile AxesRaw_t Pos_Vector = {0, 0, 0};

volatile uint32_t process_tm= 0;

int32_t LIS2HH_ReadReg(uint8_t Reg, uint8_t *val)
{
	return I2C_XferBuffer(LIS2HH_SA0_L_ADDR, Reg, 1, kI2C_Read, val, 1);
}

int32_t LIS2HH_WriteReg(uint8_t Reg, uint8_t *val)
{
	return I2C_XferBuffer(LIS2HH_SA0_L_ADDR, Reg, 1, kI2C_Write, val, 1);
}

AxesRaw_t *GetReferenceVector(void)
{
	return (AxesRaw_t *)&Ref_Vector;
}


/*******************************************************************************
* Function Name  : LIS2HH_GetStatusReg
* Description    : Read the status register
* Input          : char to empty by Status Reg Value
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
int32_t LIS2HH_GetStatusReg(uint8_t* val) {
  if( LIS2HH_ReadReg(LIS2HH_STATUS, val) != kStatus_Success )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS2HH _GetAccAxesRaw
* Description    : Read the Acceleration Values Output Registers
* Input          : buffer to empity by AxesRaw_t Typedef
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
int32_t LIS2HH_GetAccAxesRaw(AxesRaw_t* buff)
{
	uint32_t result ;
	int16_t raw_data[3];

	result = I2C_XferBuffer(LIS2HH_SA0_L_ADDR, LIS2HH_OUT_X_L, 1, kI2C_Read, (uint8_t *)raw_data, sizeof(raw_data));

	buff->X = (int32_t)raw_data[0];
	buff->Y = (int32_t)raw_data[1];
	buff->Z = (int32_t)raw_data[2];

	return result;
}

int32_t LIS2HH_Init(void)
{
	uint32_t result = kStatus_Fail;
	uint8_t id = 0, data = 0;

	for(uint32_t x = 0; x < 3; x++)
	{
		result = LIS2HH_ReadReg(LIS2HH_WHO_AM_I, &id);
		if(result == kStatus_Success && id == LIS2HH_ID)
		{
			data = LIS2HH_HR | STM_LIS2HH_ODR800 | LIS2HH_XEN | LIS2HH_YEN | LIS2HH_ZEN;
			result = LIS2HH_WriteReg(LIS2HH_CTRL1, &data);
			if(result == kStatus_Success)
			{
				data = 0x00;	// Cutoff frequency for high resolution set at ODR / 50
				result = LIS2HH_WriteReg(LIS2HH_CTRL2, &data);
				if(result == kStatus_Success)
				{
					data = 0x01;	 // Data Ready Interrupt on INT1
					result = LIS2HH_WriteReg(LIS2HH_CTRL3, &data);
					if(result == kStatus_Success)
					{
						data = 0x01;	 // Data Ready Interrupt on INT1
						result = LIS2HH_WriteReg(LIS2HH_CTRL3, &data);
						result = LIS2HH_ReadReg(LIS2HH_CTRL4, &data);
						if(result == kStatus_Success && !(data & LIS2HH_ADD_INC))
						{
							data |= LIS2HH_ADD_INC;
							result = LIS2HH_WriteReg(LIS2HH_CTRL4, &data);
							break;
						}
					}
				}
			}
		}

		if(result == kStatus_Success)
			break;
		else
		{
			I2C_MasterStop(I2C1);
			I2C_ClkBurst();
		}
	}
	return result;
}

float* ComputeTiltAngles(AxesRaw_t* accel)
{
	LPTMR_SetTimerPeriod(LPTMR0, 0xffff);
	LPTMR_StartTimer(LPTMR0);

	float Ax = (float)accel->X, Ay = (float)accel->Y, Az = (float)accel->Z;
	float Ax2 = Ax * Ax, Ay2 = Ay * Ay ,Az2 = Az * Az;
	float k_g = 180.0 / M_PI;

	Tilt_Angle[LIS2HH_PITCH] = k_g * atan2f(sqrtf(Ay2 + Az2), Ax) - 90.0;
	Tilt_Angle[LIS2HH_ROLL] = k_g * atan2f(sqrtf(Ax2 + Az2), Ay) - 90.0;
	Tilt_Angle[LIS2HH_YAW] = k_g * atan2f(sqrtf(Ax2 + Ay2), Az) - 90.0;

	process_tm = LPTMR_GetCurrentTimerCount(LPTMR0);
	LPTMR_StopTimer(LPTMR0);
	return (float *)Tilt_Angle;
}


float Calc2VectorAngle(AxesRaw_t *v1, AxesRaw_t *v2)
{
	float Mv1, Mv2, Div;
	float Mvf1, Mvf2, AlphaCos, Alpha;
	float a[3] = {(float)v1->X, (float)v1->Y, (float)v1->Z};
	float b[3] = {(float)v2->X, (float)v2->Y, (float)v2->Z};


	Mv1 = a[0] * a[0] + a[1] * a[1] + a[2] * a[2];
	Mv2 = b[0] * b[0] + b[1] * b[1] + b[2] * b[2];
	Div = a[0] * b[0] + a[1] * b[1] + a[2] * b[2];

	Mvf1 = sqrtf((float)Mv1);
	Mvf2 = sqrtf((float)Mv2);

	AlphaCos = (float)Div / (Mvf1 * Mvf2);

	Alpha = LIS2HH_K_R2G * acosf(AlphaCos);	// Compute Alpha and covert from radiant to grade

	return Alpha;
}

 void VectorialProduct( AxesRaw_t *v1, AxesRaw_t *v2, AxesRaw_t *vp)
 {
	 vp->X = v1->Y * v2->Z - v1->Z * v2->Y;
	 vp->Y = v1->X * v2->Z - v1->Z * v2->X;
	 vp->Z = v1->X * v2->Y - v1->Y * v2->X;
 }

 void CalculateNormalVector(AxesRaw_t *accel, AxesRaw_t *vp)
 {
	 /*AxesRaw_t v1 = {accel->X, accel->Y, 0};
	 AxesRaw_t v2 = {0, accel->Y, accel->Z};*/

	 AxesRaw_t v1 = {accel->X, 0, accel->Z};
	 AxesRaw_t v2 = {accel->X, accel->Y, accel->Z};

	 VectorialProduct(&v1, &v2, vp);
 }

 float ComputePlaneAngle(AxesRaw_t *accel)
 {
	 float angle;
	 AxesRaw_t vp1, vp2;
	 LPTMR_SetTimerPeriod(LPTMR0, 0xffff);
	 LPTMR_StartTimer(LPTMR0);

	 CalculateNormalVector(accel, &vp1);
	 CalculateNormalVector((AxesRaw_t *)&Ref_Vector, &vp2);

	 angle = Calc2VectorAngle(&vp1, &vp2);

	 if(vp1.X > 0)
		 angle= -angle;

	 process_tm = LPTMR_GetCurrentTimerCount(LPTMR0);
	 LPTMR_StopTimer(LPTMR0);
	 return angle;
 }

 int32_t CaptureRefVector(void)
 {
	uint8_t status;
	uint32_t i;
	float x = 0, y = 0, z = 0;
	AxesRaw_t axes;

	for(i = 0; i < 200; i++)
	{
		if((LIS2HH_GetStatusReg(&status) == kStatus_Success))
		{
			if(status & LIS2HH_STATUS_REG_ZYXDA)
			{
				if(LIS2HH_GetAccAxesRaw(&axes) == kStatus_Success)
				{
					x = (49 * x + (float)axes.X)/50;
					y = (49 * y + (float)axes.Y)/50;
					z = (49 * z + (float)axes.Z)/50;
				}
				else
					break;
			}
		}
	}

	if(i >= 100)
	{
		 Ref_Vector.X = (int32_t)x;
		 Ref_Vector.Y = (int32_t)y;
		 Ref_Vector.Z = (int32_t)z;
		 return 1;
	}

	return 0;
 }




