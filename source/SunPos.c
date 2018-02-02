/*
 * SunPos.c
 *
 *  Created on: 20 nov. 2017
 *      Author: agarcia
 */

#include "MK22F12810.h"
#include "math.h"
#include "SunPos.h"
#include "Control.h"
#include "Analog.h"

double sunrise, sunset;
double LSTM, EoT, B, TC, LST, HRA, declination;
double dSin, dCos, lSin, lCos, HRAcos;

const uint8_t DayByMonth[13] = { 0, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };
uint32_t bSunPos = 0;

stSunP st_SunParam ={.datetime = NULL, .Azimut = NAN, .Elevacion = NAN, .ZoneTime = 1, .Latitud = 41.77786471, .Longitud = 1.85845435};

/////////////////////////////////////////////////////////////
// Función para calcular el día del año 1 Enero=dia 1	  //
///////////////////////////////////////////////////////////

int16_t DayOfYear(int16_t day, int16_t month, int16_t year) {
	int16_t x, y, leap = 0;

	if (!(year & 0x0003))
		leap = 1;

	y = 0;
	for (x = 1; x < month; x++)
		y += DayByMonth[x];

	y += day;

	if (month > 2)
		y += leap;

	return y;
}

int32_t computeSunPos(int32_t trigger )
{
	double auxiliar;

	if(st_SunParam.datetime == NULL)
		return -1;

	switch (bSunPos)
	{
		case 0:
			if(trigger)
				bSunPos++;
			break;
		case 1:

			auxiliar = (double) DayOfYear(st_SunParam.datetime->tm_mday, st_SunParam.datetime->tm_mon + 1, st_SunParam.datetime->tm_year + 1900);
			LSTM = 15 * (double)st_SunParam.ZoneTime;
			B = K_GRAD2RAD * 360 * (auxiliar - 81) / 365;
			declination = 23.45 * sin(B);
			bSunPos++;
			break;
		case 2:
			EoT = 9.87 * sin(2 * B);
			bSunPos++;
			break;
		case 3:
			EoT = EoT - 7.53 * cos(B);
			bSunPos++;
			break;
		case 4:
			EoT = EoT - 1.5 * sin(B); // minutos
			bSunPos++;
			break;

		case 5:
			TC = 4 * (K_GRAD2RAD * (double)st_SunParam.Longitud - LSTM) + EoT;
			LST = (60 * (float)st_SunParam.datetime->tm_hour + (float)st_SunParam.datetime->tm_min + TC) / 60;

			HRA = K_GRAD2RAD * (15 * (LST - 12));

			declination = K_GRAD2RAD * declination; // en radianes

			bSunPos++;

			break;

		case 6:

			dSin = sin(declination);

			bSunPos++;

			break;

		case 7:

			dCos = cos(declination);

			bSunPos++;

			break;

		case 8:

			lSin = sin(K_GRAD2RAD * (double)st_SunParam.Latitud);

			bSunPos++;

			break;
		case 9:
			lCos = cos(K_GRAD2RAD * (double)st_SunParam.Latitud);
			HRAcos = cos(HRA);

			bSunPos++;
			break;

		case 10:
			st_SunParam.Elevacion = dSin * lSin + dCos * lCos * HRAcos;

			bSunPos++;
			break;

		case 11:
			st_SunParam.Elevacion = asin(st_SunParam.Elevacion);

			bSunPos++;
			break;

		case 12:
			st_SunParam.Azimut = dSin * lCos;
			st_SunParam.Azimut -= dCos * lSin * HRAcos;

			bSunPos++;
			break;
		case 13:
			st_SunParam.Azimut = st_SunParam.Azimut / cos(st_SunParam.Elevacion);

			bSunPos++;
			break;

		case 14:
			st_SunParam.Azimut = acos(st_SunParam.Azimut);

			bSunPos++;
			break;

		case 15:
			sunrise = acos(-lSin * dSin / (lCos * dCos));
			sunrise = K_RAD2GRAD * sunrise;
			sunrise = 12 - sunrise / 15 - TC / 60;

			bSunPos++;
			break;

		case 16:
			sunset = acos(-lSin * dSin / (lCos * dCos));
			sunset = K_RAD2GRAD * sunset;
			sunset = 12 + sunset / 15 - TC / 60;

			bSunPos++;
			break;

		case 17:
			st_SunParam.Elevacion = K_RAD2GRAD * st_SunParam.Elevacion; // en grados
			st_SunParam.Azimut = K_RAD2GRAD * st_SunParam.Azimut; // En grados

			if (LST > 12)
				st_SunParam.Azimut = 360 - st_SunParam.Azimut;

			if (st_SunParam.Elevacion > 0)							// Si es de día
			{
				auxiliar = st_SunParam.Elevacion - floor(st_SunParam.Elevacion);
				if (auxiliar >= 0.5)
					st_SunParam.Elevacion += 1;

				auxiliar = st_SunParam.Azimut - floor(st_SunParam.Azimut);
				if (auxiliar >= 0.5)
					st_SunParam.Azimut += 1;
			}
			else
			{
				st_SunParam.Azimut = 180.0;					// Si es de noche
				st_SunParam.Elevacion = 0;
			}

			st_SunParam.iElevacion = (int16_t)(10 * st_SunParam.Elevacion);
			st_SunParam.iAzimut = (int16_t)(10 * st_SunParam.Azimut);
			st_SunParam.ipElevation = (int16_t)(10 * GetMotorPIDCtrl()[0].value);
			st_SunParam.T_Board = (int16_t)(*GetAnalogData(eADCCHN_T_PCB) * 10);
			st_SunParam.T_Batt = (int16_t)(*GetAnalogData(eADCCHN_T_BATT) * 10);

			sunrise = modf(sunrise, &auxiliar);
			st_SunParam.hSunRise = (int16_t) auxiliar;
			sunrise = 60 * sunrise;
			st_SunParam.mSunRise = (int16_t) sunrise;

			sunset = modf(sunset, &auxiliar);
			st_SunParam.hSunSet = (int16_t) auxiliar;
			sunset = 60 * sunset;
			st_SunParam.mSunSet = (int16_t) sunset;
			bSunPos = 0;
			break;
		default:
			bSunPos = 0;
			break;
	}
	return bSunPos;
}

stSunP *GetSunParam(void)
{
	return &st_SunParam;
}

float GetSunAzimut(void)
{
	if(!bSunPos)
		return (float)st_SunParam.Azimut - 180;	// Ajust for 0 at sun zenith
	else
		return NAN;
}

float GetSunElevation(void)
{
	if(!bSunPos)
		return (float)st_SunParam.Elevacion;	// Ajust for 0 at sun zenith
	else
		return NAN;
}


void InitSunPos(struct tm *pDt)
{
	st_SunParam.datetime = pDt;
}
