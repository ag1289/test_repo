/*
 * SunPos.h
 *
 *  Created on: 20 nov. 2017
 *      Author: agarcia
 */

#ifndef SUNPOS_H_
#define SUNPOS_H_

#include <fastmath.h>
#include <time.h>

#define K_GRAD2RAD	(M_PI / 180)
#define K_RAD2GRAD	(180 / M_PI)

typedef struct tag_sunpos
{
	struct tm *datetime;
	float ZoneTime;
	float Latitud;		// En grados
	float Longitud;		// En grados
	double Elevacion;
	double Azimut;
	int16_t iElevacion;
	int16_t iAzimut;
	int16_t ipElevation;	// Present measured PV panels inclination respect ground plane
	int16_t hSunRise;
	int16_t mSunRise;
	int16_t hSunSet;
	int16_t mSunSet;
	int16_t T_Board;
	int16_t T_Batt;

} stSunP;

int32_t computeSunPos(int32_t);
float GetSunAzimut(void);
float GetSunElevation(void);
void InitSunPos(struct tm *);
stSunP *GetSunParam(void);

#endif /* SUNPOS_H_ */
