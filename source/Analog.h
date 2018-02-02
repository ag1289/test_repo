/*
 * Analog.h
 *
 *  Created on: 22 nov. 2017
 *      Author: agarcia
 */

#ifndef ANALOG_H_
#define ANALOG_H_

#include "MK22F12810.h"
#include "fsl_adc16.h"

#define ADC0_CHANNELS	5
#define ADC1_CHANNELS	5

#define ADCVREF		2.048
#define KADC16		(ADCVREF / 65536.0)

#define ZERO_K		273.15			// 0 ºC in Kelvin
#define NTC_K_T0	(1 / 298.15)	// 1/ T0 = 1 / (25 + 273.15) = 298.15K
#define NTC_R25		10000			// NTC temperature at 25ºC
#define RD_NTC		10000			// Reference down resistor value
#define NTC_B		3380			// Steinhart–Hart B (or β) parameter equation

// I_BATT constants

#define I_BATT_G		25
#define I_BATT_RS		25e-3
#define KI_BATT			(1 / (I_BATT_G * I_BATT_RS))
#define I_BATT_OFFSET	0

// I_PV constants

#define I_PV_G			25
#define I_PV_RS			10e-3
#define KI_PV			(1 / (I_PV_G * I_PV_RS))
#define I_PV_OFFSET		-(0.971 / (I_PV_G * I_PV_RS))

// V_PV constants
#define V_PV_RUP		15000.0
#define V_PV_RDOWN		680.0
#define KV_PV			((V_PV_RUP + V_PV_RDOWN) / V_PV_RDOWN)
#define V_PV_OFFSET		0

// V_BAT constants
#define V_BAT_RUP		15000.0
#define V_BAT_RDOWN		1000.0
#define KV_BAT			((V_BAT_RUP + V_BAT_RDOWN) / V_BAT_RDOWN)
#define V_BAT_OFFSET	0

// PV constants

#define MIN_PV_VOLT		9.0
#define MAX_PV_CURRENT	2.5

// Battery definitions
#define BAT_CAPACITY	3.2		// Battery capacity Ah
#define MAX_VBAT		28.4	// 8 cell x 3.55V
#define MIN_VBAT		17.6	// 8 cell x 2.2 V
#define CUTOFF_LOWVBAT	16		// 8 cell x 2.0 V
#define CUTOFF_HIGVBAT	30		// 8 cell x 2.0 V

#define I_FLOAT_CH		0.03	// Floating charge
#define I_STD_CHBAT		0.64	// Standard Charge And Discharge Current 0.2C
#define I_MAX_CHBAT		3.2 	// Max. Charge Current 1C
#define I_MAX_DISBAT	9.6 	// Max. Continual Discharge Current 3C
#define I_MAX_DISPEAK	16	 	// Max. Peak Discharge Current

#define MIN_CH_TEMP		0		// Charge minimum Temperature
#define MAX_CH_TEMP		45		// Charge maximum Temperature
#define MIN_DIS_TEMP	-20     // Discharge minimum Temperature
#define MAX_DIS_TEMP	60      // Discharge maximum Temperature


// ADC channel definition

#define AN_I_BATT	3	// ADC0 DP3
#define AN_V_BATT	9	// ADC0 SE9
#define AN_VREF		12	// ADC0 SE12
#define AN_T_PCB	13	// ADC0_SE13
#define AN_T_BAT	14	// ADC0_SE14

#define AN_I_PV		3	// ADC1 DP3
#define AN_V_PV		8	// ADC1 SE8
#define AN_IS_BR1	4	// ADC1_SE4B
#define AN_IS_BR2	5	// ADC1_SE5B
#define AN_BGAP		27	// ADC1_SE27 Bandgap 1V

typedef enum tag_ADCCHN
{
	eADCCHN_I_BATT = 0,
	eADCCHN_V_BATT,
	eADCCHN_VREF,
	eADCCHN_T_PCB,
	eADCCHN_T_BATT,
	eADCCHN_I_PV = 0x80,
	eADCCHN_V_PV,
	eADCCHN_IS_BR1,
	eADCCHN_IS_BR2,
	eADCCHN_BANDGAP,
	eADCCHN_
}eADCCHN;



typedef struct tag_st_ADCChn
{
	ADC_Type *nADC;			// Basse address of ADC module
	uint16_t nChannel;		// Number of channel
	uint16_t mux_mode;		// Mux selector a or b
}st_ADCChn;

typedef struct tag_stAnCfg
{
	union
	{
		struct
		{
			float Slope;
			float Offset;
		}AN;

		struct
		{
			float B;
			float R0;
		}NTC;
	};
	float samples;
}stAnCfg;

typedef struct tag_stAnalogData
{
	uint32_t raw;		// unprocessed adc read value
	float pValue;		// Instant raw value
	float mValue;		// Processed averaged value
	stAnCfg Cfg;		// Analog processing channel configuration
	st_ADCChn chCfg;	// ADC channel configuratin
	void (*analog_handle)(void *);	// function to process data
}stAnalogData;

float* GetAnalogData(eADCCHN);
void Analog_Init(void);
void ProcessVbat(stAnalogData *);
void ProcessAnalog(stAnalogData *);
void ProcessNTC(stAnalogData *);

void AnalogRunTime(void);

#endif /* ANALOG_H_ */
