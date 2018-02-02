/*
 * Analog.c
 *
 *  Created on: 22 nov. 2017
 *      Author: agarcia
 */

#include <string.h>
#include <math.h>
#include "MK22F12810.h"
#include "fsl_pmc.h"
#include "fsl_adc16.h"
#include "fsl_clock.h"
#include "user.h"
#include "Analog.h"
#include "Control.h"

uint32_t bConvert_ADC0 = 0x80, bConvert_ADC1 = 0x80, adc0_idx = 0, adc1_idx = 0;
volatile float hig_volt_dis = CUTOFF_HIGVBAT;

adc16_config_t adc16ConfigStruct;
stAnalogData ADC0_ChnCfg[ADC0_CHANNELS], ADC1_ChnCfg[ADC1_CHANNELS];
/*
** ===================================================================
**     Interrupt handler : ADC0_IRQHandler
**
**     Description :
**         User interrupt service routine.
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/
void ADC0_IRQHandler(void)
{
	uint32_t data = ADC0->R[0];

	if(bConvert_ADC0 & 0xc0)
	{
		bConvert_ADC0 |= 0x40;
	}
	else
	{
		bConvert_ADC0 = adc0_idx | 0x80;
		ADC0_ChnCfg[adc0_idx++].raw = data;
		if(adc0_idx > (ADC0_CHANNELS -1))
			adc0_idx = 0;
	}
}

/*
** ===================================================================
**     Interrupt handler : ADC1_IRQHandler
**
**     Description :
**         User interrupt service routine.
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/
void ADC1_IRQHandler(void)
{
	uint32_t data = ADC1->R[0];

	if(bConvert_ADC1 & 0xc0)
	{
		bConvert_ADC1 |= 0x40;
	}
	else
	{
		bConvert_ADC1 = adc1_idx | 0x80;
		ADC1_ChnCfg[adc1_idx++].raw = data;
		if(adc1_idx > (ADC1_CHANNELS -1))adc1_idx = 0;

	}
}

float *GetAnalogData(eADCCHN channel)
{
	if(channel & 0x80)
	{
		channel &= 0xf;
		if(channel < ADC1_CHANNELS)
			return &ADC1_ChnCfg[channel].mValue;
	}
	else
	{
		if(channel < ADC0_CHANNELS)
			return &ADC0_ChnCfg[channel].mValue;
	}
	return NULL;
}

void Analog_Init()
{
	pmc_bandgap_buffer_config_t bgap_cfg;	// Enable Bandgap buffer

	bgap_cfg.enable = true;
	bgap_cfg.enableInLowPowerMode = false;

	PMC_ConfigureBandgapBuffer(PMC, &bgap_cfg);

	adc16ConfigStruct.referenceVoltageSource = kADC16_ReferenceVoltageSourceVref;
	adc16ConfigStruct.clockSource = kADC16_ClockSourceAlt0;		// Select bus clock 95.977472MHz / 2  as ADC16 clock
	adc16ConfigStruct.enableAsynchronousClock = false;
	adc16ConfigStruct.clockDivider = kADC16_ClockDivider4;		// Select divide /4 for ADCclk = 12MHz
	adc16ConfigStruct.resolution = kADC16_Resolution16Bit;
	adc16ConfigStruct.longSampleMode = kADC16_LongSampleCycle6;
	adc16ConfigStruct.enableHighSpeed = true;
    adc16ConfigStruct.enableLowPower = false;
    adc16ConfigStruct.enableContinuousConversion = false;

	ADC16_Init(ADC0, &adc16ConfigStruct);
	ADC16_Init(ADC1, &adc16ConfigStruct);

	ADC16_EnableHardwareTrigger(ADC0, false);
	ADC16_EnableHardwareTrigger(ADC1, false);

	NVIC_SetPriority(ADC0_IRQn, 4);
	NVIC_EnableIRQ(ADC0_IRQn);

	NVIC_SetPriority(ADC1_IRQn, 4);
	NVIC_EnableIRQ(ADC1_IRQn);

	for(uint32_t x = 0; x < ADC0_CHANNELS; x++)
	{
		ADC0_ChnCfg[x].chCfg.nADC = ADC0;
		ADC0_ChnCfg[x].chCfg.nChannel = kADC16_ChannelMuxA;
		ADC0_ChnCfg[x].mValue = 0;
		ADC0_ChnCfg[x].pValue = 0;
		ADC0_ChnCfg[x].Cfg.AN.Offset = 0;
		ADC0_ChnCfg[x].Cfg.AN.Slope = 1;
		ADC0_ChnCfg[x].Cfg.samples = 10;
		ADC0_ChnCfg[x].analog_handle = (void *)&ProcessAnalog;
	}

	ADC0_ChnCfg[0].chCfg.nChannel = AN_I_BATT;
	ADC0_ChnCfg[0].Cfg.AN.Slope = KI_BATT;
	ADC0_ChnCfg[0].Cfg.AN.Offset = I_BATT_OFFSET;

	ADC0_ChnCfg[1].chCfg.nChannel = AN_V_BATT;
	ADC0_ChnCfg[1].Cfg.AN.Slope = KV_BAT;
	ADC0_ChnCfg[1].Cfg.AN.Offset = V_BAT_OFFSET;
	ADC0_ChnCfg[1].analog_handle = (void *)&ProcessVbat;

	ADC0_ChnCfg[2].chCfg.nChannel = AN_VREF;

	ADC0_ChnCfg[3].Cfg.NTC.B= NTC_B;
	ADC0_ChnCfg[3].Cfg.NTC.R0 = NTC_R25;
	ADC0_ChnCfg[3].chCfg.nChannel = AN_T_PCB;
	ADC0_ChnCfg[3].analog_handle = (void *)&ProcessNTC;

	ADC0_ChnCfg[4].Cfg.NTC.B= NTC_B;
	ADC0_ChnCfg[4].Cfg.NTC.R0 = NTC_R25;
	ADC0_ChnCfg[4].chCfg.nChannel = AN_T_BAT;
	ADC0_ChnCfg[4].analog_handle = (void *)&ProcessNTC;

	for(uint32_t x = 0; x < ADC1_CHANNELS; x++)
	{
		ADC1_ChnCfg[x].chCfg.nADC = ADC1;
		ADC1_ChnCfg[x].chCfg.nChannel = kADC16_ChannelMuxA;
		ADC1_ChnCfg[x].mValue = 0;
		ADC1_ChnCfg[x].pValue = 0;
		ADC1_ChnCfg[x].Cfg.AN.Offset = 0;
		ADC1_ChnCfg[x].Cfg.AN.Slope = 1;
		ADC1_ChnCfg[x].Cfg.samples = 10;
		ADC1_ChnCfg[x].analog_handle = (void *)&ProcessAnalog;
	}


	ADC1_ChnCfg[0].chCfg.nChannel = AN_I_PV;
	ADC1_ChnCfg[0].Cfg.AN.Slope = KI_PV;
	ADC1_ChnCfg[0].Cfg.AN.Offset = I_PV_OFFSET;

	ADC1_ChnCfg[1].chCfg.nChannel = AN_V_PV;
	ADC1_ChnCfg[1].Cfg.AN.Slope = KV_PV;
	ADC1_ChnCfg[1].Cfg.AN.Offset = V_PV_OFFSET;

	ADC1_ChnCfg[2].chCfg.nChannel = AN_IS_BR1;
	ADC1_ChnCfg[3].chCfg.nChannel = AN_IS_BR2;
	ADC1_ChnCfg[4].chCfg.nChannel = AN_BGAP;

	ADC1_ChnCfg[2].chCfg.mux_mode= kADC16_ChannelMuxB;
	ADC1_ChnCfg[3].chCfg.mux_mode = kADC16_ChannelMuxB;

}

void ProcessVbat(stAnalogData *pData)
{
	stPVDrive *pPV;
	ProcessAnalog(pData);

	if(pData->mValue > hig_volt_dis)
	{
		pPV = GetPVDrive();
		pPV->command = eDRVCMD_STOP;
		PV_Control(pPV);
		pPV->error.bits.overvoltage = 1;
		pPV->state = ePVDRV_STOP;
	}
	else if(pData->mValue < MAX_VBAT)
	{
		pPV = GetPVDrive();
		pPV->error.bits.overvoltage = 0;
	}
}

void ProcessAnalog(stAnalogData *pData)
{
	float x, y;

	x = KADC16 * (float)pData->raw;
	y = pData->pValue =  pData->Cfg.AN.Slope * x + pData->Cfg.AN.Offset;	// Scaled an corrected value
	pData->mValue = (pData->mValue * (pData->Cfg.samples - 1.0)  + y ) / pData->Cfg.samples;
}


void ProcessNTC(stAnalogData *pData)
{
	float V, result, R;

	V = KADC16 * (float)pData->raw;

	R = ((ADCVREF / V) - 1) * RD_NTC;

	result = NTC_K_T0 + (1 / pData->Cfg.NTC.B) * logf(R / pData->Cfg.NTC.R0);
	result = (1.0 / result) - ZERO_K;

	pData->mValue = (pData->mValue * (pData->Cfg.samples - 1.0)  + result ) / pData->Cfg.samples;

}

void ADC16_SetChannel(ADC_Type *base, uint32_t channelGroup, const adc16_channel_config_t *config)
{
    assert(channelGroup < ADC_SC1_COUNT);
    assert(NULL != config);

    uint32_t sc1 = ADC_SC1_ADCH(config->channelNumber); /* Set the channel number. */

#if defined(FSL_FEATURE_ADC16_HAS_DIFF_MODE) && FSL_FEATURE_ADC16_HAS_DIFF_MODE
    /* Enable the differential conversion. */
    if (config->enableDifferentialConversion)
    {
        sc1 |= ADC_SC1_DIFF_MASK;
    }
#endif /* FSL_FEATURE_ADC16_HAS_DIFF_MODE */
    /* Enable the interrupt when the conversion is done. */
    if (config->enableInterruptOnConversionCompleted)
    {
        sc1 |= ADC_SC1_AIEN_MASK;
    }
    base->SC1[channelGroup] = sc1;
}

void AnalogRunTime()
{
	adc16_channel_config_t ch_config;

	ch_config.enableDifferentialConversion = false;
	ch_config.enableInterruptOnConversionCompleted = true;

	if(bConvert_ADC0 & 0x80)
	{
		ch_config.channelNumber = ADC0_ChnCfg[adc0_idx].chCfg.nChannel;

		if((bConvert_ADC0 & 0xf) < ADC0_CHANNELS)
			ADC0_ChnCfg[bConvert_ADC0 & 0xf].analog_handle(&ADC0_ChnCfg[bConvert_ADC0 & 0xf]);

		ADC16_SetChannelConfig(ADC0_ChnCfg[adc0_idx].chCfg.nADC,ADC0_ChnCfg[adc0_idx].chCfg.mux_mode, &ch_config);
		bConvert_ADC0 = 0;
	}

	if(bConvert_ADC1 & 0x80)
	{
		ch_config.channelNumber = ADC1_ChnCfg[adc1_idx].chCfg.nChannel;

		if((bConvert_ADC1 & 0xf) < ADC1_CHANNELS)
			ADC1_ChnCfg[bConvert_ADC1 & 0xf].analog_handle(&ADC1_ChnCfg[bConvert_ADC1 & 0xf]);

		ADC16_SetChannelConfig(ADC1_ChnCfg[adc1_idx].chCfg.nADC, ADC1_ChnCfg[adc1_idx].chCfg.mux_mode, &ch_config);
		bConvert_ADC1 = 0;
	}

}
