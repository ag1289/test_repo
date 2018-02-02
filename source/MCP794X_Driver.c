#include <fsl_i2c.h>
#include <string.h>
#include "MCP794X_Driver.h"
#include "user.h"

static RTCCtrl stRTCtbl;


uint32_t fRdSRAM(uint8_t *pData, uint16_t InitAddr, uint16_t nbytes)
{
	if(I2C_XferBuffer(ADDR_SRAM, InitAddr, 1, kI2C_Read, pData, nbytes) == kStatus_Success)
		return nbytes;

	return 0;
}

// *****************************************************************************
// fWrSRAM()	 Writes data to E2PROM by Angel Garcia
// v1.00	05/09/12   ... created ...
// This routine writes data to SRAM
//	Input:				Data type:		Values:
//	------				----------		-------
//	*pData				uint8_t            pointer to data buffer
//	InitAddr			uint16_t 	SRAM addres to start writing data
//	nbyte	 			uint16_t	Number of bytes to write
//	Output:
//	-------
//	0 if fault
//	number of writen bytes
// *****************************************************************************

uint32_t fWrSRAM(uint8_t *pData, uint16_t InitAddr, uint16_t nbytes)
{
	if(I2C_XferBuffer(ADDR_SRAM, InitAddr, 1, kI2C_Write, pData, nbytes) == kStatus_Success)
		return nbytes;

	return 0;
}
// *****************************************************************************
// fWrE2prom()	 Writes data to E2PROM by Angel Garcia			      	      
// v1.00	05/09/12   ... created ...					      
// This routine Writes data to E2PROM	      
//	Input:				Data type:		Values:
//	------				----------		-------
//	*pData				uint8_t            pointer to data buffer
//	InitAddr			uint16_t 	E2Prom addres to start writing data
//	nbyte	 			uint16_t	Number of bytes to write
//	Output:
//	-------
//	0 if fault
//	number of writen bytes 
// *****************************************************************************

uint32_t fWrE2prom(uint8_t *pData, uint16_t InitAddr, uint16_t nbytes)
{
	uint8_t Status;
	uint32_t x = 0, data_length = 0;

	if (InitAddr == ORG_IDCODE48 && nbytes < 7);
	else if (!nbytes || (InitAddr + nbytes) > E2ROM_BYTES)
		return 0;

	while (nbytes)
	{
		InitAddr += x;

		if (nbytes >= E2ROM_PAGE_BYTES)
		{
			data_length = E2ROM_PAGE_BYTES;
			nbytes -= E2ROM_PAGE_BYTES;
		}
		else
		{
			data_length = nbytes;
			nbytes = 0;
		}
		
		x += data_length;

		if(I2C_XferBuffer(ADDR_EEPROM, InitAddr, 1, kI2C_Write, pData + x, data_length) != kStatus_Success)
		{
			x = 0;
			break;
		}

		while(fRdE2prom(&Status, ADDR_EEPROM_SR, 1) == RTC_ERROR);
		delay_us(50);
	};
	return x;
}

// *****************************************************************************
// fRdE2prom()	 Reads data from E2PROM by Angel Garcia			      	      
// v1.00	05/09/12   ... created ...					      
// This routine reads data from E2PROM	      
//	Input:				Data type:		Values:
//	------				----------		-------
//	*pData				uint8_t            pointer to read data buffer
//	InitAddr			uint16_t 	E2Prom addres to start writing data
//	nbyte	 			uint16_t	Number of bytes to write
//	Output:
//	-------
//	0 if fault
//	number of writen bytes 
// *****************************************************************************

uint32_t fRdE2prom(uint8_t *pData, uint16_t InitAddr, uint16_t nbytes)
{

    if ((InitAddr == ORG_IDCODE48 && nbytes < 7) || InitAddr == ADDR_EEPROM_SR);
    else if (!nbytes || (InitAddr + nbytes) > E2ROM_BYTES)return RTC_ERROR;

	if(I2C_XferBuffer(ADDR_EEPROM, InitAddr, 1, kI2C_Read, pData, nbytes) == kStatus_Success)
		return nbytes;
	
	return 0;
}

uint32_t fRdCtrlReg(uint8_t *data)
{
    if (fRdSRAM(data, ADDR_CTRL, 1))
    {
        return 1;
    }
    return 0;
}

uint32_t fRdStatReg(uint8_t *data)
{
    if (fRdSRAM(data, ADDR_STAT, 1))
    {
        return 1;
    }
    return 0;
}

uint32_t fWrCtrlReg(uint8_t *data)
{
    return (fWrSRAM(data, ADDR_CTRL, 1));
}

uint32_t fWrStatReg(uint8_t *data)
{
    return (fWrSRAM(data, ADDR_STAT, 1));
}

uint32_t fSetTime(struct tm *DTime)
{
    uint8_t data[3];
    data[0] = 0x80 | fBin2BCD(DTime->tm_sec); // Every set activates oscillator
    data[1] = fBin2BCD(DTime->tm_min);
    data[2] = fBin2BCD(DTime->tm_hour);
    return fWrSRAM(data, ADDR_SEC, 3);
}

uint32_t fSetDate(struct tm *DTime)
{
    uint8_t data[4];
	
	fRdSRAM(data, ADDR_DAY, 1);
	
	data[0] &= 0xf8;

	if(!DTime->tm_wday)
		data[0] |= 0x07;
	else
		data[0] |= DTime->tm_wday & 0x07;

    data[1] = fBin2BCD(DTime->tm_mday);
    data[2] = fBin2BCD(DTime->tm_mon + 1);	// MCP7941 month starts on 1 for January and local Broken-down Time starts on 0 for January
    data[3] = fBin2BCD((uint8_t) (DTime->tm_year + CUT_YEAR - RTC_CENTURY));
    return fWrSRAM(data, ADDR_DAY, 4);
}

uint32_t fGetTime(struct tm *DTime)
{
    uint8_t data[3];
    if (fRdSRAM(data, ADDR_SEC, 3)!= RTC_ERROR)
    {
        DTime->tm_sec = fBCD2Bin(data[ADDR_SEC] & 0x7f);
        DTime->tm_min = fBCD2Bin(data[ADDR_MIN] & 0x7f);
        DTime->tm_hour = fBCD2Bin(data[ADDR_HOUR] & 0x3f); // Changed from 0x1f a 0x3f el 06/07/2012
        if (!(data[ADDR_SEC] & START_32KHZ)) // If oscillator not enabled set ST bit
        {
            data[ADDR_SEC] |= START_32KHZ;
            if(fWrSRAM(data, ADDR_SEC, 1)!= RTC_ERROR);
        }
	
		return RTC_OK;
    }

	return RTC_ERROR;
}

uint32_t fGetDate(struct tm *DTime)
{
    uint8_t data[5];
    if (fRdSRAM(data, ADDR_DAY, 5) != RTC_ERROR)
    {
		stRTCtbl.Reg3.val = data[0];	// Load Status register
		stRTCtbl.Reg7.val = data[4];	// Load Control register

		if(stRTCtbl.Reg3.Bits.Day == 0x07)
			DTime->tm_wday = 0;
		else 
			DTime->tm_wday = stRTCtbl.Reg3.Bits.Day; // Sunday = 0 for tm strct and Sunday = 1 for RTC
		DTime->tm_mday = fBCD2Bin(data[1] & 0x3f);	

		DTime->tm_mon = fBCD2Bin(data[2] & 0x1f);	// MCP7941 month starts on 1 for January and local Broken-down Time starts on 0 for January
		
		if(DTime->tm_mon > 0)
			DTime->tm_mon--;
			
        DTime->tm_year = fBCD2Bin(data[3]) + (RTC_CENTURY - CUT_YEAR);

        if (!stRTCtbl.Reg3.Bits.VBAT_EN || stRTCtbl.Reg3.Bits.PWRFAIL) // If VBATEN=0 set it.
        {
			stRTCtbl.Reg3.Bits.PWRFAIL = 0;
            stRTCtbl.Reg3.Bits.VBAT_EN = 1;
            fWrStatReg((uint8_t *)&stRTCtbl.Reg3);
        }
		return RTC_OK;
    }
	return RTC_ERROR;
}

uint32_t fWriteID(uint8_t *pData)
{
    uint8_t ucode = UNLOCK_CODE1;
    if (fWrSRAM((uint8_t *)&ucode, ADDR_ULID, 1))
    {
        ucode = UNLOCK_CODE2;
        if (fWrSRAM((uint8_t *) &ucode, ADDR_ULID, 1))
        {
            return fWrE2prom(pData, ORG_IDCODE48, 6);
        }
    }
    return 0;
}

uint32_t fReadID48(uint8_t *pData)
{
    return fRdE2prom(pData, ORG_IDCODE48, 6);
}

uint32_t fReadUserSRAM(uint8_t *pData, uint16_t InitAddr, uint16_t nbytes)
{
	if( InitAddr + nbytes > SRAM_BYTES)return 0;
    return fRdSRAM(pData, SRAM_PTR + InitAddr, nbytes);
}

uint32_t fWriteUserSRAM(uint8_t *pData, uint16_t InitAddr, uint16_t nbytes)
{
	if( InitAddr + nbytes > SRAM_BYTES)return 0;
    return fWrSRAM(pData, SRAM_PTR + InitAddr, nbytes);
}

uint32_t fWriteE2prom(uint8_t *pData, uint16_t nbytes)
{
	if(nbytes > USER_E2PROM_SIZE)return 0;
    return fWrE2prom(pData, 0, nbytes);
}

uint32_t fReadE2prom(uint8_t *pData, uint16_t nbytes)
{
	if(nbytes > USER_E2PROM_SIZE)nbytes = USER_E2PROM_SIZE;
    return fRdE2prom(pData, 0, nbytes);
}

uint32_t fEraseE2Prom(void)
{
    uint8_t data[E2ROM_BYTES];
    memset(data, 0xff, E2ROM_BYTES);
    return fWrE2prom(data, 0, E2ROM_BYTES);
}

uint32_t fClearCalReg(void)
{
    uint8_t data = 0;
    if(fWrSRAM((uint8_t *) &data, ADDR_CAL, 1)!= RTC_ERROR)return RTC_OK;
	return RTC_ERROR;
}

uint32_t Init_RTC(void)
{
	int32_t error = kStatus_I2C_Nak;
	struct tm stDT;
	
	for(uint32_t x = 0; x < 3; x++)
	{
		
		if(fRdSRAM((uint8_t *)&stRTCtbl, ADDR_SEC, sizeof(stRTCtbl)) != RTC_ERROR) 
		{
			error = kStatus_Success;

			if(!stRTCtbl.Reg0.Bits.ST)
			{
				stRTCtbl.Reg0.Bits.ST =1;
				fWrSRAM((uint8_t *)&stRTCtbl.Reg0, ADDR_SEC, 1);
			}
		
			stRTCtbl.Reg7.val |= SQWE | MFP_64H; // Set divider at 64Hz frequency and enable MFP output
			fWrCtrlReg((uint8_t *)&stRTCtbl.Reg7);

			fClearCalReg();
			fGetTime(&stDT);
			fGetDate(&stDT);
			break;
		}
		else
			 I2C_ClkBurst();
	}
	return error;

}



