/*
 * user.h
 *
 *  Created on: 7 nov. 2017
 *      Author: agarcia
 */

#ifndef USER_H_
#define USER_H_

/* CRC16  polynomial definition  */

#define	P_CRC16			0xA001
#define INIT_CRC16		0xFFFF


void SysTick_Handler(void);
uint16_t crc16_update(uint16_t, uint8_t);
uint16_t crc16(uint8_t *, uint16_t);
uint32_t GetTickCount(void);
void InitSystick(void);
uint32_t TimeDiff(uint32_t , uint32_t);
void InitTimer_us(void);
void delay_us(uint16_t time);
void delay_ms(uint32_t time);
void I2C_ClkBurst(void);
void I2C_Init(void);
int32_t I2C_XferBuffer(uint8_t , uint32_t , uint8_t , uint8_t , uint8_t *, uint16_t);
uint32_t Dig8_Bin2BCD(uint32_t);
uint32_t fBin2BCD(uint8_t);
uint32_t fBCD2Bin(uint8_t);
void PWM_Init(void);

#endif /* USER_H_ */
