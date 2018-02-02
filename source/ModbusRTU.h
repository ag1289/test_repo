/*
 * ModbusRTU.h
 *
 * Created: 08/02/2016 8:58:54
 *  Author: agarcia
 */ 


#ifndef MODBUSRTU_H_
#define MODBUSRTU_H_

#define MASTER_PORT			1		// Number of master port
#define MBUS_RSPTOUT		6		// 600uS timeout for 19200baud
// #define Slave_ID			1		// Mod bus Salve address
#define MBUS_TIMEOUT		35		// Timeout in bit time
#define BACKDOOR_PASS		"webdom07"

#define LORA_PORT	0x80

/********************************************************************
These exceptions are defined in the MODBUS protocol.  These can be
used by the slave to communicate problems with the transmission back
to the master who can also use these to easily check the exceptions.
The first exception is the only one that is not part of the protocol
specification.  The TIMEOUT exception is returned when no slave
responds to the master's request within the timeout period.
********************************************************************/
typedef enum _Modbus_exception{NO_ERROR=0,ILLEGAL_FUNCTION=1,ILLEGAL_DATA_ADDRESS=2,
	ILLEGAL_DATA_VALUE = 3,SLAVE_DEVICE_FAILURE=4,ACKNOWLEDGE=5,SLAVE_DEVICE_BUSY=6,
	NEGATIVE_ACK = 7, MEMORY_PARITY_ERROR = 8,GATEWAY_PATH_UNAVAILABLE=10,GATEWAY_TARGET_NO_RESPONSE=11,
TIMEOUT=12, SLAVE_NOT_ME=15} exception;

/********************************************************************
These functions are defined in the MODBUS protocol.  These can be
used by the slave to check the incomming function.  See
ex_modbus_slave.c for example usage.
********************************************************************/
typedef enum _Modbus_function{FUNC_READ_COILS=0x01,FUNC_READ_DISCRETE_INPUT=0x02,
	FUNC_READ_HOLDING_REGISTERS=0x03,FUNC_READ_INPUT_REGISTERS=0x04,
	FUNC_WRITE_SINGLE_COIL=0x05,FUNC_WRITE_SINGLE_REGISTER=0x06,
	FUNC_READ_EXCEPTION_STATUS=0x07,FUNC_DIAGNOSTICS=0x08,
	FUNC_GET_COMM_EVENT_COUNTER=0x0B,FUNC_GET_COMM_EVENT_LOG=0x0C,
	FUNC_WRITE_MULTIPLE_COILS=0x0F,FUNC_WRITE_MULTIPLE_REGISTERS=0x10,
	FUNC_REPORT_SLAVE_ID=0x11,FUNC_READ_FILE_RECORD=0x14,
	FUNC_WRITE_FILE_RECORD=0x15,FUNC_MASK_WRITE_REGISTER=0x16,
FUNC_READ_WRITE_MULTIPLE_REGISTERS=0x17,FUNC_READ_FIFO_QUEUE=0x18} function;


// MODBUS define

#define MB_MAX_DATALENGTH		126	// Maximum data length for modbus datagram in uint16_t size

// Defines of MODBUS memory regions

#define MEAS_GROUP      0x00    // Measured registers and digital output register
#define DEVCFG_GROUP    0xF0    // Device configuration registers

//  Default configuration for MCU COM

#define COM_BUFFER	128		// Comunication buffer size

// Define value to execute commands

#define CMD_PASS 0xff00

typedef enum _eGROUP_HLD
{
	REG_RAW = 0,
	REG_DATETIME,
	REG_LORA,
	REG_CMD = 0xe0
}eGROUP_HLD;

typedef enum _eGROUP_SYSCFG
{
	REG_CFG_RAW = 0x10,
	REG_CFG_ANALOG,
	REG_CFG_STPDRV,
	REG_CFG_MODBUS,
	REG_CFG_ANOUT,
	REG_CFG_DIN,
	REG_CFG_DOUT
}eGROUP_SYSCFG;

// Data structures definition

typedef struct _stSerialPort
{
	struct {
		uint32_t bTX_InProgress:1;	// Transmission in progress bit
		uint32_t bFrame:1;			// Frame received bit
		uint32_t bRX_BufferFull:1;	// Receiver buffer is full
		uint32_t bOverrun:1;		// Overrun bit, data is not read when new data were received
		uint32_t: 28;
	};
	uint16_t RX_bytes;				// Received bytes
	uint16_t TX_bytes;				// Number of remaining TX bytes
	uint8_t *pTX_Data;				// Pointer to TX data
	uint16_t Error_cnt;				// Communication error counter
	uint16_t LastError;				// Last communiaction error
	uint32_t lastCom_tm;			// Last communication time in tick counts
	uint8_t  Buffer[COM_BUFFER];
}stSerialPort;

typedef struct _stCOM_Cfg
{
	uint16_t Mbus_sID;		// Modbus slave ID
	uint16_t Timeout;		// Serial timeout in serial bit time (Ej. 3.5*(startbit+databits+parity+stopbits)/BaudRate  for frame detection)
	uint32_t BaudRate;		// Serial Baud rate
	union{
		struct{
			uint32_t Chr_Length:4;
			uint32_t Parity:3;
			uint32_t Stop_bits:2;
			uint32_t :23;
		}MR_Bits;
		uint32_t MR_Reg;
	}Ctrl;
}stCOM_Cfg;

typedef struct _stPassword
{
	union
	{
		struct
		{
			uint32_t rd_hld:1;
			uint32_t wr_hld :1;
			uint32_t rd_cfg :1;
			uint32_t wr_cfg :1;
			uint32_t rd_firm :1;
			uint32_t wr_firm :1;
			uint32_t rd_password :1;
			uint32_t wr_password:1;
			uint32_t Master_EN:1;
			uint32_t :24;
		}bits;
		uint32_t Reg;
	}Accesslevel;
	uint32_t MasterPass_tm;			// Master password expiration time
	uint8_t cfg_password[8];		// Password to access and modify configuration parameters
}stPassword;

typedef struct _stMbusReadRSP
{
	uint8_t SlaveID;
	uint8_t Function;
	uint8_t nBytes;	// Byte count
	uint8_t Data[COM_BUFFER];
}stMbusReadRSP;

typedef struct _stMbusWriteRSP
{
	uint8_t SlaveID;
	uint8_t Function;
	uint16_t Start;
	uint16_t nRegisters;
	uint16_t CRC;
}stMbusWriteRSP;

typedef struct _stMbusRequest
{
	uint8_t SlaveID;			// Modbus Slave address
	uint8_t Function;			// Modbus Function
	uint16_t Start;             // Register start
	uint16_t Length;            // N� of register to read or write
	uint16_t nBytes;            // N� of bytes 2*Length
	uint16_t CRC;               // Modbus Frame CRC
	uint8_t Data[COM_BUFFER];   // Formation frame data buffer
	uint8_t *pData;             // Pointer to data (if read points to input buffer, if write points data to send)
}stMbusRequest;

typedef struct
{
	uint16_t cmd;
	uint16_t param[3];
}stModBusCmd;


// Function prototypes


void SendSerial(uint8_t*,uint8_t,uint8_t);
void Configure_COM_Dflt();
void Configure_COM(stCOM_Cfg *);
uint16_t CRC16add(uint8_t* , uint16_t, uint16_t);
uint16_t CRC16(uint8_t *, uint16_t);								// Mod bus CRC16 computing
uint16_t fReadHldRegRSP(uint8_t *,uint16_t ,uint16_t, stMbusReadRSP *, uint8_t );   // Function Read Holding register response computing
uint16_t fWriteHldRegRSP(uint8_t* ,uint8_t *, uint16_t, stMbusWriteRSP *, uint8_t);
uint32_t Compute_MBUSRequest(uint8_t *, uint8_t, uint8_t);                        // Mod bus master request computing
uint16_t fMaster_RW_Registers(stMbusRequest *, uint8_t);        // Process Mod bus Request Read or Write Holding Register
uint16_t swap16(uint16_t);									// void fConfigUsart(Usart *,stCOM_Cfg *);							// Make USART initialization with stCOM_Cfg parameters
void fSetCOM_Param(stCOM_Cfg *);
void fLoadCOM_Param(stCOM_Cfg *);
void fLoadCOM_Default(void);								// Load default parameters to stCOM_Cfg
stCOM_Cfg* fGet_COM_Param(void);
void fComInit(void);
uint32_t fGetLastCOM_tm(uint32_t );		// Get last communication time in tick count
uint32_t fGetErrCOM_Cnt(uint32_t );		// Get number of comunication errrors
void fClr_LastCOM(uint32_t port);
uint32_t fMbus_Engine(void);
uint32_t USART_SEND(uint8_t* ,uint8_t, uint8_t);
void USART_Init(void);
uint32_t USART_StarFrame(uint8_t *);
void TestUart(void);
#endif /* MODBUSRTU_H_ */
