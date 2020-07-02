/**
* Copyright (c) 2020 TE Connectivity. ALL RIGHTS RESERVED.
* Use of the Software is governed by the terms of the license agreement, if any,
* which accompanies or is included with the Software ("License Agreement").
***********************************************************************************************************************
* @file   htu31d.c
* @brief  htu31d Temperature & Humidity sensor driver file
* For details on programming, refer to htu31d datasheet:
* www.te.com/usa-en/product-CAT-HSC0007.html
*/

/** @addtogroup GROUP_HTU31D
* @{
*/

#ifdef __cplusplus
extern "C" {
#endif

/****************************************************
* INCLUDES
****************************************************/

/* 
* The header "basic_types.h" contain basic types definitions.
* According to project integration, this header can be used or replaced.
* typedef int               bool_t;      Boolean
* typedef signed char       int8_t;      8-Bit Signed
* typedef unsigned char     uint8_t;     8-Bit Unsigned
* typedef signed short      int16_t;    16-Bit Signed
* typedef unsigned short    uint16_t;   16-Bit Unsigned
* typedef signed long       int32_t;    32-Bit Signed
* typedef unsigned long     uint32_t;   32-Bit Unsigned
* typedef float             float32_t;  32-Bit Float
* #define B_TRUE            (1U) Boolean true definition
* #define B_FALSE           (0U) Boolean false definition
*/
#include "basic_types.h"

/*
* The header "i2c.h" has to be implemented for your own platform to
* conform the following protocol :
*
* Enumeration type definition for i2c returned status
* typedef enum
* {
*     eI2C_STATUS_OK              = 0x00, Exchange complete without error
*     eI2C_STATUS_ERR_OVERFLOW    = 0x01, i2c driver full
*     eI2C_STATUS_ERR_ADDRESS     = 0x02, device not respond on address
*     eI2C_STATUS_ERR_TRANSFER    = 0x03  transfer error. Unexpected NACK
* }eI2c_StatusCode_t;
*
* typedef struct
* {
*     uint16_t u16Address;    Address to slave device
*     uint16_t u16DataLength; Length of data array
*     uint8_t *pu8Data;       Data array containing all data to be transferred
* }xI2c_MasterPacket_t;
*
* Initialise Driver and sub layer
* void i2c_master_init(void);
* 
* Read data from i2c device
* eI2c_StatusCode_t tI2c_MasterReadPacket(xI2c_MasterPacket_t *const ptPacket);
* 
* write data to i2c device
* eI2c_StatusCode_t tI2c_MasterWritePacket(xI2c_MasterPacket_t *const ptPacket);
* 
* Write data to i2C device without closing link with device. Mainly used as first step of a i2c read process
* eI2c_StatusCode_t tI2c_MasterWritePacketNoStop(xI2c_MasterPacket_t *const ptPacket);
*/
#include "i2c.h"

/** Htu31d driver header */
#include "htu31d.h"

/** math.h is include to use "powf" and "log10" during Dew Point computation */
#include <math.h>

/****************************************************
* DEFINES
****************************************************/

/** HTU31D Command: Perform a reset of Htu31d */
#define U8_COMMAND_RESET                ((uint8_t) 0x1EU)
/** HTU31D Command: Activate Htu31d heater */
#define U8_COMMAND_HEATER_ON            ((uint8_t) 0x04U)
/** HTU31D Command: Deactivate Htu31d heater */
#define U8_COMMAND_HEATER_OFF           ((uint8_t) 0x02U)
/** HTU31D Command: Request Temperature and Humidity measurement with specified oversampling configuration */
#define U8_COMMAND_MEAS_RH_0_TEMP_0     ((uint8_t) 0x40U)
/** HTU31D Command: Request Temperature and Humidity measurement with specified oversampling configuration */
#define U8_COMMAND_MEAS_RH_0_TEMP_1     ((uint8_t) 0x42U)
/** HTU31D Command: Request Temperature and Humidity measurement with specified oversampling configuration */
#define U8_COMMAND_MEAS_RH_0_TEMP_2     ((uint8_t) 0x44U)
/** HTU31D Command: Request Temperature and Humidity measurement with specified oversampling configuration */
#define U8_COMMAND_MEAS_RH_0_TEMP_3     ((uint8_t) 0x46U)
/** HTU31D Command: Request Temperature and Humidity measurement with specified oversampling configuration */
#define U8_COMMAND_MEAS_RH_1_TEMP_0     ((uint8_t) 0x48U)
/** HTU31D Command: Request Temperature and Humidity measurement with specified oversampling configuration */
#define U8_COMMAND_MEAS_RH_1_TEMP_1     ((uint8_t) 0x4AU)
/** HTU31D Command: Request Temperature and Humidity measurement with specified oversampling configuration */
#define U8_COMMAND_MEAS_RH_1_TEMP_2     ((uint8_t) 0x4CU)
/** HTU31D Command: Request Temperature and Humidity measurement with specified oversampling configuration */
#define U8_COMMAND_MEAS_RH_1_TEMP_3     ((uint8_t) 0x4EU)
/** HTU31D Command: Request Temperature and Humidity measurement with specified oversampling configuration */
#define U8_COMMAND_MEAS_RH_2_TEMP_0     ((uint8_t) 0x50U)
/** HTU31D Command: Request Temperature and Humidity measurement with specified oversampling configuration */
#define U8_COMMAND_MEAS_RH_2_TEMP_1     ((uint8_t) 0x52U)
/** HTU31D Command: Request Temperature and Humidity measurement with specified oversampling configuration */
#define U8_COMMAND_MEAS_RH_2_TEMP_2     ((uint8_t) 0x54U)
/** HTU31D Command: Request Temperature and Humidity measurement with specified oversampling configuration */
#define U8_COMMAND_MEAS_RH_2_TEMP_3     ((uint8_t) 0x56U)
/** HTU31D Command: Request Temperature and Humidity measurement with specified oversampling configuration */
#define U8_COMMAND_MEAS_RH_3_TEMP_0     ((uint8_t) 0x58U)
/** HTU31D Command: Request Temperature and Humidity measurement with specified oversampling configuration */
#define U8_COMMAND_MEAS_RH_3_TEMP_1     ((uint8_t) 0x5AU)
/** HTU31D Command: Request Temperature and Humidity measurement with specified oversampling configuration */
#define U8_COMMAND_MEAS_RH_3_TEMP_2     ((uint8_t) 0x5CU)
/** HTU31D Command: Request Temperature and Humidity measurement with specified oversampling configuration */
#define U8_COMMAND_MEAS_RH_3_TEMP_3     ((uint8_t) 0x5EU)
/** HTU31D Command: Read Temperature and Humidity measurement result*/
#define U8_COMMAND_READ_ADC_T_RH        ((uint8_t) 0x00U)
/** HTU31D Command: Read Humidity measurement result only */
#define U8_COMMAND_READ_ADC_RH          ((uint8_t) 0x10U)
/** HTU31D Command: Read Htu31d internal diagnostics */
#define U8_COMMAND_READ_DIAGNOSTIC      ((uint8_t) 0x08U)
/** HTU31D Command: Read Htu31d Serial Number */
#define U8_COMMAND_READ_SN              ((uint8_t) 0x0AU)

/** I2C protocol data size: Read Serial Number request 3 bytes + 1 crc byte */
#define U8_READ_SN_I2C_SIZE				((uint8_t) 0x04U)
/** I2C protocol data size: Read Temperature and Humidity is 2 bytes for temperature + 1 crc byte + 2 bytes for humidity + 1 crc byte */
#define U8_READ_T_RH_I2C_SIZE			((uint8_t) 0x06U)
/** I2C protocol data size: Read Temperature request 2 bytes + 1 crc byte */
#define U8_READ_T_I2C_SIZE				((uint8_t) 0x03U)
/** I2C protocol data size: Read Humidity request 2 bytes + 1 crc byte */
#define U8_READ_RH_I2C_SIZE				((uint8_t) 0x03U)
/** I2C protocol data size: Read Diagnostics request 1 bytes + 1 crc byte */
#define U8_READ_DIAG_I2C_SIZE			((uint8_t) 0x02U)
/** I2C protocol data size: Write Command 1 byte no crc byte */
#define U16_WRITE_I2C_SIZE              ((uint16_t)0x01U)

/** HTU31D diagnostic mask. To be used on shift result with eHtu31d_DiagnosticsBitsIndex_t enumeration */
#define U16_DIAG_FLAG_MSK				((uint16_t) 0x01U)

/** Temperature computation coefficients */
#define F32_TEMPERATURE_COEFF_MUL		((float32_t)165.00f)
#define F32_TEMPERATURE_COEFF_ADD		((float32_t)-40.00f)
#define F32_TEMPERATURE_COEFF_DIV		((float32_t)((1UL << 16) - 1UL))

/**  Humidity computation coefficients */
#define F32_HUMIDITY_COEFF_MUL			((float32_t)(100.0f))
#define F32_HUMIDITY_COEFF_ADD			((float32_t)(0.0f))
#define F32_HUMIDITY_COEFF_DIV			((float32_t)((1UL << 16) - 1UL))

/** Dew Point computation constants */
#define F32_DEW_POINT_CONSTANT_A		((float32_t)(8.1332f))
#define F32_DEW_POINT_CONSTANT_B		((float32_t)(1762.39f))
#define F32_DEW_POINT_CONSTANT_C		((float32_t)(235.66f))

/** Crc polynomial form: x^8 + x^5 + x^4 + 1 */
#define U32_CRC_3_BYTES_POLYNOM         ((uint32_t)0x98800000UL)
/** Crc computation MSB pattern */
#define U32_CRC_3_BYTES_MSB             ((uint32_t)0x80000000UL)
/** Crc computation MSB threshold value */
#define U32_CRC_MSB_THRESHOLD           ((uint32_t)0x00000080UL)
/** Crc computation data mask */
#define U32_CRC_3_BYTES_MASK            ((uint32_t)0xFF800000UL)
/** Mask to be applied on updated result to extract computed CRC */
#define U32_CRC_RESULT_MASK             ((uint32_t)0x000000FFUL)

/** Bit manipulation symbols: 8 Bit Shifting value for storage handling */
#define U8_BYTE_SHIFT                   ((uint8_t)  8U)
/** Bit manipulation symbols: 16 Bit Shifting value for storage handling */
#define U8_SHORT_SHIFT                  ((uint8_t) 16U)

/****************************************************
* TYPEDEFS
****************************************************/
/** HTU31D diagnostics bit index */
typedef enum
{
    eDIAG_HEATER_ON_IDX = 0U,      /**< Index 0: Heater on index */
    eDIAG_TEMP_LOW_ERR_IDX,        /**< Index 1: Temperature low error index */
    eDIAG_TEMP_HIGH_ERR_IDX,       /**< Index 2: Temperature high error index */
    eDIAG_TEMP_UND_OVR_ERR_IDX,    /**< Index 3: Temperature under/overrun error index */
    eDIAG_HUM_LOW_ERR_IDX,         /**< Index 4: Humidity low error index */
    eDIAG_HUM_HIGH_ERR_IDX,        /**< Index 5: Humidity high error index */
    eDIAG_HUM_UND_OVR_ERR_IDX,     /**< Index 6: Humidity under/overrun error index */
    eDIAG_NVM_ERR_IDX              /**< Index 7: NVM error index */    
}eHtu31d_DiagnosticsBitsIndex_t;

/****************************************************
* VARIABLES
****************************************************/
/****************************************************
* CONSTANTS
****************************************************/
/****************************************************
* PROTOTYPES
****************************************************/
static bool_t				bIsDataCrcCorrect(uint8_t const au8Buffer[], const uint8_t u8BufferSize);
static eHtu31d_CommStatus_t eWriteCommand(const uint8_t u8Command, const bool_t bIsReadCommand);
static eHtu31d_CommStatus_t eReadCommand(uint8_t au8Buffer[], const uint8_t u8Size);
static float32_t			f32ConvertTemperatureAdcToDegree(const uint16_t u16TemperatureAdc);
static float32_t			f32ConvertRhAdcToPercent(const uint16_t u16RhAdc);

/****************************************************
* PRIVATE FUNCTIONS
****************************************************/

/**
*@brief         Perform a crc check on a buffer containing [data + crc]
*@param[in]     au8Buffer[]: Buffer to check
*@param[in,out] u8BufferSize: size of the data + crc
*@warning       function available for One, Two and Tree byte data size.
*/
static bool_t bIsDataCrcCorrect(uint8_t const au8Buffer[], const uint8_t u8BufferSize)
{
    bool_t   bStatus       = B_FALSE;
    uint8_t  u8ExpectedCrc = (uint8_t)0x0U;
    uint32_t u32Result     = (uint32_t)0x0U;
    uint32_t u32Polynom    = U32_CRC_3_BYTES_POLYNOM; 
    uint32_t u32Msb        = U32_CRC_3_BYTES_MSB;
    uint32_t u32Mask       = U32_CRC_3_BYTES_MASK;
    
    if (u8BufferSize == U8_READ_SN_I2C_SIZE)
    {
        u8ExpectedCrc = au8Buffer[U8_READ_SN_I2C_SIZE - 1U];
        u32Result  = (uint32_t)au8Buffer[0];
        u32Result <<= U8_BYTE_SHIFT;
        u32Result |= (uint32_t)au8Buffer[1];
        u32Result <<= U8_BYTE_SHIFT;
        u32Result |= (uint32_t)au8Buffer[2];
        u32Result <<= U8_BYTE_SHIFT;
        //au8Buffer[3] (U8_READ_SN_I2C_SIZE - 1U) contain expected crc. u32Result LSB (byte) shall cleared
    }
    // In order to prevent duplicated code, test on U8_READ_T_I2C_SIZE is not performed.
    // U8_READ_RH_I2C_SIZE is equal to U8_READ_T_I2C_SIZE, so computations are identical.
    else if (u8BufferSize == U8_READ_RH_I2C_SIZE)
    {
        u8ExpectedCrc = au8Buffer[U8_READ_RH_I2C_SIZE - 1U];
        u32Result  = (uint32_t)au8Buffer[0] << U8_SHORT_SHIFT;
        u32Result |= (uint32_t)au8Buffer[1] <<  U8_BYTE_SHIFT;
        //au8Buffer[2] (U8_READ_RH_I2C_SIZE - 1U) contain expected crc. u32Result LSB (byte) shall cleared
    }
    else if (u8BufferSize == U8_READ_DIAG_I2C_SIZE)
    {
        u8ExpectedCrc = au8Buffer[U8_READ_DIAG_I2C_SIZE - 1U];
        u32Result  = (uint32_t)au8Buffer[0] << U8_BYTE_SHIFT;
        //au8Buffer[1] (U8_READ_DIAG_I2C_SIZE - 1U) contain expected crc. u32Result LSB (byte) shall cleared
    }
    else
    {
    	// unexpected size: generate crc error
        u8ExpectedCrc = 0U;
        u32Result = U32_CRC_RESULT_MASK;
        u32Msb = U32_CRC_MSB_THRESHOLD;
    }
    
    while(u32Msb != U32_CRC_MSB_THRESHOLD)
    {
        // Check if msb of current value is 1 and apply XOR mask
        if((u32Result & u32Msb) != 0UL)
        {
            u32Result = ((u32Result ^ u32Polynom) & u32Mask) | (u32Result & ~u32Mask);
        }
        
        // Shift by one
        u32Msb     >>= 1UL;
        u32Mask    >>= 1UL;
        u32Polynom >>= 1UL;
    }
    
    
    if(u8ExpectedCrc == (uint8_t)(U32_CRC_RESULT_MASK & u32Result))
    {
        bStatus = B_TRUE;
    }
    else
    {
        bStatus = B_FALSE;
    }
    
return (bStatus);
}

/**
*@brief			Perform a Write command to Htu31d
*@param[in]		u8Command: Command to be transmitted
*@param[in]		bIsReadCommand: Is the Command is a read command
*@return		tCommStatus: execution status of the command
*				- eHTU31D_COMMSTATUS_OK: command executed as expected
*				- eHTU31D_COMMSTATUS_NO_RESPONSE: No response from Htu31d
*				- eHTU31D_COMMSTATUS_COMMAND_REJECTED: Command Rejected
*/
static eHtu31d_CommStatus_t eWriteCommand(const uint8_t u8Command, const bool_t bIsReadCommand)
{
	eHtu31d_CommStatus_t	tCommStatus;
	eI2c_StatusCode_t		eI2cStatus;
	uint8_t					u8Data;
	xI2c_MasterPacket_t     tPacket;

	tPacket.u16Address		= HTU31D_ADDRESS;
	tPacket.u16DataLength	= U16_WRITE_I2C_SIZE;
	tPacket.pu8Data		    = &u8Data;
	
    // Get the command
    u8Data = u8Command;

	// send the command
	if(B_FALSE == bIsReadCommand)
	{
		eI2cStatus = tI2c_MasterWritePacket(&tPacket);
	}
	else
	{
		eI2cStatus = tI2c_MasterWritePacketNoStop(&tPacket);
	}

	if (eI2C_STATUS_ERR_OVERFLOW == eI2cStatus)
	{
		tCommStatus =  eHTU31D_COMMSTATUS_NO_RESPONSE;
	}
	else if (eI2C_STATUS_ERR_ADDRESS == eI2cStatus)
	{
		tCommStatus =  eHTU31D_COMMSTATUS_NO_RESPONSE;
	}
	else if (eI2C_STATUS_ERR_TRANSFER == eI2cStatus)
	{
		tCommStatus = eHTU31D_COMMSTATUS_COMMAND_REJECTED;
	}
	else
	{
		tCommStatus = eHTU31D_COMMSTATUS_OK;
	}

	return (tCommStatus);
}

/**
*@brief			Perform a Read command to Htu31d
*@param[in]		au8Buffer[]: Buffer to be filled
*@param[in]		u8Size: Size of the buffer
*@return		tCommStatus: execution status of the command
*				- eHTU31D_COMMSTATUS_OK: command executed as expected
*				- eHTU31D_COMMSTATUS_NO_RESPONSE: No response from Htu31d
*				- eHTU31D_COMMSTATUS_READ_ERROR: Read Error
*/
static eHtu31d_CommStatus_t eReadCommand(uint8_t au8Buffer[], const uint8_t u8Size)
{
	eHtu31d_CommStatus_t	tCommStatus;
	eI2c_StatusCode_t		eI2cStatus;
	xI2c_MasterPacket_t     tPacket;
	
	tPacket.u16Address		= HTU31D_ADDRESS;
	tPacket.u16DataLength	= (uint16_t) u8Size;
	tPacket.pu8Data		    = au8Buffer;

	// read data
	eI2cStatus = tI2c_MasterReadPacket(&tPacket);

	if (eI2C_STATUS_ERR_OVERFLOW == eI2cStatus)
	{
		tCommStatus =  eHTU31D_COMMSTATUS_NO_RESPONSE;
	}
	else if (eI2C_STATUS_ERR_ADDRESS == eI2cStatus)
	{
		tCommStatus =  eHTU31D_COMMSTATUS_NO_RESPONSE;
	}
	else if (eI2C_STATUS_ERR_TRANSFER == eI2cStatus)
	{
		tCommStatus = eHTU31D_COMMSTATUS_READ_ERROR;
	}
	else
	{
		tCommStatus = eHTU31D_COMMSTATUS_OK;
	}

	return (tCommStatus);
}

/**
*@brief			Convert 16-bits ADC temperature from Htu31d to floating point degree celsius
*@param[in]		u16TemperatureAdc: Raw ADC temperature from Htu31d
*@return		f32Temperature: Temperature in degree celsius
*/
static float32_t f32ConvertTemperatureAdcToDegree(const uint16_t u16TemperatureAdc)
{
	float32_t f32Temperature;

	//Temperature = Offset + Coef * AdcValue / AdcResolution
	f32Temperature  = (float32_t)u16TemperatureAdc;
	f32Temperature *= F32_TEMPERATURE_COEFF_MUL;
	f32Temperature /= F32_TEMPERATURE_COEFF_DIV;
	f32Temperature += F32_TEMPERATURE_COEFF_ADD;

	return (f32Temperature);
}

/**
*@brief			Convert 16-bits ADC Rh from Htu31d to floating point percent
*@param[in]		u16RhAdc: Raw ADC Rh from Htu31d
*@return		f32Humidity: Rh in percent
*/
static float32_t f32ConvertRhAdcToPercent(const uint16_t u16RhAdc)
{
	float32_t f32Humidity;

	//Rh = Offset + Coef * AdcValue / AdcResolution
	f32Humidity  = (float32_t)u16RhAdc;
	f32Humidity *= F32_HUMIDITY_COEFF_MUL;
	f32Humidity /= F32_HUMIDITY_COEFF_DIV;
	//lint -e{835} Suppress Zero given as right argument
	f32Humidity += F32_HUMIDITY_COEFF_ADD;
	
	return (f32Humidity);
}

/****************************************************
* PUBLIC FUNCTIONS
****************************************************/

/**
*@brief  Initialise Htu31d driver
*@brief  In the current implementation, there is nothing to do.
*@brief  At PowerUp, HTU31D need at least 20ms to handle internal nvm check.
*@brief  User can implement here a procedure to handle power lines and timings.
*/
void vHtu31d_init(void)
{
    //power line can be managed here.
}


/**
*@brief    Request a reset sequence of Htu31d
*@return   tCommStatus: execution status of the command
*				- eHTU31D_COMMSTATUS_OK: command executed as expected
*				- eHTU31D_COMMSTATUS_NO_RESPONSE: No response from Htu31d
*				- eHTU31D_COMMSTATUS_COMMAND_REJECTED: Command Rejected
*@warning  When requested, all commands will be rejected for at least 15ms
*/
eHtu31d_CommStatus_t eHtu31d_Reset(void)
{
	eHtu31d_CommStatus_t	tCommStatus;
	
	// Send the command
	tCommStatus = eWriteCommand(U8_COMMAND_RESET, B_FALSE);

	return (tCommStatus);
}

/**
*@brief    Enable Htu31d heater
*@return   tCommStatus: execution status of the command
*				- eHTU31D_COMMSTATUS_OK: command executed as expected
*				- eHTU31D_COMMSTATUS_NO_RESPONSE: No response from Htu31d
*				- eHTU31D_COMMSTATUS_COMMAND_REJECTED: Command Rejected
*/
eHtu31d_CommStatus_t eHtu31d_EnableHeater(void)
{
	eHtu31d_CommStatus_t	tCommStatus;

	// Send the command
	tCommStatus = eWriteCommand(U8_COMMAND_HEATER_ON, B_FALSE);

	return (tCommStatus);
}

/**
*@brief    Disable Htu31d heater
*@return   tCommStatus: execution status of the command
*				- eHTU31D_COMMSTATUS_OK: command executed as expected
*				- eHTU31D_COMMSTATUS_NO_RESPONSE: No response from Htu31d
*				- eHTU31D_COMMSTATUS_COMMAND_REJECTED: Command Rejected
*/
eHtu31d_CommStatus_t eHtu31d_DisableHeater(void)
{
	eHtu31d_CommStatus_t	tCommStatus;

	// Send the command
	tCommStatus = eWriteCommand(U8_COMMAND_HEATER_OFF, B_FALSE);

	return (tCommStatus);
}

/**
*@brief      Launch measurement of temperature and relative humidity according to configuration parameter
*@param[in]  tMeasureConfiguration: Measurement configuration
*@return     tCommStatus: execution status of the command
*				- eHTU31D_COMMSTATUS_OK: command executed as expected
*				- eHTU31D_COMMSTATUS_NO_RESPONSE: No response from Htu31d
*				- eHTU31D_COMMSTATUS_COMMAND_REJECTED: Command Rejected
*/
eHtu31d_CommStatus_t eHtu31d_LaunchMeasurements(const eHtu31d_MeasureConfiguration_t tMeasureConfiguration)
{
	eHtu31d_CommStatus_t	tCommStatus;
	uint8_t					u8Command;

    //Command construction
    u8Command = (uint8_t)tMeasureConfiguration;
	
    if (u8Command <= (uint8_t)eHTU31D_CFG_RH_3_TEMPERATURE_3)
	{
         // According to datasheet, command conversion byte is construct as bellow
         // bit index: 7 | 6 | 5 |  4  |  3  |  2 |  1 | 0
         // bit value: 0 | 1 | 0 | RH1 | RH0 | T1 | T0 | 0
         // Using an enumerate from 0 to 15, with shift by one and set the sixth bit.
         u8Command <<= 1U;
         u8Command += U8_COMMAND_MEAS_RH_0_TEMP_0;

         // Send the command
         tCommStatus = eWriteCommand(u8Command, B_FALSE);
	}
	else
	{
	    // set status to rejected 
	    tCommStatus = eHTU31D_COMMSTATUS_COMMAND_REJECTED;
	}

	return (tCommStatus);
}


/**
*@brief          Read Htu31d serial number
*@param[in,out]  ptSerialNumber: Serial Number container
*@return         tCommStatus: execution status of the command
*					- eHTU31D_COMMSTATUS_OK: command executed as expected
*					- eHTU31D_COMMSTATUS_NO_RESPONSE: No response from Htu31d
*					- eHTU31D_COMMSTATUS_COMMAND_REJECTED: Command Rejected
*					- eHTU31D_COMMSTATUS_READ_ERROR: Read Error
*					- eHTU31D_COMMSTATUS_DATA_CRC_ERROR: Serial number crc check is not correct
*/
eHtu31d_CommStatus_t eHtu31d_ReadSerialNumber(Htu31d_SerialNumber_t * const ptSerialNumber)
{
	eHtu31d_CommStatus_t	tCommStatus;
	bool_t					bIsDataCrcOk;
	uint8_t					au8Buffer[U8_READ_SN_I2C_SIZE] = {0U};

	// clear Serial Number container
	*ptSerialNumber = 0U;

	// Send the command
	tCommStatus = eWriteCommand(U8_COMMAND_READ_SN, B_TRUE);

	if (eHTU31D_COMMSTATUS_OK == tCommStatus)
	{
		tCommStatus = eReadCommand(au8Buffer, U8_READ_SN_I2C_SIZE);
	}

	if (eHTU31D_COMMSTATUS_OK == tCommStatus)
	{
		//Check data crc
		bIsDataCrcOk = bIsDataCrcCorrect(au8Buffer, U8_READ_SN_I2C_SIZE);

		if (B_TRUE == bIsDataCrcOk)
		{
			// copy Serial Number data
			ptSerialNumber[1] = au8Buffer[0];
			ptSerialNumber[2] = au8Buffer[1];
			ptSerialNumber[3] = au8Buffer[2];
		}
		else
		{
			//set error status and keep Serial Number cleared
			tCommStatus = eHTU31D_COMMSTATUS_DATA_CRC_ERROR;
		}
	}
	return (tCommStatus);
}

/**
*@brief          Read measured Temperature and measured  Humidity
*@param[in,out]  pf32Temperature: measured Temperature in degree celsius
*@param[in,out]  pf32Humidity: measured  Humidity, in percent
*@return         tCommStatus: execution status of the command
*					- eHTU31D_COMMSTATUS_OK: command executed as expected
*					- eHTU31D_COMMSTATUS_NO_RESPONSE: No response from Htu31d
*					- eHTU31D_COMMSTATUS_COMMAND_REJECTED: Command Rejected
*					- eHTU31D_COMMSTATUS_READ_ERROR: Read Error
*					- eHTU31D_COMMSTATUS_DATA_CRC_ERROR: measurement crc check is not correct
*/
eHtu31d_CommStatus_t eHtu31d_ReadTemperatureAndHumidity(float32_t * const pf32Temperature, float32_t * const pf32Humidity)
{
	eHtu31d_CommStatus_t	tCommStatus;
	bool_t					bIsDataCrcOk;
	uint16_t				u16AdcValue;
	uint8_t					au8Buffer[U8_READ_T_RH_I2C_SIZE] = {0U};

	// Send the command
	tCommStatus = eWriteCommand(U8_COMMAND_READ_ADC_T_RH, B_TRUE);

	if (eHTU31D_COMMSTATUS_OK == tCommStatus)
	{
		tCommStatus = eReadCommand(au8Buffer, U8_READ_T_RH_I2C_SIZE);
	}

	if (eHTU31D_COMMSTATUS_OK == tCommStatus)
	{
		//Check temperature data crc
		bIsDataCrcOk = bIsDataCrcCorrect(au8Buffer, U8_READ_T_I2C_SIZE);
		if (B_TRUE == bIsDataCrcOk)
		{
			// construct 16bits data
			u16AdcValue = au8Buffer[0];
			u16AdcValue <<= U8_BYTE_SHIFT;
			u16AdcValue |= au8Buffer[1];
			//Convert adc value to degree
			*pf32Temperature = f32ConvertTemperatureAdcToDegree(u16AdcValue);
		}
		else
		{
			//set error status
			tCommStatus = eHTU31D_COMMSTATUS_DATA_CRC_ERROR;
		}

		//Check rh data crc
		bIsDataCrcOk = bIsDataCrcCorrect(&au8Buffer[3], U8_READ_RH_I2C_SIZE);
		if (B_TRUE == bIsDataCrcOk)
		{
			// construct 16bits data
			u16AdcValue = au8Buffer[3];
			u16AdcValue <<= U8_BYTE_SHIFT;
			u16AdcValue |= au8Buffer[4];
			//Convert adc value to degree
			*pf32Humidity = f32ConvertRhAdcToPercent(u16AdcValue);
		}
		else
		{
			//set error status
			tCommStatus = eHTU31D_COMMSTATUS_DATA_CRC_ERROR;
		}
	}

	return (tCommStatus);
}

/**
*@brief          Read measured Temperature
*@param[in,out]  pf32Temperature: measured Temperature in degree celsius
*@return         tCommStatus: execution status of the command
*					- eHTU31D_COMMSTATUS_OK: command executed as expected
*					- eHTU31D_COMMSTATUS_NO_RESPONSE: No response from Htu31d
*					- eHTU31D_COMMSTATUS_COMMAND_REJECTED: Command Rejected
*					- eHTU31D_COMMSTATUS_READ_ERROR: Read Error
*					- eHTU31D_COMMSTATUS_DATA_CRC_ERROR: Temperature crc check is not correct
*/
eHtu31d_CommStatus_t eHtu31d_ReadTemperature(float32_t * const pf32Temperature)
{
	eHtu31d_CommStatus_t	tCommStatus;
	bool_t					bIsDataCrcOk;
	uint16_t				u16AdcValue;
	uint8_t					au8Buffer[U8_READ_T_I2C_SIZE] = {0U};

	// Send the command
	tCommStatus = eWriteCommand(U8_COMMAND_READ_ADC_T_RH, B_TRUE);

	if (eHTU31D_COMMSTATUS_OK == tCommStatus)
	{
		tCommStatus = eReadCommand(au8Buffer, U8_READ_T_I2C_SIZE);
	}

	if (eHTU31D_COMMSTATUS_OK == tCommStatus)
	{
		//Check temperature data crc
		bIsDataCrcOk = bIsDataCrcCorrect(au8Buffer, U8_READ_T_I2C_SIZE);

		if (B_TRUE == bIsDataCrcOk)
		{
			// construct 16bits data
			u16AdcValue = au8Buffer[0];
			u16AdcValue <<= U8_BYTE_SHIFT;
			u16AdcValue |= au8Buffer[1];
			//Convert adc value to degree
			*pf32Temperature = f32ConvertTemperatureAdcToDegree(u16AdcValue);
		}
		else
		{
			//set error status
			tCommStatus = eHTU31D_COMMSTATUS_DATA_CRC_ERROR;
		}
	}
	return (tCommStatus);
}

/**
*@brief          Read measured Humidity
*@param[in,out]  pf32Humidity: measured Humidity in percent
*@return         tCommStatus: execution status of the command
*					- eHTU31D_COMMSTATUS_OK: command executed as expected
*					- eHTU31D_COMMSTATUS_NO_RESPONSE: No response from Htu31d
*					- eHTU31D_COMMSTATUS_COMMAND_REJECTED: Command Rejected
*					- eHTU31D_COMMSTATUS_READ_ERROR: Read Error
*					- eHTU31D_COMMSTATUS_DATA_CRC_ERROR: Humidity crc check is not correct
*/
eHtu31d_CommStatus_t eHtu31d_ReadHumidity(float32_t * const pf32Humidity)
{
	eHtu31d_CommStatus_t	tCommStatus;
	bool_t					bIsDataCrcOk;
	uint16_t				u16AdcValue;
	uint8_t					au8Buffer[U8_READ_RH_I2C_SIZE] = {0U};

	// Send the command
	tCommStatus = eWriteCommand(U8_COMMAND_READ_ADC_RH, B_TRUE);

	if (eHTU31D_COMMSTATUS_OK == tCommStatus)
	{
		tCommStatus = eReadCommand(au8Buffer, U8_READ_RH_I2C_SIZE);
	}

	if (eHTU31D_COMMSTATUS_OK == tCommStatus)
	{
		//Check Humidity data crc
		bIsDataCrcOk = bIsDataCrcCorrect(au8Buffer, U8_READ_RH_I2C_SIZE);

		if (B_TRUE == bIsDataCrcOk)
		{
			// construct 16bits data
			u16AdcValue = au8Buffer[0];
			u16AdcValue <<= U8_BYTE_SHIFT;
			u16AdcValue |= au8Buffer[1];
			//Convert adc value to degree
			*pf32Humidity = f32ConvertRhAdcToPercent(u16AdcValue);
		}
		else
		{
			//set error status
			tCommStatus = eHTU31D_COMMSTATUS_DATA_CRC_ERROR;
		}
	}
	return (tCommStatus);
}

/**
*@brief          Read Htu31d diagnostics
*@param[in,out]  ptDiagnostics: Diagnostics container
*@return         tCommStatus: execution status of the command
*					- eHTU31D_COMMSTATUS_OK: command executed as expected
*					- eHTU31D_COMMSTATUS_NO_RESPONSE: No response from Htu31d
*					- eHTU31D_COMMSTATUS_COMMAND_REJECTED: Command Rejected
*					- eHTU31D_COMMSTATUS_READ_ERROR: Read Error
*					- eHTU31D_COMMSTATUS_DATA_CRC_ERROR: Serial number crc check is not correct
*/
eHtu31d_CommStatus_t eHtu31d_ReadDiagnostics(xHtu31d_Diagnostics_t * const ptDiagnostics)
{
	eHtu31d_CommStatus_t	tCommStatus;
	bool_t					bIsDataCrcOk;
	uint8_t					au8Buffer[U8_READ_DIAG_I2C_SIZE] = {0U};


	// Send the read command
	tCommStatus = eWriteCommand(U8_COMMAND_READ_DIAGNOSTIC, B_TRUE);

	// If no error, then read data
	if (eHTU31D_COMMSTATUS_OK == tCommStatus)
	{
		tCommStatus = eReadCommand(au8Buffer, U8_READ_DIAG_I2C_SIZE);
	}

	// If no error, then check data integrity
	if (eHTU31D_COMMSTATUS_OK == tCommStatus)
	{
		// Check diagnostic data crc
		bIsDataCrcOk = bIsDataCrcCorrect(au8Buffer, U8_READ_DIAG_I2C_SIZE);

		//if Crc is OK, then extract diagnostic data
		if (B_TRUE == bIsDataCrcOk)
		{
			// Extract diagnostics flags
		    ptDiagnostics->bIsNvmInError                = (bool_t)(U16_DIAG_FLAG_MSK & ((uint16_t)au8Buffer[0] >> (uint16_t)eDIAG_NVM_ERR_IDX));
			ptDiagnostics->bIsHumidityUnderOverRun      = (bool_t)(U16_DIAG_FLAG_MSK & ((uint16_t)au8Buffer[0] >> (uint16_t)eDIAG_HUM_UND_OVR_ERR_IDX));
			ptDiagnostics->bIsHumidityHighError         = (bool_t)(U16_DIAG_FLAG_MSK & ((uint16_t)au8Buffer[0] >> (uint16_t)eDIAG_HUM_HIGH_ERR_IDX));
			ptDiagnostics->bIsHumidityLowError          = (bool_t)(U16_DIAG_FLAG_MSK & ((uint16_t)au8Buffer[0] >> (uint16_t)eDIAG_HUM_LOW_ERR_IDX));
			ptDiagnostics->bIsTemperatureUnderOverRun   = (bool_t)(U16_DIAG_FLAG_MSK & ((uint16_t)au8Buffer[0] >> (uint16_t)eDIAG_TEMP_UND_OVR_ERR_IDX));
			ptDiagnostics->bIsTemperatureHighError      = (bool_t)(U16_DIAG_FLAG_MSK & ((uint16_t)au8Buffer[0] >> (uint16_t)eDIAG_TEMP_HIGH_ERR_IDX));
			ptDiagnostics->bIsTemperatureLowError       = (bool_t)(U16_DIAG_FLAG_MSK & ((uint16_t)au8Buffer[0] >> (uint16_t)eDIAG_TEMP_LOW_ERR_IDX));
			ptDiagnostics->bIsHeasterOn                 = (bool_t)(U16_DIAG_FLAG_MSK & ((uint16_t)au8Buffer[0] >> (uint16_t)eDIAG_HEATER_ON_IDX));
		}
		else
		{
			//set error status
			tCommStatus = eHTU31D_COMMSTATUS_DATA_CRC_ERROR;
		}
	}
	return (tCommStatus);
}

/**
*@brief      Compute DewPoint temperature according to temperature and relative humidity
*@param[in]  f32Temperature: Temperature in degree celsius
*@param[in]  f32Humidity:  Humidity in percent
*@return     f32DewPoint: DewPoint temperature in degree celsius
*@note       This function request "powf" and "log10" from math.h
*/
float32_t f32Htu31d_ComputeDewPoint(const float32_t f32Temperature, const float32_t f32Humidity)
{
	float32_t f32PartialPressure;
	float32_t f32DewPoint;

	// Compute partial pressure
	f32PartialPressure = powf(10.0f, F32_DEW_POINT_CONSTANT_A - (F32_DEW_POINT_CONSTANT_B / (f32Temperature + F32_DEW_POINT_CONSTANT_C)) );

	// Compute Dew Point
	f32DewPoint =  - ((F32_DEW_POINT_CONSTANT_B / (log10(f32Humidity * f32PartialPressure / 100.0f) - F32_DEW_POINT_CONSTANT_A)) + F32_DEW_POINT_CONSTANT_C);
	
	return (f32DewPoint);
}

/****************************************************
* Unit Test Wrappers
****************************************************/
#ifdef UNIT_TEST
/**
*@brief      Unit test wrapper of the function bIsDataCrcCorrect()
*/
bool_t UT_bIsDataCrcCorrect(uint8_t const au8Buffer[], const uint8_t u8BufferSize)
{
    return (bIsDataCrcCorrect(au8Buffer, u8BufferSize));
}

/**
*@brief      Unit test wrapper of the function eWriteCommand()
*/
eHtu31d_CommStatus_t UT_eWriteCommand(const uint8_t u8Command, const bool_t bIsReadCommand)
{
    return (eWriteCommand(u8Command, bIsReadCommand));
}

/**
*@brief      Unit test wrapper of the function eReadCommand()
*/
eHtu31d_CommStatus_t UT_eReadCommand(uint8_t au8Buffer[], const uint8_t u8Size)
{
    return (eReadCommand(au8Buffer, u8Size));
}

/**
*@brief      Unit test wrapper of the function f32ConvertTemperatureAdcToDegree()
*/
float32_t UT_f32ConvertTemperatureAdcToDegree(const uint16_t u16TemperatureAdc)
{
    return (f32ConvertTemperatureAdcToDegree(u16TemperatureAdc));
}

/**
*@brief      Unit test wrapper of the function f32ConvertRhAdcToPercent()
*/
float32_t UT_f32ConvertRhAdcToPercent(const uint16_t u16RhAdc)
{
    return (f32ConvertRhAdcToPercent(u16RhAdc));
}
#endif


#ifdef __cplusplus
}
#endif
/**@}*/
/* -------------------------------------------------------------------------  */
/* End of file                                              				 */
/* -------------------------------------------------------------------------  */
