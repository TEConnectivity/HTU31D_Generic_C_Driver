/**
* Copyright (c) 2020 TE Connectivity. ALL RIGHTS RESERVED.
* Use of the Software is governed by the terms of the license agreement, if any,
* which accompanies or is included with the Software ("License Agreement").
***********************************************************************************************************************
* @file   htu31d.h
* @brief  htu31d Temperature & Humidity sensor driver header file
*/

#ifndef HTU31D_H
#define HTU31D_H

/**
* @defgroup GROUP_HTU31D Htu31d driver subcomponent
* This subcomponent manage HTU31d sensor
*/

/****************************************************
* INCLUDES
****************************************************/
/****************************************************
* PUBLIC CONFIGURATION DEFINES
****************************************************/

/** HTU31D I2C address value when I2C_ADD pin is routed to GND */
#define HTU31D_ADDR_1   ((uint16_t)0x40U)
/** HTU31D I2C address value when I2C_ADD pin is routed to VDD */
#define HTU31D_ADDR_2   ((uint16_t)0x41U)
/** HTU31D I2C address selection */
#define HTU31D_ADDRESS  (HTU31D_ADDR_1)

/****************************************************
* PUBLIC DEFINES
*****************************************************/
/****************************************************
* PUBLIC TYPEDEFS
*****************************************************/

/** Enumeration type definition for command execution status */
typedef enum
{
	eHTU31D_COMMSTATUS_OK,					/**< No communication error  */
	eHTU31D_COMMSTATUS_NO_RESPONSE,			/**< Htu31d not respond      */
	eHTU31D_COMMSTATUS_COMMAND_REJECTED,	/**< Htu31d command rejected */
	eHTU31D_COMMSTATUS_READ_ERROR,			/**< Htu31d read error       */
	eHTU31D_COMMSTATUS_DATA_CRC_ERROR		/**< htu31d data crc error   */
}eHtu31d_CommStatus_t;

/** Structure type definition for Htu31d internal diagnostics */
typedef struct
{
	bool_t  bIsNvmInError;				/**< Htu31d internal nvm error                                           */
	bool_t  bIsHumidityUnderOverRun;	/**< Htu31d Humidity sensor under or over run (internal adc clampled)    */
	bool_t  bIsHumidityHighError;		/**< Htu31d Humidity sensor error High (> 120% rh)                       */
	bool_t  bIsHumidityLowError;		/**< Htu31d Humidity sensor error Low (< -10% rh)                        */
	bool_t  bIsTemperatureUnderOverRun;	/**< Htu31d Temperature sensor under or over run (internal adc clampled) */
	bool_t  bIsTemperatureHighError;	/**< Htu31d Temperature sensor error High (> 150 degree celsius)         */
	bool_t  bIsTemperatureLowError;		/**< Htu31d Temperature sensor error Low (< -50 degree celsius)          */
	bool_t  bIsHeasterOn;				/**< Htu31d heater enabled state                                         */
}xHtu31d_Diagnostics_t;

/** Enumeration type definition for measurement configuration of  Humidity and Temperature */
typedef enum
{
	eHTU31D_CFG_RH_0_TEMPERATURE_0,	/**< Oversampling configuration: Humidity: 1.11ms; Temperature: 1.57ms  */
	eHTU31D_CFG_RH_0_TEMPERATURE_1,	/**< Oversampling configuration: Humidity: 1.11ms; Temperature: 3.06ms  */
	eHTU31D_CFG_RH_0_TEMPERATURE_2,	/**< Oversampling configuration: Humidity: 1.11ms; Temperature: 6.03ms  */
	eHTU31D_CFG_RH_0_TEMPERATURE_3,	/**< Oversampling configuration: Humidity: 1.11ms; Temperature: 11.98ms */
	eHTU31D_CFG_RH_1_TEMPERATURE_0,	/**< Oversampling configuration: Humidity: 2.14ms; Temperature: 1.57ms  */
	eHTU31D_CFG_RH_1_TEMPERATURE_1,	/**< Oversampling configuration: Humidity: 2.14ms; Temperature: 3.06ms  */
	eHTU31D_CFG_RH_1_TEMPERATURE_2,	/**< Oversampling configuration: Humidity: 2.14ms; Temperature: 6.03ms  */
	eHTU31D_CFG_RH_1_TEMPERATURE_3,	/**< Oversampling configuration: Humidity: 2.14ms; Temperature: 11.98ms */
	eHTU31D_CFG_RH_2_TEMPERATURE_0,	/**< Oversampling configuration: Humidity: 4.21ms; Temperature: 1.57ms  */
	eHTU31D_CFG_RH_2_TEMPERATURE_1,	/**< Oversampling configuration: Humidity: 4.21ms; Temperature: 3.06ms  */
	eHTU31D_CFG_RH_2_TEMPERATURE_2,	/**< Oversampling configuration: Humidity: 4.21ms; Temperature: 6.03ms  */
	eHTU31D_CFG_RH_2_TEMPERATURE_3,	/**< Oversampling configuration: Humidity: 4.21ms; Temperature: 11.98ms */
	eHTU31D_CFG_RH_3_TEMPERATURE_0,	/**< Oversampling configuration: Humidity: 8.34ms; Temperature: 1.57ms  */
	eHTU31D_CFG_RH_3_TEMPERATURE_1,	/**< Oversampling configuration: Humidity: 8.34ms; Temperature: 3.06ms  */
	eHTU31D_CFG_RH_3_TEMPERATURE_2,	/**< Oversampling configuration: Humidity: 8.34ms; Temperature: 6.03ms  */
	eHTU31D_CFG_RH_3_TEMPERATURE_3	/**< Oversampling configuration: Humidity: 8.34ms; Temperature: 11.98ms */
}eHtu31d_MeasureConfiguration_t;

/** Type definition for Htu31d Serial number */
typedef uint32_t Htu31d_SerialNumber_t;

/****************************************************
* PUBLIC FUNCTION PROTOTYPES
*****************************************************/
void vHtu31d_init(void);
eHtu31d_CommStatus_t eHtu31d_Reset(void);
eHtu31d_CommStatus_t eHtu31d_EnableHeater(void);
eHtu31d_CommStatus_t eHtu31d_DisableHeater(void);
eHtu31d_CommStatus_t eHtu31d_LaunchMeasurements(const eHtu31d_MeasureConfiguration_t tMeasureConfiguration);
eHtu31d_CommStatus_t eHtu31d_ReadSerialNumber(Htu31d_SerialNumber_t * const ptSerialNumber);
eHtu31d_CommStatus_t eHtu31d_ReadTemperatureAndHumidity(float32_t * const pf32Temperature, float32_t * const pf32Humidity);
eHtu31d_CommStatus_t eHtu31d_ReadTemperature(float32_t * const pf32Temperature);
eHtu31d_CommStatus_t eHtu31d_ReadHumidity(float32_t * const pf32Humidity);
eHtu31d_CommStatus_t eHtu31d_ReadDiagnostics(xHtu31d_Diagnostics_t * const ptDiagnostics);
float32_t f32Htu31d_ComputeDewPoint(const float32_t f32Temperature, const float32_t f32Humidity);

/****************************************************
* Unit Test Wrappers
****************************************************/
#ifdef UNIT_TEST
bool_t UT_bIsDataCrcCorrect(uint8_t const au8Buffer[], const uint8_t u8BufferSize);
eHtu31d_CommStatus_t UT_eWriteCommand(const uint8_t u8Command, const bool_t bIsReadCommand);
eHtu31d_CommStatus_t UT_eReadCommand(uint8_t au8Buffer[], const uint8_t u8Size);
float32_t UT_f32ConvertTemperatureAdcToDegree(const uint16_t u16TemperatureAdc);
float32_t UT_f32ConvertRhAdcToPercent(const uint16_t u16RhAdc);
#endif

#endif /* HTU31D_H */
