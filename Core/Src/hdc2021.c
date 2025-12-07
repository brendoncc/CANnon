/**
 * @file hdc2021.c
 * @brief Source file for the HDC2021 Temperature/Humidity Sensor Driver.
 *
 * This file provides the implementation of functions to initialize,
 * configure, and read data from the HDC2021 sensor via I2C.
 */

/* Includes ------------------------------------------------------------------*/
#include "hdc2021.h"

/* Private Defines -----------------------------------------------------------*/
#define HDC2021_I2C_TIMEOUT_MS       10  // I2C communication timeout in milliseconds
#define HDC2021_MEAS_DELAY_MS        10  // Delay after triggering measurement (max conversion time is 8ms for 14-bit)

/* Private Functions Prototypes ----------------------------------------------*/
// These are internal helper functions, declared static to limit scope
/**
 * @brief Reads a single byte from a specified HDC2021 register.
 * @param hi2c Pointer to the I2C_HandleTypeDef structure for the I2C peripheral.
 * @param reg_addr Address of the register to read.
 * @param data Pointer to store the read data.
 * @return HAL_StatusTypeDef HAL_OK if read is successful, HAL_ERROR otherwise.
 */
HAL_StatusTypeDef HDC2021_ReadRegister(I2C_HandleTypeDef *hi2c,
		uint8_t reg_addr, uint8_t *data)
{
	// Use HAL_I2C_Mem_Read to read one byte from the specified register address.
	// The device address is HDC2021_I2C_ADDR, register address size is 1 byte (I2C_MEMADD_SIZE_8BIT).
	// Timeout is HDC2021_I2C_TIMEOUT_MS.
	if (HAL_I2C_Mem_Read(hi2c, HDC2021_I2C_ADDR, reg_addr, I2C_MEMADD_SIZE_8BIT,
			data, 1, HDC2021_I2C_TIMEOUT_MS) != HAL_OK)
	{
		// Error occurred during I2C read.
		return HAL_ERROR;
	}
	return HAL_OK;
}

/**
 * @brief Writes a single byte to a specified HDC2021 register.
 * @param hi2c Pointer to the I2C_HandleTypeDef structure for the I2C peripheral.
 * @param reg_addr Address of the register to write.
 * @param data Data to write to the register.
 * @return HAL_StatusTypeDef HAL_OK if write is successful, HAL_ERROR otherwise.
 */
HAL_StatusTypeDef HDC2021_WriteRegister(I2C_HandleTypeDef *hi2c,
		uint8_t reg_addr, uint8_t data)
{
	// Use HAL_I2C_Mem_Write to write one byte to the specified register address.
	// The device address is HDC2021_I2C_ADDR, register address size is 1 byte (I2C_MEMADD_SIZE_8BIT).
	// Timeout is HDC2021_I2C_TIMEOUT_MS.
	if (HAL_I2C_Mem_Write(hi2c, HDC2021_I2C_ADDR, reg_addr,
			I2C_MEMADD_SIZE_8BIT, &data, 1, HDC2021_I2C_TIMEOUT_MS) != HAL_OK)
	{
		// Error occurred during I2C write.
		return HAL_ERROR;
	}
	return HAL_OK;
}

/* Public Functions ----------------------------------------------------------*/

/**
 * @brief Initializes the HDC2021 sensor.
 * @param hi2c Pointer to the I2C_HandleTypeDef structure for the I2C peripheral.
 * @param tempRes Temperature measurement resolution.
 * @param humidRes Humidity measurement resolution.
 * @param measRate Continuous measurement rate (set to HDC2021_RATE_OFF for single measurement mode).
 * @return HAL_StatusTypeDef HAL_OK if initialization is successful, HAL_ERROR otherwise.
 */
HAL_StatusTypeDef HDC2021_Init(I2C_HandleTypeDef *hi2c,
		HDC2021_Resolution_t tempRes, HDC2021_Resolution_t humidRes,
		HDC2021_Rate_t measRate)
{
	HAL_StatusTypeDef status;
	uint8_t config_reg_val;
	uint8_t meas_config_reg_val;

	// Perform a soft reset to ensure the sensor is in a known state
	status = HDC2021_SoftReset(hi2c);
	if (status != HAL_OK)
	{
		return status;
	}
	// Give some time for reset to complete
	HAL_Delay(10);

	// --- Configure Measurement Configuration Register (0x0F) ---
	// Set Temperature Resolution (bits 1:0)
	// Set Humidity Resolution (bits 3:2)
	// Set Measurement Rate (bits 6:4)
	// Clear Continuous Measurement Mode Enable (bit 7) if measRate is OFF, otherwise set it
	meas_config_reg_val = 0x00; // Start with a clear register value

	// Set Temperature Resolution
	meas_config_reg_val |= (tempRes & 0x03) << 0;

	// Set Humidity Resolution
	meas_config_reg_val |= (humidRes & 0x03) << 2;

	// Set Measurement Rate and Continuous Measurement Mode
	if (measRate == HDC2021_RATE_OFF)
	{
		// Single shot mode, clear continuous mode enable
		meas_config_reg_val &= ~HDC2021_MEAS_CONT_MODE_EN;
		meas_config_reg_val &= ~HDC2021_MEAS_RATE_MASK; // Ensure rate bits are clear
	}
	else
	{
		// Continuous measurement mode
		meas_config_reg_val |= HDC2021_MEAS_CONT_MODE_EN; // Enable continuous mode
		meas_config_reg_val |= (measRate << 4) & HDC2021_MEAS_RATE_MASK; // Set rate
	}

	status = HDC2021_WriteRegister(hi2c, HDC2021_REG_MEAS_CONFIG,
			meas_config_reg_val);
	if (status != HAL_OK)
	{
		return status;
	}

	// --- Configure Configuration Register (0x0E) ---
	// Read current config to avoid overwriting other settings, then modify.
	status = HDC2021_ReadRegister(hi2c, HDC2021_REG_CONFIG, &config_reg_val);
	if (status != HAL_OK)
	{
		return status;
	}

	// Set Mode (Measurement cycle enabled if in continuous mode, otherwise keep sleep)
	if (measRate != HDC2021_RATE_OFF)
	{
		config_reg_val |= HDC2021_CONFIG_MODE_MEAS; // Enable measurement cycle
	}
	else
	{
		config_reg_val &= ~HDC2021_CONFIG_MODE_MEAS; // Keep in sleep mode for single shot
	}

	// We can also disable heater or data ready interrupt if not needed for simplicity
	config_reg_val &= ~HDC2021_CONFIG_HEATER_EN;      // Disable heater for now
	config_reg_val &= ~HDC2021_CONFIG_DRDY_INT_EN; // Disable data ready interrupt for now

	status = HDC2021_WriteRegister(hi2c, HDC2021_REG_CONFIG, config_reg_val);
	if (status != HAL_OK)
	{
		return status;
	}

	return HAL_OK;
}

/**
 * @brief Triggers a single measurement on the HDC2021 sensor.
 * This function is typically used in single shot measurement mode.
 * @param hi2c Pointer to the I2C_HandleTypeDef structure for the I2C peripheral.
 * @return HAL_StatusTypeDef HAL_OK if trigger is successful, HAL_ERROR otherwise.
 */
HAL_StatusTypeDef HDC2021_TriggerMeasurement(I2C_HandleTypeDef *hi2c)
{
	HAL_StatusTypeDef status;
	uint8_t meas_config_reg_val;

	// Read current measurement config register
	status = HDC2021_ReadRegister(hi2c, HDC2021_REG_MEAS_CONFIG,
			&meas_config_reg_val);
	if (status != HAL_OK)
	{
		return status;
	}

	// Set the MEAS_TRIG bit (bit 0) to trigger a measurement
	meas_config_reg_val |= HDC2021_MEAS_TRIG;

	status = HDC2021_WriteRegister(hi2c, HDC2021_REG_MEAS_CONFIG,
			meas_config_reg_val);
	if (status != HAL_OK)
	{
		return status;
	}

	// Wait for the measurement to complete.
	// Max conversion time for 14-bit resolution is ~8ms.
	// A longer delay is safer to ensure data is ready.
	HAL_Delay(HDC2021_MEAS_DELAY_MS);

	return HAL_OK;
}

/**
 * @brief Reads the temperature from the HDC2021 sensor.
 * @param hi2c Pointer to the I2C_HandleTypeDef structure for the I2C peripheral.
 * @return float Temperature in degrees Celsius. Returns a large negative number on error.
 */
float HDC2021_ReadTemperature(I2C_HandleTypeDef *hi2c)
{
	uint8_t temp_data[2]; // temp_low, temp_high
	HAL_StatusTypeDef status;
	uint16_t raw_temp;
	float temperature_celsius = -999.0f; // Default error value

	// Read Temperature Low and High registers
	// Use HAL_I2C_Mem_Read to read two bytes starting from HDC2021_REG_TEMP_LOW.
	if (HAL_I2C_Mem_Read(hi2c, HDC2021_I2C_ADDR, HDC2021_REG_TEMP_LOW,
			I2C_MEMADD_SIZE_8BIT, temp_data, 2, HDC2021_I2C_TIMEOUT_MS)
			!= HAL_OK)
	{
		return temperature_celsius; // Return error value
	}

	// Combine the two bytes into a 16-bit raw value
	// Data is 14-bit, but read as 16-bit. LSB is 0x00, MSB is 0x01.
	// The MSB (High byte) is received first, then LSB (Low byte).
	// The datasheet implies that Temp_High (0x01) contains bits [15:8] and Temp_Low (0x00) contains bits [7:0].
	// This forms a 16-bit unsigned integer. The 14-bit value is the most significant 14 bits of this 16-bit value.
	// So, Temp_High contains bits D13-D6 and Temp_Low contains bits D5-D0 plus two LSBs (D1, D0) which are 0.
	// We combine them as (High_Byte << 8) | Low_Byte.
	raw_temp = (uint16_t) ((temp_data[1] << 8) | temp_data[0]);

	// Convert raw temperature to Celsius
	// The datasheet formula for 14-bit resolution: Temp = ((raw_temp / 65536.0) * 165.0) - 40.0
	temperature_celsius = ((float) raw_temp / 65536.0f) * 165.0f - 40.0f;

	return temperature_celsius;
}

/**
 * @brief Reads the humidity from the HDC2021 sensor.
 * @param hi2c Pointer to the I2C_HandleTypeDef structure for the I2C peripheral.
 * @return float Humidity in %RH. Returns a large negative number on error.
 */
float HDC2021_ReadHumidity(I2C_HandleTypeDef *hi2c)
{
	uint8_t humid_data[2]; // humid_low, humid_high
	HAL_StatusTypeDef status;
	uint16_t raw_humid;
	float humidity_rh = -999.0f; // Default error value

	// Read Humidity Low and High registers
	// Use HAL_I2C_Mem_Read to read two bytes starting from HDC2021_REG_HUMID_LOW.
	if (HAL_I2C_Mem_Read(hi2c, HDC2021_I2C_ADDR, HDC2021_REG_HUMID_LOW,
			I2C_MEMADD_SIZE_8BIT, humid_data, 2, HDC2021_I2C_TIMEOUT_MS)
			!= HAL_OK)
	{
		return humidity_rh; // Return error value
	}

	// Combine the two bytes into a 16-bit raw value
	// Similar to temperature, High_Byte << 8 | Low_Byte.
	raw_humid = (uint16_t) ((humid_data[1] << 8) | humid_data[0]);

	// Convert raw humidity to %RH
	// The datasheet formula for 14-bit resolution: Humid = ((raw_humid / 65536.0) * 100.0)
	humidity_rh = ((float) raw_humid / 65536.0f) * 100.0f;

	return humidity_rh;
}

/**
 * @brief Resets the HDC2021 sensor.
 * @param hi2c Pointer to the I2C_HandleTypeDef structure for the I2C peripheral.
 * @return HAL_StatusTypeDef HAL_OK if reset is successful, HAL_ERROR otherwise.
 */
HAL_StatusTypeDef HDC2021_SoftReset(I2C_HandleTypeDef *hi2c)
{
	HAL_StatusTypeDef status;
	uint8_t config_reg_val;

	// Read the current configuration register value
	status = HDC2021_ReadRegister(hi2c, HDC2021_REG_CONFIG, &config_reg_val);
	if (status != HAL_OK)
	{
		return status;
	}

	// Set the RST bit (bit 7) to trigger a soft reset
	config_reg_val |= HDC2021_CONFIG_RST;

	// Write the modified configuration register back to the sensor
	status = HDC2021_WriteRegister(hi2c, HDC2021_REG_CONFIG, config_reg_val);
	if (status != HAL_OK)
	{
		return status;
	}
	// The RST bit self-clears, so no need to clear it manually.

	return HAL_OK;
}
