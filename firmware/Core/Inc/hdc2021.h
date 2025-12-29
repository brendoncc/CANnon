/**
 * @file hdc2021.h
 * @brief Header file for the HDC2021 Temperature/Humidity Sensor Driver.
 *
 * This file contains the function prototypes, register definitions,
 * and configuration options for the HDC2021 sensor.
 */

#ifndef __HDC2021_H
#define __HDC2021_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g0xx_hal.h" // Assuming you are using STM32 HAL library

/* Defines -------------------------------------------------------------------*/

/**
 * @brief HDC2021 I2C Slave Address
 * The ADDR pin can be used to set the 7-bit I2C address.
 * If ADDR is connected to GND, the address is 0x40.
 * If ADDR is connected to VDD, the address is 0x41.
 * Shifted left by 1 for 8-bit R/W address.
 */
#define HDC2021_I2C_ADDR             (0x40 << 1) // 0x80 for write, 0x81 for read (if ADDR pin is GND)

/* HDC2021 Register Addresses */
#define HDC2021_REG_TEMP_LOW         0x00
#define HDC2021_REG_TEMP_HIGH        0x01
#define HDC2021_REG_HUMID_LOW        0x02
#define HDC2021_REG_HUMID_HIGH       0x03
#define HDC2021_REG_CONFIG           0x0E
#define HDC2021_REG_MEAS_CONFIG      0x0F
#define HDC2021_REG_TEMP_MAX         0x10
#define HDC2021_REG_HUMID_MAX        0x11
#define HDC2021_REG_INTERRUPT_DRDY   0x12
#define HDC2021_REG_INTERRUPT_MASK   0x13
#define HDC2021_REG_TEMP_THR_LOW     0x14
#define HDC2021_REG_TEMP_THR_HIGH    0x15
#define HDC2021_REG_HUMID_THR_LOW    0x16
#define HDC2021_REG_HUMID_THR_HIGH   0x17
#define HDC2021_REG_RST_DRDY_INT_CONF 0x18
#define HDC2021_REG_MANUFACTURER_ID_LOW 0xFC
#define HDC2021_REG_MANUFACTURER_ID_HIGH 0xFD
#define HDC2021_REG_DEVICE_ID_LOW    0xFE
#define HDC2021_REG_DEVICE_ID_HIGH   0xFF

/* Configuration Register (0x0E) Bit Definitions */
#define HDC2021_CONFIG_RST           (1 << 7) // Soft Reset
#define HDC2021_CONFIG_HEATER_EN     (1 << 3) // Heater Enable
#define HDC2021_CONFIG_MODE_MEAS     (1 << 2) // Mode: 1=Measurement cycle, 0=Sleep
#define HDC2021_CONFIG_DRDY_INT_EN   (1 << 0) // Data Ready Interrupt Enable

/* Measurement Configuration Register (0x0F) Bit Definitions */
#define HDC2021_MEAS_TRIG            (1 << 0) // Trigger Measurement (self-clear)
#define HDC2021_MEAS_CONT_MODE_EN    (1 << 7) // Continuous Measurement Mode Enable
#define HDC2021_MEAS_RATE_MASK       (0x07 << 4) // Measurement Rate Mask (bits 6:4)
#define HDC2021_MEAS_RATE_OFF        (0 << 4)
#define HDC2021_MEAS_RATE_0_125_HZ   (1 << 4)
#define HDC2021_MEAS_RATE_0_25_HZ    (2 << 4)
#define HDC2021_MEAS_RATE_0_5_HZ     (3 << 4)
#define HDC2021_MEAS_RATE_1_HZ       (4 << 4)
#define HDC2021_MEAS_RATE_2_HZ       (5 << 4)
#define HDC2021_MEAS_RATE_5_HZ       (6 << 4)
#define HDC2021_MEAS_RATE_10_HZ      (7 << 4)

#define HDC2021_MEAS_TEMP_RES_MASK   (0x03 << 0) // Temperature Resolution Mask (bits 1:0)
#define HDC2021_MEAS_TEMP_RES_14BIT  (0 << 0)
#define HDC2021_MEAS_TEMP_RES_11BIT  (1 << 0)
#define HDC2021_MEAS_TEMP_RES_9BIT   (2 << 0)

#define HDC2021_MEAS_HUMID_RES_MASK  (0x03 << 2) // Humidity Resolution Mask (bits 3:2)
#define HDC2021_MEAS_HUMID_RES_14BIT (0 << 2)
#define HDC2021_MEAS_HUMID_RES_11BIT (1 << 2)
#define HDC2021_MEAS_HUMID_RES_9BIT  (2 << 2)

/* Typedefs ------------------------------------------------------------------*/

/**
 * @brief Enum for HDC2021 measurement resolution settings.
 */
typedef enum
{
    HDC2021_RESOLUTION_14BIT = 0,
    HDC2021_RESOLUTION_11BIT = 1,
    HDC2021_RESOLUTION_9BIT  = 2
} HDC2021_Resolution_t;

/**
 * @brief Enum for HDC2021 measurement rate settings.
 */
typedef enum
{
    HDC2021_RATE_OFF = 0,
    HDC2021_RATE_0_125_HZ,
    HDC2021_RATE_0_25_HZ,
    HDC2021_RATE_0_5_HZ,
    HDC2021_RATE_1_HZ,
    HDC2021_RATE_2_HZ,
    HDC2021_RATE_5_HZ,
    HDC2021_RATE_10_HZ
} HDC2021_Rate_t;

/* Function Prototypes -------------------------------------------------------*/

/**
 * @brief Initializes the HDC2021 sensor.
 * @param hi2c Pointer to the I2C_HandleTypeDef structure for the I2C peripheral.
 * @param tempRes Temperature measurement resolution.
 * @param humidRes Humidity measurement resolution.
 * @param measRate Continuous measurement rate (set to HDC2021_RATE_OFF for single measurement mode).
 * @return HAL_StatusTypeDef HAL_OK if initialization is successful, HAL_ERROR otherwise.
 */
HAL_StatusTypeDef HDC2021_Init(I2C_HandleTypeDef *hi2c, HDC2021_Resolution_t tempRes,
                               HDC2021_Resolution_t humidRes, HDC2021_Rate_t measRate);

/**
 * @brief Triggers a single measurement on the HDC2021 sensor.
 * This function is typically used in single shot measurement mode.
 * @param hi2c Pointer to the I2C_HandleTypeDef structure for the I2C peripheral.
 * @return HAL_StatusTypeDef HAL_OK if trigger is successful, HAL_ERROR otherwise.
 */
HAL_StatusTypeDef HDC2021_TriggerMeasurement(I2C_HandleTypeDef *hi2c);

/**
 * @brief Reads the temperature from the HDC2021 sensor.
 * @param hi2c Pointer to the I2C_HandleTypeDef structure for the I2C peripheral.
 * @return float Temperature in degrees Celsius. Returns a large negative number on error.
 */
float HDC2021_ReadTemperature(I2C_HandleTypeDef *hi2c);

/**
 * @brief Reads the humidity from the HDC2021 sensor.
 * @param hi2c Pointer to the I2C_HandleTypeDef structure for the I2C peripheral.
 * @return float Humidity in %RH. Returns a large negative number on error.
 */
float HDC2021_ReadHumidity(I2C_HandleTypeDef *hi2c);

/**
 * @brief Resets the HDC2021 sensor.
 * @param hi2c Pointer to the I2C_HandleTypeDef structure for the I2C peripheral.
 * @return HAL_StatusTypeDef HAL_OK if reset is successful, HAL_ERROR otherwise.
 */
HAL_StatusTypeDef HDC2021_SoftReset(I2C_HandleTypeDef *hi2c);

/**
 * @brief Reads a single byte from a specified HDC2021 register.
 * @param hi2c Pointer to the I2C_HandleTypeDef structure for the I2C peripheral.
 * @param reg_addr Address of the register to read.
 * @param data Pointer to store the read data.
 * @return HAL_StatusTypeDef HAL_OK if read is successful, HAL_ERROR otherwise.
 */
HAL_StatusTypeDef HDC2021_ReadRegister(I2C_HandleTypeDef *hi2c, uint8_t reg_addr, uint8_t *data);

/**
 * @brief Writes a single byte to a specified HDC2021 register.
 * @param hi2c Pointer to the I2C_HandleTypeDef structure for the I2C peripheral.
 * @param reg_addr Address of the register to write.
 * @param data Data to write to the register.
 * @return HAL_StatusTypeDef HAL_OK if write is successful, HAL_ERROR otherwise.
 */
HAL_StatusTypeDef HDC2021_WriteRegister(I2C_HandleTypeDef *hi2c, uint8_t reg_addr, uint8_t data);


#ifdef __cplusplus
}
#endif

#endif /* __HDC2021_H */

