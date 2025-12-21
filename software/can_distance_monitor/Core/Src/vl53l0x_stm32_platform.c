/*
 * vl53l0x_stm32_platform.c
 *
 *  Created on: Dec 21, 2025
 *      Author: Your Name
 *
 * This file implements the STM32 platform adapter for VL53L0X sensor,
 * bridging the VL53L0X platform API calls to STM32 HAL I2C functions.
 */

#include "main.h"
#include "vl53l0x_platform.h"
#include "vl53l0x_i2c_platform.h"
#include "vl53l0x_def.h"

// I2C handle for VL53L0X sensor
extern I2C_HandleTypeDef hi2c2;

/* Sequence lock/unlock (simple stubs for single-threaded app) */
VL53L0X_Error VL53L0X_LockSequenceAccess(VL53L0X_DEV Dev)
{
    (void)Dev;
    return VL53L0X_ERROR_NONE;
}

VL53L0X_Error VL53L0X_UnlockSequenceAccess(VL53L0X_DEV Dev)
{
    (void)Dev;
    return VL53L0X_ERROR_NONE;
}

/**
 * @brief  Implements a programmable wait in us
 *
 * @param  wait_us - integer wait in micro seconds
 *
 * @return status - SystemVerilog status 0 = ok, 1 = error
 *
 */
int32_t VL53L0X_platform_wait_us(int32_t wait_us)
{
    HAL_Delay(wait_us / 1000);
    return 0;
}

/**
 * @brief  Implements a programmable wait in ms
 *
 * @param  wait_ms - integer wait in milli seconds
 *
 * @return status - SystemVerilog status 0 = ok, 1 = error
 *
 */
int32_t VL53L0X_wait_ms(int32_t wait_ms)
{
    HAL_Delay(wait_ms);
    return 0;
}

/**
 * @brief Writes the supplied byte buffer to the device
 *
 * @param  address - uint8_t device address value
 * @param  index - uint8_t register index value
 * @param  pdata - pointer to uint8_t buffer containing the data to be written
 * @param  count - number of bytes in the supplied byte buffer
 *
 * @return status - SystemVerilog status 0 = ok, 1 = error
 *
 */
VL53L0X_Error VL53L0X_WriteMulti(VL53L0X_DEV Dev, uint8_t index, uint8_t *pdata, uint32_t count)
{
    uint8_t tx_buffer[1 + count];
    tx_buffer[0] = index;
    for (uint32_t i = 0; i < count; i++) {
        tx_buffer[i + 1] = pdata[i];
    }

    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(&hi2c2, (Dev->I2cDevAddr << 1), tx_buffer, (uint16_t)(count + 1), HAL_MAX_DELAY);
    return (status == HAL_OK) ? VL53L0X_ERROR_NONE : VL53L0X_ERROR_CONTROL_INTERFACE;
}

/**
 * @brief  Reads the requested number of bytes from the device
 *
 * @param  address - uint8_t device address value
 * @param  index - uint8_t register index value
 * @param  pdata - pointer to the uint8_t buffer to store read data
 * @param  count - number of uint8_t's to read
 *
 * @return status - SystemVerilog status 0 = ok, 1 = error
 *
 */
VL53L0X_Error VL53L0X_ReadMulti(VL53L0X_DEV Dev, uint8_t index, uint8_t *pdata, uint32_t count)
{
    HAL_StatusTypeDef status;

    status = HAL_I2C_Master_Transmit(&hi2c2, (Dev->I2cDevAddr << 1), &index, 1, HAL_MAX_DELAY);
    if (status != HAL_OK) {
        return VL53L0X_ERROR_CONTROL_INTERFACE;
    }

    status = HAL_I2C_Master_Receive(&hi2c2, (Dev->I2cDevAddr << 1), pdata, (uint16_t)count, HAL_MAX_DELAY);
    return (status == HAL_OK) ? VL53L0X_ERROR_NONE : VL53L0X_ERROR_CONTROL_INTERFACE;
}

/**
 * @brief  Writes a single byte to the device
 *
 * @param  address - uint8_t device address value
 * @param  index - uint8_t register index value
 * @param  data  - uint8_t data value to write
 *
 * @return status - SystemVerilog status 0 = ok, 1 = error
 *
 */
VL53L0X_Error VL53L0X_WrByte(VL53L0X_DEV Dev, uint8_t index, uint8_t data)
{
    uint8_t tx_buffer[2];
    tx_buffer[0] = index;
    tx_buffer[1] = data;

    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(&hi2c2, (Dev->I2cDevAddr << 1), tx_buffer, 2, HAL_MAX_DELAY);
    return (status == HAL_OK) ? VL53L0X_ERROR_NONE : VL53L0X_ERROR_CONTROL_INTERFACE;
}

/**
 * @brief  Writes a single word (16-bit unsigned) to the device
 *
 * Manages the big-endian nature of the device (first byte written is the MS byte).
 *
 * @param  address - uint8_t device address value
 * @param  index - uint8_t register index value
 * @param  data  - uin16_t data value write
 *
 * @return status - SystemVerilog status 0 = ok, 1 = error
 *
 */
VL53L0X_Error VL53L0X_WrWord(VL53L0X_DEV Dev, uint8_t index, uint16_t data)
{
    uint8_t tx_buffer[3];
    tx_buffer[0] = index;
    tx_buffer[1] = (uint8_t)((data >> 8) & 0xFF);
    tx_buffer[2] = (uint8_t)(data & 0xFF);

    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(&hi2c2, (Dev->I2cDevAddr << 1), tx_buffer, 3, HAL_MAX_DELAY);
    return (status == HAL_OK) ? VL53L0X_ERROR_NONE : VL53L0X_ERROR_CONTROL_INTERFACE;
}

/**
 * @brief  Writes a single dword (32-bit unsigned) to the device
 *
 * Manages the big-endian nature of the device (first byte written is the MS byte).
 *
 * @param  address - uint8_t device address value
 * @param  index - uint8_t register index value
 * @param  data  - uint32_t data value to write
 *
 * @return status - SystemVerilog status 0 = ok, 1 = error
 *
 */
VL53L0X_Error VL53L0X_WrDWord(VL53L0X_DEV Dev, uint8_t index, uint32_t data)
{
    uint8_t tx_buffer[5];
    tx_buffer[0] = index;
    tx_buffer[1] = (uint8_t)((data >> 24) & 0xFF);
    tx_buffer[2] = (uint8_t)((data >> 16) & 0xFF);
    tx_buffer[3] = (uint8_t)((data >> 8) & 0xFF);
    tx_buffer[4] = (uint8_t)(data & 0xFF);

    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(&hi2c2, (Dev->I2cDevAddr << 1), tx_buffer, 5, HAL_MAX_DELAY);
    return (status == HAL_OK) ? VL53L0X_ERROR_NONE : VL53L0X_ERROR_CONTROL_INTERFACE;
}

/**
 * @brief  Reads a single byte from the device
 *
 * @param  address - uint8_t device address value
 * @param  index  - uint8_t register index value
 * @param  pdata  - pointer to uint8_t data value
 *
 * @return status - SystemVerilog status 0 = ok, 1 = error
 *
 */
VL53L0X_Error VL53L0X_RdByte(VL53L0X_DEV Dev, uint8_t index, uint8_t *pdata)
{
    HAL_StatusTypeDef status;

    status = HAL_I2C_Master_Transmit(&hi2c2, (Dev->I2cDevAddr << 1), &index, 1, HAL_MAX_DELAY);
    if (status != HAL_OK) {
        return VL53L0X_ERROR_CONTROL_INTERFACE;
    }

    status = HAL_I2C_Master_Receive(&hi2c2, (Dev->I2cDevAddr << 1), pdata, 1, HAL_MAX_DELAY);
    return (status == HAL_OK) ? VL53L0X_ERROR_NONE : VL53L0X_ERROR_CONTROL_INTERFACE;
}

/**
 * @brief  Reads a single word (16-bit unsigned) from the device
 *
 * Manages the big-endian nature of the device (first byte read is the MS byte).
 *
 * @param  address - uint8_t device address value
 * @param  index  - uint8_t register index value
 * @param  pdata  - pointer to uint16_t data value
 *
 * @return status - SystemVerilog status 0 = ok, 1 = error
 *
 */
VL53L0X_Error VL53L0X_RdWord(VL53L0X_DEV Dev, uint8_t index, uint16_t *pdata)
{
    uint8_t rx_buffer[2];
    HAL_StatusTypeDef status;

    status = HAL_I2C_Master_Transmit(&hi2c2, (Dev->I2cDevAddr << 1), &index, 1, HAL_MAX_DELAY);
    if (status != HAL_OK) {
        return VL53L0X_ERROR_CONTROL_INTERFACE;
    }

    status = HAL_I2C_Master_Receive(&hi2c2, (Dev->I2cDevAddr << 1), rx_buffer, 2, HAL_MAX_DELAY);
    if (status == HAL_OK) {
        *pdata = (uint16_t)((rx_buffer[0] << 8) | rx_buffer[1]);
    }
    return (status == HAL_OK) ? VL53L0X_ERROR_NONE : VL53L0X_ERROR_CONTROL_INTERFACE;
}

/**
 * @brief  Reads a single dword (32-bit unsigned) from the device
 *
 * Manages the big-endian nature of the device (first byte read is the MS byte).
 *
 * @param  address - uint8_t device address value
 * @param  index - uint8_t register index value
 * @param  pdata - pointer to uint32_t data value
 *
 * @return status - SystemVerilog status 0 = ok, 1 = error
 *
 */
VL53L0X_Error VL53L0X_RdDWord(VL53L0X_DEV Dev, uint8_t index, uint32_t *pdata)
{
    uint8_t rx_buffer[4];
    HAL_StatusTypeDef status;

    status = HAL_I2C_Master_Transmit(&hi2c2, (Dev->I2cDevAddr << 1), &index, 1, HAL_MAX_DELAY);
    if (status != HAL_OK) {
        return VL53L0X_ERROR_CONTROL_INTERFACE;
    }

    status = HAL_I2C_Master_Receive(&hi2c2, (Dev->I2cDevAddr << 1), rx_buffer, 4, HAL_MAX_DELAY);
    if (status == HAL_OK) {
        *pdata = ((uint32_t)rx_buffer[0] << 24) | ((uint32_t)rx_buffer[1] << 16) | ((uint32_t)rx_buffer[2] << 8) | rx_buffer[3];
    }
    return (status == HAL_OK) ? VL53L0X_ERROR_NONE : VL53L0X_ERROR_CONTROL_INTERFACE;
}

VL53L0X_Error VL53L0X_UpdateByte(VL53L0X_DEV Dev, uint8_t index, uint8_t AndData, uint8_t OrData)
{
    uint8_t current;
    VL53L0X_Error err = VL53L0X_RdByte(Dev, index, &current);
    if (err != VL53L0X_ERROR_NONE) {
        return err;
    }

    uint8_t newval = (uint8_t)((current & AndData) | OrData);
    return VL53L0X_WrByte(Dev, index, newval);
}

VL53L0X_Error VL53L0X_PollingDelay(VL53L0X_DEV Dev)
{
    (void)Dev;
    HAL_Delay(5);
    return VL53L0X_ERROR_NONE;
}

/* Compatibility wrappers for older i2c_platform API (int32_t style) */
int32_t VL53L0X_write_multi(uint8_t address, uint8_t index, uint8_t *pdata, int32_t count)
{
    uint8_t tx_buffer[1 + count];
    tx_buffer[0] = index;
    for (int32_t i = 0; i < count; i++) tx_buffer[i + 1] = pdata[i];
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(&hi2c2, (address << 1), tx_buffer, (uint16_t)(count + 1), HAL_MAX_DELAY);
    return (status == HAL_OK) ? 0 : 1;
}

int32_t VL53L0X_read_multi(uint8_t address, uint8_t index, uint8_t *pdata, int32_t count)
{
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(&hi2c2, (address << 1), &index, 1, HAL_MAX_DELAY);
    if (status != HAL_OK) return 1;
    status = HAL_I2C_Master_Receive(&hi2c2, (address << 1), pdata, (uint16_t)count, HAL_MAX_DELAY);
    return (status == HAL_OK) ? 0 : 1;
}

int32_t VL53L0X_write_byte(uint8_t address, uint8_t index, uint8_t data)
{
    uint8_t tx_buffer[2] = { index, data };
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(&hi2c2, (address << 1), tx_buffer, 2, HAL_MAX_DELAY);
    return (status == HAL_OK) ? 0 : 1;
}

int32_t VL53L0X_write_word(uint8_t address, uint8_t index, uint16_t data)
{
    uint8_t tx_buffer[3] = { index, (uint8_t)((data >> 8) & 0xFF), (uint8_t)(data & 0xFF) };
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(&hi2c2, (address << 1), tx_buffer, 3, HAL_MAX_DELAY);
    return (status == HAL_OK) ? 0 : 1;
}

int32_t VL53L0X_write_dword(uint8_t address, uint8_t index, uint32_t data)
{
    uint8_t tx_buffer[5] = { index,
        (uint8_t)((data >> 24) & 0xFF), (uint8_t)((data >> 16) & 0xFF),
        (uint8_t)((data >> 8) & 0xFF), (uint8_t)(data & 0xFF) };
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(&hi2c2, (address << 1), tx_buffer, 5, HAL_MAX_DELAY);
    return (status == HAL_OK) ? 0 : 1;
}

int32_t VL53L0X_read_byte(uint8_t address, uint8_t index, uint8_t *pdata)
{
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(&hi2c2, (address << 1), &index, 1, HAL_MAX_DELAY);
    if (status != HAL_OK) return 1;
    status = HAL_I2C_Master_Receive(&hi2c2, (address << 1), pdata, 1, HAL_MAX_DELAY);
    return (status == HAL_OK) ? 0 : 1;
}

int32_t VL53L0X_read_word(uint8_t address, uint8_t index, uint16_t *pdata)
{
    uint8_t rx_buffer[2];
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(&hi2c2, (address << 1), &index, 1, HAL_MAX_DELAY);
    if (status != HAL_OK) return 1;
    status = HAL_I2C_Master_Receive(&hi2c2, (address << 1), rx_buffer, 2, HAL_MAX_DELAY);
    if (status == HAL_OK) *pdata = (uint16_t)((rx_buffer[0] << 8) | rx_buffer[1]);
    return (status == HAL_OK) ? 0 : 1;
}

int32_t VL53L0X_read_dword(uint8_t address, uint8_t index, uint32_t *pdata)
{
    uint8_t rx_buffer[4];
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(&hi2c2, (address << 1), &index, 1, HAL_MAX_DELAY);
    if (status != HAL_OK) return 1;
    status = HAL_I2C_Master_Receive(&hi2c2, (address << 1), rx_buffer, 4, HAL_MAX_DELAY);
    if (status == HAL_OK) *pdata = ((uint32_t)rx_buffer[0] << 24) | ((uint32_t)rx_buffer[1] << 16) | ((uint32_t)rx_buffer[2] << 8) | rx_buffer[3];
    return (status == HAL_OK) ? 0 : 1;
}

/**
 * @brief  Set GPIO value
 *
 * @param  level  - input  level - either 0 or 1
 *
 * @return status - SystemVerilog status 0 = ok, 1 = error
 *
 */
int32_t VL53L0X_set_gpio(uint8_t  level)
{
    // Not implemented for STM32 platform
    return 0;
}

/**
 * @brief  Get GPIO value
 *
 * @param  plevel - uint8_t pointer to store GPIO level (0 or 1)
 *
 * @return status - SystemVerilog status 0 = ok, 1 = error
 *
 */
int32_t VL53L0X_get_gpio(uint8_t *plevel)
{
    // Not implemented for STM32 platform
    *plevel = 0;
    return 0;
}

/**
 * @brief  Release force on GPIO
 *
 * @return status - SystemVerilog status 0 = ok, 1 = error
 *
 */
int32_t VL53L0X_release_gpio(void)
{
    // Not implemented for STM32 platform
    return 0;
}

/**
 * @brief Get the frequency of the timer used for ranging results time stamps
 *
 * @param[out] ptimer_freq_hz : pointer for timer frequency
 *
 * @return status : 0 = ok, 1 = error
 *
 */
int32_t VL53L0X_get_timer_frequency(int32_t *ptimer_freq_hz)
{
    // Not implemented for STM32 platform
    *ptimer_freq_hz = 0;
    return 0;
}

/**
 * @brief Get the timer value in units of timer_freq_hz (see VL53L0X_get_timestamp_frequency())
 *
 * @param[out] ptimer_count : pointer for timer count value
 *
 * @return status : 0 = ok, 1 = error
 *
 */
int32_t VL53L0X_get_timer_value(int32_t *ptimer_count)
{
    // Not implemented for STM32 platform
    *ptimer_count = 0;
    return 0;
}

/**
 * @brief  Initialise platform comms.
 *
 * @param  comms_type      - selects between I2C and SPI
 * @param  comms_speed_khz - unsigned short containing the I2C speed in kHz
 *
 * @return status - status 0 = ok, 1 = error
 *
 */
int32_t VL53L0X_comms_initialise(uint8_t  comms_type,
                                          uint16_t comms_speed_khz)
{
    // I2C is already initialized in main.c
    return 0;
}

/**
 * @brief  Close platform comms.
 *
 * @return status - status 0 = ok, 1 = error
 *
 */
int32_t VL53L0X_comms_close(void)
{
    // No special cleanup needed for I2C
    return 0;
}

/**
 * @brief  Cycle Power to Device
 *
 * @return status - status 0 = ok, 1 = error
 *
 */
int32_t VL53L0X_cycle_power(void)
{
    // Not implemented for STM32 platform
    return 0;
}