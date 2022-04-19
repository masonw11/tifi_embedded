/*
 * TMP116.c
 *
 *  Created on: Apr 15, 2022
 *      Author: MEW24
 */

#include "TMP116.h"

#define SUCCESS 0
#define FAILURE -1

static int8_t get_register_value(I2C_Handle i2c_handle, uint8_t device_address,
                                 uint8_t register_address,
                                 uint16_t *register_value);

/*!
 * Reads data in a specified register over I2c to a value destination.
 *
 * @param[in] i2c_handle The handle to the specified I2C device.
 * @param[in] device_address The I2C slave address of the device.
 * @param[in] register_address The register of the value to be read.
 * @param[out] val_dest The pointer to where the register value will be stored.
 *
 * @return 0: Success, < 0: Failure.
 */
int8_t get_register_value(I2C_Handle i2c_handle, uint8_t device_address,
                          uint8_t register_address, uint16_t *val_dest)
{
    I2C_Transaction i2c_transaction;
    uint8_t register_rx_buf[2];
    int8_t status;

    /* Set up the transaction parameters to write the address byte and read two bytes back. */
    i2c_transaction.slaveAddress = device_address;
    i2c_transaction.writeCount = 1;
    i2c_transaction.writeBuf = &register_address;
    i2c_transaction.readCount = 2;
    i2c_transaction.readBuf = register_rx_buf;

    status = I2C_transfer(i2c_handle, &i2c_transaction);

    if (!status)
    {
        /* TODO: use the I2C error handler */
        return (FAILURE);
    }

    *val_dest = (uint16_t) (register_rx_buf[0] << 8 | register_rx_buf[1]);
    return (SUCCESS);
}

/*!
 * Reads temperature data from the result register to a temperature destination.
 *
 * @param[in] i2c_handle The handle to the specified I2C device.
 * @param[in] device_address The I2C slave address of the device.
 * @param[out] temp_dest The pointer to where the temperature value will be stored in degrees Celsius.
 *
 * @return 0: Success, < 0: Failure.
 */
int8_t get_temperature(I2C_Handle i2c_handle, float *temp_dest)
{
    uint16_t raw_temp_data;
    int8_t status;

    status = get_register_value(i2c_handle, TMP116_ADDR, TMP116_RESULT_REG, &raw_temp_data);
    if (status != 0)
    {
        return (FAILURE);
    }

    /* 0.0078125°C resolution */
    *temp_dest = raw_temp_data * 0.0078125;
    return (SUCCESS);
}
