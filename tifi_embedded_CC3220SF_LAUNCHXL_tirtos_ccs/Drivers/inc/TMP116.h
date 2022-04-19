/*
 * TMP116.h
 *
 *  Created on: Apr 15, 2022
 *      Author: MEW24
 */

#ifndef DRIVERS_INC_TMP116_H_
#define DRIVERS_INC_TMP116_H_

/* TI-DRIVERS Header files */
#include <ti/drivers/I2C.h>

/* Driver configuration */
#include "ti_drivers_config.h"

#define TMP116_ADDR         0x49
#define TMP116_RESULT_REG   0x00

int8_t get_temperature(I2C_Handle i2c_handle, float *temp_dest);

#endif /* DRIVERS_INC_TMP116_H_ */
