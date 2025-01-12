/*
 * mpu6500.h
 *
 *  Created on: Nov 19, 2024
 *      Author: SinlessKid
 */

#ifndef SRC_MPU6500_H_
#define SRC_MPU6500_H_


#define MPU6500_I2C_ADDR (uint8_t)0x68

#include <stdint.h>
#include "stm32f4xx_hal.h"



typedef enum
{
	MPU6500_OK,
	MPU6500_ERR
}mpu6500_status_t;


typedef struct{
	int16_t x;
	int16_t y;
	int16_t z;
}mpu6500_accel_data_t;

typedef enum {
	DLPF_CFG_260HZ = 0,
	DLPF_CFG_184HZ = 1,
	DLPF_CFG_94HZ = 2,
	DLPF_CFG_44HZ = 3,
	DLPF_CFG_21HZ = 4,
	DLPF_CFG_10HZ = 5,
	DLPF_CFG_5HZ = 6,

}mpu6500_dlpf_config_t;




mpu6500_status_t mpu6500_configure_low_pass_filter \
(\
		I2C_HandleTypeDef *hi2c, mpu6500_dlpf_config_t dlpf \
);
mpu6500_status_t mpu6500_init(I2C_HandleTypeDef *hi2c, uint8_t i2c_dev_addr);
mpu6500_status_t  mpu6500_read_accelerometer_data(I2C_HandleTypeDef *hi2c, uint8_t i2c_dev_addr, mpu6500_accel_data_t *accel_data);
mpu6500_accel_data_t mpu6500_accelerometer_calibration(const mpu6500_accel_data_t *error_offset, mpu6500_accel_data_t *raw_data);


#define MPU6500_REG_WHOAMI (uint8_t)117
#define MPU6500_REG_PWMGMT_1 (uint8_t)107
#define MPU6500_REG_ACCEL_START (uint8_t)59
#define MPU6500_REG_CONFIG      (uint8_t)26
#define MPU6500_ACC_CNFG_REG   0x1C
#define MPU6500_GYRO_CNFG_REG 0x1B

#endif /* SRC_MPU6500_H_ */
