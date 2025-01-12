/*
 * mpu6500.c
 *
 *  Created on: Nov 19, 2024
 *      Author: SinlessKid
 */


#include "mpu6500.h"
#define I2C_TIMEOUT 500UL



mpu6500_status_t mpu6500_read_byte(I2C_HandleTypeDef *hi2c,uint8_t reg_addr, uint8_t *data)
{
	HAL_StatusTypeDef status = \
			HAL_I2C_Mem_Read(hi2c, MPU6500_I2C_ADDR<<1, reg_addr, 1, data, 1, I2C_TIMEOUT);


	return (status == HAL_OK) ? MPU6500_OK : MPU6500_ERR;
}


mpu6500_status_t mpu6500_read(I2C_HandleTypeDef *hi2c,uint8_t reg_base_addr, uint8_t *buffer, uint32_t nbytes)
{
	HAL_StatusTypeDef status = \
			HAL_I2C_Mem_Read(hi2c, MPU6500_I2C_ADDR<<1, reg_base_addr, 1, buffer, nbytes, HAL_MAX_DELAY);


	return (status == HAL_OK) ? MPU6500_OK : MPU6500_ERR;
}


mpu6500_status_t mpu6500_write_byte(I2C_HandleTypeDef *hi2c,uint8_t reg_addr, uint8_t data)
{
	HAL_StatusTypeDef status = \
			HAL_I2C_Mem_Write(hi2c, MPU6500_I2C_ADDR<<1, reg_addr, 1, &data, 1, I2C_TIMEOUT);


	return (status == HAL_OK) ? MPU6500_OK : MPU6500_ERR;
}


static uint8_t mpu6500_i2c_addr;

mpu6500_status_t mpu6500_init(I2C_HandleTypeDef *hi2c, uint8_t i2c_dev_addr)
{

	mpu6500_i2c_addr = i2c_dev_addr;
	uint8_t read_byte = 0;

	if(mpu6500_read_byte(hi2c, MPU6500_REG_WHOAMI, &read_byte) != MPU6500_OK)
	{
		return MPU6500_ERR;
	}


	if(read_byte == 0x68 || read_byte == 0x70)
	{
		printf("Valid mpu6500 sensor found ");
	}
	else
	{
		printf("Valid mpu6500 sensor not found ");
		return MPU6500_ERR;
	}
	uint8_t data = 0x00;
	if(mpu6500_write_byte(hi2c, MPU6500_REG_PWMGMT_1, data ) !=  MPU6500_OK)
	{
		return MPU6500_ERR;
	}

	data = 0x08;
	if(mpu6500_write_byte(hi2c, MPU6500_GYRO_CNFG_REG, data) != MPU6500_OK)
	{
		return MPU6500_ERR;
	}

	data = 0x10;
	if(mpu6500_write_byte(hi2c, MPU6500_ACC_CNFG_REG, data) != MPU6500_OK)
		{
			return MPU6500_ERR;
		}


		return MPU6500_OK;

}



mpu6500_status_t  mpu6500_read_accelerometer_data(I2C_HandleTypeDef *hi2c, uint8_t i2c_dev_addr, mpu6500_accel_data_t *accel_data)
{

	uint8_t raw_data[6];
	mpu6500_status_t status = mpu6500_read(hi2c, MPU6500_REG_ACCEL_START,raw_data, sizeof(raw_data));

	if(status != MPU6500_OK)
	{
		return status;
	}

	accel_data->x = (int16_t)(raw_data[0]<< 8 | raw_data[1]);
	accel_data->y = (int16_t)(raw_data[2]<< 8 | raw_data[3]);
	accel_data->z = (int16_t)(raw_data[4]<< 8 | raw_data[5]);



	return MPU6500_OK;

}




mpu6500_accel_data_t mpu6500_accelerometer_calibration(const mpu6500_accel_data_t *error_offset, mpu6500_accel_data_t *raw_data)
{
	mpu6500_accel_data_t accel_calibrated;
	accel_calibrated.x = raw_data->x - error_offset->x;
	accel_calibrated.y = raw_data->y - error_offset->y;
	accel_calibrated.z = raw_data->z - error_offset->z;

	return accel_calibrated;
}




mpu6500_status_t mpu6500_configure_low_pass_filter \
(\
		I2C_HandleTypeDef *hi2c, mpu6500_dlpf_config_t dlpf \
)
{

	uint8_t value = 0;

	if (mpu6500_read_byte(hi2c, MPU6500_REG_CONFIG, &value) != MPU6500_OK){
		return MPU6500_ERR;
	}

	value &= ~(0x7);
	value |= (uint8_t)dlpf;
	if (mpu6500_write_byte(hi2c, MPU6500_REG_CONFIG, value) != MPU6500_OK){
		return MPU6500_ERR;
	}

	return MPU6500_OK;
}










