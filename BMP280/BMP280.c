/*
Library:				BMP280
Written by:				Pritham H suvarna
Description:			This is an STM32 device driver library for the BMP280 pressure sensor, using STM HAL libraries
*/
// List of header files 
#include<BMP280.h>
#include<stdio.h>

/*********Low level functions*******/
HAL_StatusTypeDef register_read(BMP280 * _BMP280, uint16_t MemAddress, uint8_t * pData){
    HAL_I2C_Mem_Read(_BMP280->i2cHandle, CHIP_ID, MemAddress, I2C_MEMADD_SIZE_8BIT, pData, 4, HAL_MAX_DELAY);// need to decide pData
}
HAL_StatusTypeDef register_write(BMP280 * _BMP280, uint16_t MemAddress, uint8_t * pData){
    HAL_I2C_Mem_write(_BMP280->i2cHandle, CHIP_ID, MemAddress, I2C_MEMADD_SIZE_8BIT, pData, 4, HAL_MAX_DELAY);// need to decide pData
}
/*********initialising  function*******/
bool BMP280_init(I2C_HandleTypeDef *pI2cHandle,){
    //writing to swtich to normal mode
    register_read(* _BMP280, )
}