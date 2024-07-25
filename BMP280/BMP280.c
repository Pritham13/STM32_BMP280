
/*
Library:				BMP280
Written by:				Pritham H suvarna
Description:			This is an STM32 device driver library for the BMP280 pressure sensor, using STM HAL libraries
*/
// List of header files 
#include"BMP280.h"
#include<stdio.h>

/* Checking status  */

void BMP280CheckStatus(I2C_HandleTypeDef &hi2c){
  if(HAL_I2C_IsDeviceReady(&hi2c,CHIP_ID,2,10) == HAL_OK)
    printf("BMP280 found\r\n");
  else 
    printf("BMP280 not found\r\n");
}

/**
/*********Low level functions*******/
uint8_t register_read_8(I2C_HandleTypeDef &hi2c, uint16_t MemAddress){
  uint8_t pdata;
  HAL_I2C_Mem_Read(&hi2c, CHIP_ID, MemAddress, 8, &pData, 4, HAL_MAX_DELAY);// need to decide pData
  return pdata;
}
void register_write_8(I2C_HandleTypeDef &hi2c, uint16_t MemAddress, uint8_t * pData){
  HAL_I2C_Mem_write(&hi2c, CHIP_ID, MemAddress,8, pData, 4, HAL_MAX_DELAY);
}
uint16_t register_read_16(I2C_HandleTypeDef &hi2c, uint16_t MemAddress){
  uint16_t pdata;
  HAL_I2C_Mem_Read(&hi2c, CHIP_ID, MemAddress, 16, &pData, 4, HAL_MAX_DELAY);// need to decide pData
  return pdata;
}
// TODO: have to see if its required
void register_write_16(I2C_HandleTypeDef &hi2c, uint16_t MemAddress, uint16_t * pData){
  HAL_I2C_Mem_write(&hi2c, CHIP_ID, MemAddress, 16, pData, 4, HAL_MAX_DELAY);
}

uint16_t register_read_16_LE(I2C_HandleTypeDef &hi2c, uint16_t MemAddress){
  uint16_t temp = register_read_16(hi2c,MemAddress);
  return (temp >> 8) | (temp << 8);
}

uint32_t register_read_24 (I2C_HandleTypeDef &hi2c, uint16_t MemAddress){
  uint8_t temp[2];
  temp[0] = register_read_8(hi2c,MemAddress);
  temp[1] = register_read_8(hi2c,(MemAddress+1);
  temp[2] = register_read_8(hi2c,(MemAddress+2);
  return ((temp[0]<<16)|(temp[1]<<8)|temp[2]);
} 

/*********initialising  function*******/
void BMP280_init(I2C_HandleTypeDef *pI2cHandle,){
    /* writing data to the control register */
  uint8_t data=(MODE_FORCED|(SAMPLING_X1<<2)|(SAMPLING_X1<<5));
  register_write_8(I2C_HandleTypeDef &hi2c,CONTROL_REG,&data);
  /*Writing data into the config register */
  data =((STANDBY_1_MS<<5)|(FILTER_OFF<<2)|(SPI_ENABLE));
  register_write_8(I2C_HandleTypeDef &hi2c,CONFIG_REG,&data);
}
void BMP280_Read_Calib_Data(I2C_HandleTypeDef &hi2c){
  bmp280_calib_data.dig_T1 = register_read_16_LE(hi2c,DIG_T1_REG);
  bmp280_calib_data.dig_T2 = register_read_16_LE(hi2c,DIG_T2_REG);
  bmp280_calib_data.dig_T3 = register_read_16_LE(hi2c,DIG_T3_REG);

  bmp280_calib_data.dig_P1 = register_read_16_LE(hi2c,DIG_P1_REG);
  bmp280_calib_data.dig_P2 = register_read_16_LE(hi2c,DIG_P2_REG);
  bmp280_calib_data.dig_P3 = register_read_16_LE(hi2c,DIG_P3_REG);
  bmp280_calib_data.dig_P4 = register_read_16_LE(hi2c,DIG_P4_REG);
  bmp280_calib_data.dig_P5 = register_read_16_LE(hi2c,DIG_P5_REG);
  bmp280_calib_data.dig_P6 = register_read_16_LE(hi2c,DIG_P6_REG);
  bmp280_calib_data.dig_P7 = register_read_16_LE(hi2c,DIG_P7_REG);
  bmp280_calib_data.dig_P8 = register_read_16_LE(hi2c,DIG_P8_REG);
  bmp280_calib_data.dig_P9 = register_read_16_LE(hi2c,DIG_P9_REG);
  }

