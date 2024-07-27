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

uint8_t register_read_8(I2C_HandleTypeDef &hi2c, uint16_t MemAddress){
  uint8_t pdata;
  HAL_I2C_Mem_Read(&hi2c, CHIP_ID, MemAddress, 8, &pData, 4, HAL_MAX_DELAY);// need to decide pData
  return pdata;
}
void register_write_8(I2C_HandleTypeDef &hi2c, uint16_t MemAddress, uint8_t * pData){
  HAL_I2C_Mem_write(&hi2c, CHIP_ID, MemAddress,8, pData, 4, HAL_MAX_DELAY);
}
uint16_t register_read_16(I2C_HandleTypeDef &hi2c, uint16_t MemAddress){
  uint8_t msb,lsb;
  HAL_I2C_Mem_Read(&hi2c, CHIP_ID, MemAddress, 8, &msb, 4, HAL_MAX_DELAY);
  HAL_I2C_Mem_Read(&hi2c, CHIP_ID, MemAddress+1, 8, &lsb, 4, HAL_MAX_DELAY);
  return ((msb<<8)|lsb);
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
float BMP280_read_Temperature(I2C_HandleTypeDef &i2c){
  int32_t var1, var2;

  int32_t adc_T = register_read_24(TEMP_MSB_REG);
  adc_T >>= 4;

  var1 = ((((adc_T >> 3) - ((int32_t)_bmp280_calib.dig_T1 << 1))) *
          ((int32_t)_bmp280_calib.dig_T2)) >>
         11;

  var2 = (((((adc_T >> 4) - ((int32_t)_bmp280_calib.dig_T1)) *
            ((adc_T >> 4) - ((int32_t)_bmp280_calib.dig_T1))) >>
           12) *
          ((int32_t)_bmp280_calib.dig_T3)) >>
         14;

  t_fine = var1 + var2;

  float T = (t_fine * 5 + 128) >> 8;
  return T / 100;
}
float BMP280_read_Pressure()
{
  int64_t var1, var2, p;

  // Must be done first to get the t_fine variable set up
  BMP280_read_Temperature();

  int32_t adc_P = register_read_24(PRESSURE_MSB_REG);
  adc_P >>= 4;

  var1 = ((int64_t)t_fine) - 128000;
  var2 = var1 * var1 * (int64_t)_bmp280_calib.dig_P6;
  var2 = var2 + ((var1 * (int64_t)_bmp280_calib.dig_P5) << 17);
  var2 = var2 + (((int64_t)_bmp280_calib.dig_P4) << 35);
  var1 = ((var1 * var1 * (int64_t)_bmp280_calib.dig_P3) >> 8) +
         ((var1 * (int64_t)_bmp280_calib.dig_P2) << 12);
  var1 =
      (((((int64_t)1) << 47) + var1)) * ((int64_t)_bmp280_calib.dig_P1) >> 33;

  if (var1 == 0) {
    return 0; // avoid exception caused by division by zero
  }
  p = 1048576 - adc_P;
  p = (((p << 31) - var2) * 3125) / var1;
  var1 = (((int64_t)_bmp280_calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
  var2 = (((int64_t)_bmp280_calib.dig_P8) * p) >> 19;

  p = ((p + var1 + var2) >> 8) + (((int64_t)_bmp280_calib.dig_P7) << 4);
  return (float)p / 256;
}


