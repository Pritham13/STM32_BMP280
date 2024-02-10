#ifndef __BMP280_H__
#define __BMP280_H__

/*includes*/
#include <cstdint>
#include <stdbool.h>
#include "stm32f4xx_hal.h"
/*Identification*/
#define CHIP_ID (0x58<<1)
/* memory map*/
#define TEMP_XLSB_REG 0xFC
#define TEMP_LSB_REG 0xFB
#define TEMP_MSB_REG 0xFA
#define PRESSURE_XLSB_REG 0xF9
#define PRESSURE_LSB_REG 0xF8
#define PRESSURE_MSB_REG 0xF7
#define CONFIG_REG 0xF5
#define CONTROL_REG 0xF4 //bit 1,0 ->mode[2:0] //Bit 4, 3, 2 ->osrs_p[2:0]//Bit 7, 6, 5 ->osrs_t[2:0]
#define STATUS_REG 0xE0
#define RESET_REG 0xE0
#define  DEVICE_ID_REG 0xD0
//register locations for the calibration data
#define DIG_T1_REG 0x88
#define DIG_T2_REG 0x8A
#define DIG_T3_REG 0x8C
#define DIG_P1_REG 0x8E
#define DIG_P2_REG 0x90
#define DIG_P3_REG 0x92
#define DIG_P4_REG 0x94
#define DIG_P5_REG 0x96
#define DIG_P6_REG 0x98
#define DIG_P7_REG 0x9A
#define DIG_P8_REG 0x9C
#define DIG_P9_REG 0x9E

////////////////structures//////////////////////
/*
 *  Struct to hold calibration data.
 */
/*Sensor structure*/
typredef struct {
    /* I2C handle*/
    I2C_HandleTypeDef *i2cHandle;
    /*processed temperature*/
    float Temp_c;
    /*processed pressure*/
    float pressure_pa;
}BMP280;
typedef struct {
  uint16_t dig_T1; /**< dig_T1 cal register. */
  int16_t dig_T2;  /**<  dig_T2 cal register. */
  int16_t dig_T3;  /**< dig_T3 cal register. */

  uint16_t dig_P1; /**< dig_P1 cal register. */
  int16_t dig_P2;  /**< dig_P2 cal register. */
  int16_t dig_P3;  /**< dig_P3 cal register. */
  int16_t dig_P4;  /**< dig_P4 cal register. */
  int16_t dig_P5;  /**< dig_P5 cal register. */
  int16_t dig_P6;  /**< dig_P6 cal register. */
  int16_t dig_P7;  /**< dig_P7 cal register. */
  int16_t dig_P8;  /**< dig_P8 cal register. */
  int16_t dig_P9;  /**< dig_P9 cal register. */
} bmp280_calib_data;
//////////////enum////////////////////
/** Operating mode for the sensor. */
enum sensor_mode {
  /** Sleep mode. */
  MODE_SLEEP = 0x00,
  /** Forced mode. */
  MODE_FORCED = 0x01,
  /** Normal mode. */
  MODE_NORMAL = 0x03,
  /** Software reset. */
  MODE_SOFT_RESET_CODE = 0xB6
};
enum sensor_oversampling{
      /** No over-sampling. */
    SAMPLING_NONE = 0x00,
    /** 1x over-sampling. */
    SAMPLING_X1 = 0x01,
    /** 2x over-sampling. */
    SAMPLING_X2 = 0x02,
    /** 4x over-sampling. */
    SAMPLING_X4 = 0x03,
    /** 8x over-sampling. */
    SAMPLING_X8 = 0x04,
    /** 16x over-sampling. */
    SAMPLING_X16 = 0x05
}
enum sensor_filter {
  /** No filtering. */
  FILTER_OFF = 0x00,
  /** 2x filtering. */
  FILTER_X2 = 0x01,
  /** 4x filtering. */
  FILTER_X4 = 0x02,
  /** 8x filtering. */
  FILTER_X8 = 0x03,
  /** 16x filtering. */
  FILTER_X16 = 0x04
};
enum standby_duration{
  /** 0.5ms **/
  STANDBY_1_MS = 0x00,
  /** 62.5ms **/
  STANDBY_63_MS = 0x01,
  /** 125ms **/
  STANDBY_125_MS = 0x02,
  /** 250ms **/
  STANDBY_250_MS = 0x03,
  /** 500ms **/
  STANDBY_500_MS = 0x04,
  /** 1000 ms **/
  STANDBY_1000_MS = 0x05,
  /** 2000ms **/
  STANDBY_2000_MS = 0x06,
  /** 4000ms **/
  STANDBY_4000_MS = 0x07,
};

/*Funtion Prototypes*/
bool BMP280_init(I2C_HandleTypeDef *pI2cHandle);
void reset(void);
/**************DATA AQUISITION FUNCTIONS*******************/
float read_temperature(void);
float read_pressure(void);
/**************LOW LEVEL FUNCTIONS*******************/
HAL_StatusTypeDef register_write(I2C_HandleTypeDef * hi2c, uint16_t DevAddress, uint16_t MemAddress, uint8_t * pData);
HAL_StatusTypeDef register_read(I2C_HandleTypeDef * hi2c, uint16_t DevAddress, uint16_t MemAddress, uint8_t * pData);
/*I2C_HandleTypeDef * hi2c: A pointer to a I2C_HandleTypeDef structure that contains the configuration information for the specified I2C.
uint16_t DevAddress: The device address. The device 7 bits address value in the datasheet must be shifted to the left before calling the interface.
uint16_t MemAddress: The memory address to read from.
uint16_t MemAddSize: The size of the memory address.
uint8_t * pData: A pointer to the buffer that will store the read data.
uint16_t Size: The number of bytes to read.*/
/*******struct declarations******/
BMP280 _BMP280;
bmp280_calib_data _bmp280_calib_data;
#endif /*__BMP280_H__*/