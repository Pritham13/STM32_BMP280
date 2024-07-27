# BMP280 Driver

This repository contains the driver code for interfacing with the BMP280 sensor using I2C communication. The BMP280 is a barometric pressure sensor that can measure temperature and pressure.

## Files

- `BMP280.h` - Header file containing function declarations and necessary macros.
- `BMP280.c` - Source file containing function definitions for interacting with the BMP280 sensor.

## Functions

### BMP280CheckStatus

```c
void BMP280CheckStatus(I2C_HandleTypeDef &hi2c);
```

Checks if the BMP280 sensor is connected and ready for communication.

- **Parameters**: `hi2c` - I2C handle.
- **Returns**: Prints "BMP280 found" if the sensor is ready, otherwise prints "BMP280 not found".

### register_read_8

```c
uint8_t register_read_8(I2C_HandleTypeDef &hi2c, uint16_t MemAddress);
```

Reads an 8-bit value from the specified register.

- **Parameters**: 
  - `hi2c` - I2C handle.
  - `MemAddress` - Memory address to read from.
- **Returns**: The 8-bit value read from the specified register.

### register_write_8

```c
void register_write_8(I2C_HandleTypeDef &hi2c, uint16_t MemAddress, uint8_t * pData);
```

Writes an 8-bit value to the specified register.

- **Parameters**: 
  - `hi2c` - I2C handle.
  - `MemAddress` - Memory address to write to.
  - `pData` - Pointer to the data to be written.

### register_read_16

```c
uint16_t register_read_16(I2C_HandleTypeDef &hi2c, uint16_t MemAddress);
```

Reads a 16-bit value from the specified register.

- **Parameters**: 
  - `hi2c` - I2C handle.
  - `MemAddress` - Memory address to read from.
- **Returns**: The 16-bit value read from the specified register.

### register_read_16_LE

```c
uint16_t register_read_16_LE(I2C_HandleTypeDef &hi2c, uint16_t MemAddress);
```

Reads a 16-bit value from the specified register and converts it to little-endian format.

- **Parameters**: 
  - `hi2c` - I2C handle.
  - `MemAddress` - Memory address to read from.
- **Returns**: The 16-bit value in little-endian format.

### register_read_24

```c
uint32_t register_read_24 (I2C_HandleTypeDef &hi2c, uint16_t MemAddress);
```

Reads a 24-bit value from the specified register.

- **Parameters**: 
  - `hi2c` - I2C handle.
  - `MemAddress` - Memory address to read from.
- **Returns**: The 24-bit value read from the specified register.

### BMP280_init

```c
void BMP280_init(I2C_HandleTypeDef *pI2cHandle);
```

Initializes the BMP280 sensor by writing to the control and config registers.

- **Parameters**: `pI2cHandle` - Pointer to the I2C handle.

### BMP280_Read_Calib_Data

```c
void BMP280_Read_Calib_Data(I2C_HandleTypeDef &hi2c);
```

Reads calibration data from the BMP280 sensor.

- **Parameters**: `hi2c` - I2C handle.

### BMP280_read_Temperature

```c
float BMP280_read_Temperature(I2C_HandleTypeDef &i2c);
```

Reads and calculates the temperature from the BMP280 sensor.

- **Parameters**: `i2c` - I2C handle.
- **Returns**: The temperature in degrees Celsius.

### BMP280_read_Pressure

```c
float BMP280_read_Pressure();
```

Reads and calculates the pressure from the BMP280 sensor.

- **Returns**: The pressure in Pascals.

## Usage

1. Initialize the I2C handle.
2. Call `BMP280CheckStatus` to ensure the BMP280 sensor is connected.
3. Use `BMP280_init` to initialize the sensor.
4. Call `BMP280_Read_Calib_Data` to read the sensor's calibration data.
5. Use `BMP280_read_Temperature` and `BMP280_read_Pressure` to read temperature and pressure values, respectively.

## License

This project is licensed under the MIT License.

---

Feel free to contribute to this repository by creating pull requests or reporting issues. For any questions, please open an issue or contact the repository owner.
