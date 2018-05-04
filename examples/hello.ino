
// Arduino I2C variant
#include <MPU925x.hpp>
MPU925x mpu(Wire);

// Arduino SPI variant
#include <MPU925x.hpp>
MPU925x mpu(SPI);



//Custom lib I2C variant
#include <I2C_atmega328.hpp>
#include <MPU925x.hpp>
m328::I2C i2c;
MPU925x mpu(i2c);

//Custom lib SPI variant
#include <SPI_atmega328.hpp>
#include <MPU925x.hpp>
m328::SPI spi;
MPU925x mpu(spi);



nWI i2c_ino(Wire);
MPU925x mpu(i2c_ino);



// Ino
#include <MPU925x.hpp>
MPU925x mpu(i2c);

BMP280 bmp(i2c);


// Custom lib
#include <I2C_atmega328.hpp>
m328::I2C i2c_mpu;
MPU925x mpu(i2c_mpu);

m328::I2C i2c_bmp;
BMP280 bmp(i2c_bmp);



// Custom lib
#include <SPI_atmega328.hpp>
m328::SPI spi_mpu;
MPU925x mpu(spi_mpu);



// ----------------------
// Arduino
#include <MPU925x.hpp>
MPU925x mpu(SPI, 7);

#include <MPU925x.hpp>
MPU925x mpu(Wire, 0x78);


// Custom library
#include <MPU925x.hpp>

#include <SPI_atmega328.hpp>
m328::SPI spi;
MPU925x mpu(spi, PORTB, PINB3);

#include <I2C_atmega328.hpp>
m328::I2C i2c;
MPU925x mpu(i2c, 0x78);
