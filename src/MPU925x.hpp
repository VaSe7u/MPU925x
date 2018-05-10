/**
The MIT License (MIT)

Copyright (c) 2016 Vasil Kalchev

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.


@file       MPU925x.hpp
@author     Vasil Kalchev
@date       2018
@version    0.1.0
@copyright  The MIT License
@brief      MPU925x - I2C and SPI library.

@todo:
*/

#pragma once
#include <stdint.h>

// TODO: SPI compatible, configure mag and slaves

/*
Single operation:
  Write:
  I2C_SLV0_ADDR - i2c address of slave, bit7 - 1read 0write
  I2C_SLV0_REG - register to r/w
  I2C_SLV0_DO - packet to r/w
  I2C_SLV0_CTRL - enable I2C (I2C_SLV0_EN)

  Read:
  I2C_SLV0_ADDR - i2c address of slave or'ed with I2C_READ_FLAG
  I2C_SLV0_REG - register to r/w
  I2C_SLV0_CTRL - enable I2C (I2C_SLV0_EN) or'ed with number of bytes
  wait for 1ms
  EXT_SENS_DATA_00 - get the data

Configure continuous mode:
  Set I2C_SLV0_ADDR to device address
  Set I2C_SLV0_REG to the first data register
  Set I2C_SLV0_CTRL[I2C_SLV0_EN] to 1 to enable continuous reading
  Set I2C_SLV0_CTRL[I2C_SLV0_LENG(3:0)] to the number of bytes to be read
*/

/*
Gyroscope DLPF
==============
| Fchoice 1:0 | DLPF_CFG | Bandwidth, Hz | Delay, ms | Fs, kHz |
|-------------|----------|---------------|-----------|---------|
|     x 0     |    x     |     8800      |   0.064   |   32    |
|     0 1     |    x     |     3600      |    0.11   |   32    |
|     1 1     |    0     |      250      |    0.97   |    8    |
|     1 1     |    1     |      184      |     2.9   |    1    |
|     1 1     |    2     |       92      |     3.9   |    1    |
|     1 1     |    3     |       41      |     5.9   |    1    |
|     1 1     |    4     |       20      |     9.9   |    1    |
|     1 1     |    5     |       10      |   17.85   |    1    |
|     1 1     |    6     |        5      |   33.48   |    1    |
|     1 1     |    7     |     3600      |    0.17   |    8    |

Accelerometer DLPF for MPU9255
==============================
| Fchoice | DLPF_CFG | Bandwidth, Hz | Delay, ms | Fs, kHz |
|---------|----------|---------------|-----------|---------|
|    0    |     x    |     1.13k     |    0.75   |    4    |
|    1    |     0    |       460     |    1.94   |    1    |
|    1    |     1    |       184     |     5.8   |    1    |
|    1    |     2    |        92     |     7.8   |    1    |
|    1    |     3    |        41     |    11.8   |    1    |
|    1    |     4    |        20     |    19.8   |    1    |
|    1    |     5    |        10     |    35.7   |    1    |
|    1    |     6    |         5     |   66.96   |    1    |
|    1    |     7    |       420     |    1.94   |    1    |
*/

#define MPU925x_ADDRESS_AD0_LOW         0x68 // address pin low (GND)
#define MPU925x_ADDRESS_AD0_HIGH        0x69 // address pin high (VCC)

#define MPU9250_ID                      0x71
#define MPU9255_ID                      0x73


class MPU925x {
public:
  // Initialization.
  // ===============
  enum class AD0 : bool {
    LOW = 0x68, HIGH = 0x69,
  };
  explicit MPU925x(const uint8_t device = 0x68);
  explicit MPU925x(const AD0 ad0);
  void attachInterface(bool (*read)(uint8_t device, uint8_t registerAddress, uint8_t *data, int8_t size),
                       bool (*write)(uint8_t device, uint8_t registerAddress, uint8_t *data, int8_t size));

  void initialize(); // 100ms after power-up.
  void reset();
  int8_t getDeviceId() const;
  bool checkConnection() const;

  void sleepMode(const bool enable);
  bool getSleepMode() const;

  enum ClockSource : int8_t {
    CLOCK_INTERNAL = 0,
    CLOCK_20MHz = 0,
    CLOCK_PLL = 1,
    CLOCK_DISABLE = 7,
  };
  void clockSource(const ClockSource clockSource);
  ClockSource getClockSource() const;

  bool selfTestAccel() const;
  bool seltTestGyro() const;
  // ===============


  // Basic configuration.
  // ====================
  enum Bandwidth : int8_t {
    BANDWIDTH_8800Hz = -2, // gyro only
    BANDWIDTH_3600Hz = -1, // gyro only
    BANDWIDTH_1130Hz = -1, // accel only
    BANDWIDTH_460Hz = 0, // accel only
    BANDWIDTH_250Hz = 0, // gyro only
    BANDWIDTH_184Hz = 1,
    BANDWIDTH_92Hz = 2,
    BANDWIDTH_41Hz = 3,
    BANDWIDTH_20Hz = 4,
    BANDWIDTH_10Hz = 5,
    BANDWIDTH_5Hz = 6,
  };
  bool dlpf(const Bandwidth bandwidth);
  bool accelDlpf(const Bandwidth bandwidth);
  Bandwidth getAccelDlpf() const;
  bool gyroDlpf(const Bandwidth bandwidth);
  Bandwidth getGyroDlpf() const;

  void outputDataRateDivider(const int16_t outputDataRateDivider);
  int16_t getOutputDataRateDivider() const;
  bool outputDataRate(const int16_t outputDataRate);
  int16_t getOutputDataRate() const;

  enum Range : int8_t {
    RANGE_2G = 0b00,
    RANGE_4G = 0b01,
    RANGE_8G = 0b10,
    RANGE_16G = 0b11,
    RANGE_250DPS = 0b00,
    RANGE_500DPS = 0b01,
    RANGE_1000DPS = 0b10,
    RANGE_2000DPS = 0b11,
  };
  bool accelRange(const Range range);
  Range getAccelRange() const;
  bool gyroRange(const Range range);
  Range getGyroRange() const;

  bool offsetAccel(const int16_t x, const int16_t y, const int16_t z);
  bool getOffsetAccel(int16_t &x, int16_t &y, int16_t &z) const;
  bool offsetAccelX(const int16_t xOffset);
  bool getOffsetAccelX(int16_t &xOffset) const;
  bool offsetAccelY(const int16_t yOffset);
  bool getOffsetAccelY(int16_t &yOffset) const;
  bool offsetAccelZ(const int16_t zOffset);
  bool getOffsetAccelZ(int16_t &zOffset) const;

  bool offsetGyro(const int16_t x, const int16_t y, const int16_t z);
  bool getOffsetGyro(int16_t &x, int16_t &y, int16_t &z) const;
  bool offsetGyroX(const int16_t xOffset);
  bool getOffsetGyroX(int16_t &xOffset) const;
  bool offsetGyroY(const int16_t yOffset);
  bool getOffsetGyroY(int16_t &yOffset) const;
  bool offsetGyroZ(const int16_t zOffset);
  bool getOffsetGyroZ(int16_t &zOffset) const;

  bool autoOffsetAccel(int16_t &x, int16_t &y, int16_t &z);
  bool autoOffsetGyro(int16_t &x, int16_t &y, int16_t &z);
  // ====================

  // Advanced configuration.
  // =======================
  void i2cMaster(const bool enable); //106,
  void resetI2cMaster();
  void disableI2c(); // Should be called first when using SPI

  enum Clock : int8_t {
    CLOCK_258kHz = 8,
    CLOCK_267kHz = 7,
    CLOCK_276kHz = 6,
    CLOCK_286kHz = 5,
    CLOCK_296kHz = 4,
    CLOCK_308kHz = 3,
    CLOCK_320kHz = 2,
    CLOCK_333kHz = 1,
    CLOCK_348kHz = 0,
    CLOCK_364kHz = 15,
    CLOCK_381kHz = 14,
    CLOCK_400kHz = 13,
    CLOCK_421kHz = 12,
    CLOCK_444kHz = 11,
    CLOCK_471kHz = 10,
    CLOCK_500kHz = 9,
  };
  bool masterI2cClock(Clock clock);


  void fifo(const bool enable);
  void resetFifo();
  void fifoFullOverflow(const bool enable);
  void fifoTemperature(const bool enable);
  void fifoGyroX(const bool enable);
  void fifoGyroY(const bool enable);
  void fifoGyroZ(const bool enable);
  void fifoGyro(const bool enable);
  void fifoAccel(const bool enable);
  void fifoMag(const bool enable);
  void fifoSlave0(const bool enable);
  void fifoSlave1(const bool enable);
  void fifoSlave2(const bool enable);
  void fifoSlave3(const bool enable);

  uint16_t fifoSize() const;
  bool readFifo(uint8_t fifo[], const int8_t size);


  enum OutputDataRate : int8_t {
    ODR_0p24Hz = 0,
    ODR_0p49Hz = 1,
    ODR_0p98Hz = 2,
    ODR_1p95Hz = 3,
    ODR_3p91Hz = 4,
    ODR_7p81Hz = 5,
    ODR_15p63Hz = 6,
    ODR_31p25Hz = 7,
    ODR_62p60Hz = 8,
    ODR_125Hz = 9,
    ODR_250Hz = 10,
    ODR_500Hz = 11,
  };
  /* The MPU-9255 can be put into Accelerometer Only Low Power Mode using the following steps:
  (i) Set CYCLE bit to 1
  (ii) Set SLEEP bit to 0
  (iii) Set TEMP_DIS bit to 1
  (iv) Set DIS_XG, DIS_YG, DIS_ZG bits to 1
  */
  bool lowPowerAccelOutputDataRate(const OutputDataRate outputDataRate);
  bool lowPowerAccelMode(const bool enable);
  void cycle(const bool enable);
  void standbyGyro(const bool enable);
  void powerDownPtat(const bool enable);

  void sensorAccelX(const bool enable);
  void sensorAccelY(const bool enable);
  void sensorAccelZ(const bool enable);
  void sensorAccel(const bool state);
  void sensorGyroX(const bool enable);
  void sensorGyroY(const bool enable);
  void sensorGyroZ(const bool enable);
  void sensorGyro(const bool state);


  void wakeOnMotionInterrupt(const bool enable);
  bool wakeOnMotionTriggered() const;
  void wakeOnMotionDetection(const bool enable);
  void wakeOnMotionThreshold(const uint16_t mg);

  void fifoOverflowInterrupt(const bool enable);
  bool fifoOverflowTriggered() const;

  void dataReadyInterrupt(const bool enable);
  bool dataReadyTriggered() const;
  void waitForExternalDataBeforeTriggering(const bool enable);
  void delayShadowingOfExternalData(const bool enable);

  void interruptPinActiveLogicLevel(const bool high);
  void openDrainInterruptPin();
  void pushPullInterruptPin();
  void latchInterruptPin(const bool enable);
  void anyReadClearsInterruptStatus(const bool enable);
  void interruptStatus(bool & wakeOnMotion, bool & fifoOverflow, bool & dataReady) const;


  void resetAccelDigitalSignalPath();
  void resetGyroDigitalSignalPath();
  void resetTempDigitalSignalPath();
  void resetAllDigitalSignalPaths();

  // =======================

  // Data.
  // =====
  // Single sensor.
  // --------------
  bool readAccel(int16_t &x, int16_t &y, int16_t &z) const;
  bool readAccel(int16_t (&accel)[3]) const;
  bool readAccelX(int16_t &x) const;
  bool readAccelY(int16_t &y) const;
  bool readAccelZ(int16_t &z) const;

  bool readGyro(int16_t &x, int16_t &y, int16_t &z) const;
  bool readGyro(int16_t (&gyro)[3]) const;
  bool readGyroX(int16_t &x) const;
  bool readGyroY(int16_t &y) const;
  bool readGyroZ(int16_t &z) const;

  bool readMag(int16_t &x, int16_t &y, int16_t &z) const;
  bool readMag(int16_t (&mag)[3]) const;
  bool readMagX(int16_t &x) const;
  bool readMagY(int16_t &y) const;
  bool readMagZ(int16_t &z) const;

  bool readTemp(int16_t &temp) const;

  bool readExternalData(const int8_t startingAddress, uint8_t externalData[],
                        const int8_t size) const; //max size 24
  bool readExternalData(const int8_t startingAddress, int8_t externalData[],
                        const int8_t size) const; //max size 24
  bool readExternalData(const int8_t startingAddress, uint16_t externalData[],
                        const int8_t size) const; //max size 12
  bool readExternalData(const int8_t startingAddress, int16_t externalData[],
                        const int8_t size) const; //max size 12
  // --------------

  // Multiple sensors.
  // -----------------
  bool readAccelGyro(int16_t &ax, int16_t &ay, int16_t &az,
                     int16_t &gx, int16_t &gy, int16_t &gz) const;
  bool readAccelGyro(int16_t (&accel)[3], int16_t (&gyro)[3]) const;
  bool readAccelGyro(int16_t (&accelGyro)[6]) const;

  bool readAccelGyroTemp(int16_t &ax, int16_t &ay, int16_t &az,
                         int16_t &gx, int16_t &gy, int16_t &gz,
                         int16_t &temp) const;
  bool readAccelGyroTemp(int16_t (&accel)[3], int16_t (&gyro)[3], int16_t &temp) const;
  bool readAccelGyroTemp(int16_t (&accelGyroTemp)[7]) const;

  bool readAccelGyroMag(int16_t &ax, int16_t &ay, int16_t &az,
                        int16_t &gx, int16_t &gy, int16_t &gz,
                        int16_t &mx, int16_t &my, int16_t &mz) const;
  bool readAccelGyroMag(int16_t (&accel)[3], int16_t (&gyro)[3], int16_t (&mag)[3]) const;
  bool readAccelGyroMag(int16_t (&accelGyroMag)[9]) const;

  bool readAccelGyroMagTemp(int16_t &ax, int16_t &ay, int16_t &az,
                            int16_t &gx, int16_t &gy, int16_t &gz,
                            int16_t &mx, int16_t &my, int16_t &mz,
                            int16_t &temp) const;
  bool readAccelGyroMagTemp(int16_t (&accel)[3], int16_t (&gyro)[3], int16_t (&mag)[3],
                            int16_t &temp) const;
  bool readAccelGyroMagTemp(int16_t (&accelGyroMagTemp)[10]) const;
  // -----------------

  // Raw.
  // ----
  bool read(const int8_t startingAddress, uint8_t data[],
            const int8_t size) const; //max size 38
  bool read(const int8_t startingAddress, int16_t data[],
            const int8_t size) const; //max size 19
  // ----
  // =====

  // Convert functions.
  // ==================
  float accel_to_g(const int16_t accel) const;
  float accel_to_ms2(const int16_t accel) const;

  float gyro_to_radS(const int16_t gyro) const;
  float gyro_to_degS(const int16_t gyro) const;

  float temp_to_c(const int16_t temp) const;
  float temp_to_f(const int16_t temp) const;
  // ==================

private:
  uint8_t _device = 0;

  uint16_t _accelSensitivity_g = 16384;
  float _accelSensitivity_mS2 = (float)_accelSensitivity_g / 9.80665f;
  float _gyroSensitivity_degS = 131.0f;
  float _gyroSensitivity_radS = _gyroSensitivity_degS / 0.0174533f;

  bool (*_read)(uint8_t device, uint8_t registerAddress, uint8_t *data, int8_t size) = nullptr;
  bool (*_write)(uint8_t device, uint8_t registerAddress, uint8_t *data, int8_t size) = nullptr;

  static bool _offsetToRegister(const int16_t offset, uint8_t &registerBytes[2]) const;
  static bool _registerToOffset(const uint8_t *const registerBytes, int16_t &offset) const;


  static bool readBit(const Register registerAddress, bool *const bit,
                      const Bit position) const;
  static bool writeBit(const Register registerAddress, const bool bit,
                       const Bit position);
  static bool readBits(const Register registerAddress, uint8_t *const bits,
                       const Bit firstBit, const Length length) const;
  static bool writeBits(const Register registerAddress, const uint8_t bits,
                        const Bit firstBit, const Length length);

  bool readBytes(const Register registerAddress, uint8_t *const bytes,
                 const int8_t size = 1) const;
  bool writeBytes(const Register registerAddress, const uint8_t *const byte,
                  const int8_t size = 1);


  enum class Register : const uint8_t {
    SELF_TEST_X_GYRO = 0x00,
    SELF_TEST_Y_GYRO = 0x01,
    SELF_TEST_Z_GYRO = 0x02,
    SELF_TEST_X_ACCEL = 0x0D,
    SELF_TEST_Y_ACCEL = 0x0E,
    SELF_TEST_Z_ACCEL = 0x0F,
    XG_OFFSET_H = 0x13,
    XG_OFFSET_L = 0x14,
    YG_OFFSET_H = 0x15,
    YG_OFFSET_L = 0x16,
    ZG_OFFSET_H = 0x17,
    ZG_OFFSET_L = 0x18,
    SMPLRT_DIV = 0x19,
    CONFIG = 0x1A,
    GYRO_CONFIG = 0x1B,
    ACCEL_CONFIG = 0x1C,
    ACCEL_CONFIG_2 = 0x1D,
    LP_ACCEL_ODR = 0x1E,
    WOM_THR = 0x1F,
    FIFO_EN = 0x23,
    I2C_MST_CTRL = 0x24,
    I2C_SLV0_ADDR = 0x25,
    I2C_SLV0_REG =  0x26,
    I2C_SLV0_CTRL = 0x27,
    I2C_SLV1_ADDR = 0x28,
    I2C_SLV1_REG =  0x29,
    I2C_SLV1_CTRL = 0x2A,
    I2C_SLV2_ADDR = 0x2B,
    I2C_SLV2_REG =  0x2C,
    I2C_SLV2_CTRL = 0x2D,
    I2C_SLV3_ADDR = 0x2E,
    I2C_SLV3_REG =  0x2F,
    I2C_SLV3_CTRL = 0x30,
    I2C_SLV4_ADDR = 0x31,
    I2C_SLV4_REG =  0x32,
    I2C_SLV4_DO =   0x33,
    I2C_SLV4_CTRL = 0x34,
    I2C_SLV4_DI =   0x35,
    I2C_MST_STATUS = 0x36,
    INT_PIN_CFG = 0x37,
    INT_ENABLE =  0x38,
    INT_STATUS =  0x3A,
    ACCEL_XOUT_H = 0x3B,
    ACCEL_XOUT_L = 0x3C,
    ACCEL_YOUT_H = 0x3D,
    ACCEL_YOUT_L = 0x3E,
    ACCEL_ZOUT_H = 0x3F,
    ACCEL_ZOUT_L = 0x40,
    TEMP_OUT_H =   0x41,
    TEMP_OUT_L =   0x42,
    GYRO_XOUT_H =  0x43,
    GYRO_XOUT_L =  0x44,
    GYRO_YOUT_H =  0x45,
    GYRO_YOUT_L =  0x46,
    GYRO_ZOUT_H =  0x47,
    GYRO_ZOUT_L =  0x48,
    EXT_SENS_DATA_00 = 0x49,
    EXT_SENS_DATA_01 = 0x4A,
    EXT_SENS_DATA_02 = 0x4B,
    EXT_SENS_DATA_03 = 0x4C,
    EXT_SENS_DATA_04 = 0x4D,
    EXT_SENS_DATA_05 = 0x4E,
    EXT_SENS_DATA_06 = 0x4F,
    EXT_SENS_DATA_07 = 0x50,
    EXT_SENS_DATA_08 = 0x51,
    EXT_SENS_DATA_09 = 0x52,
    EXT_SENS_DATA_10 = 0x53,
    EXT_SENS_DATA_11 = 0x54,
    EXT_SENS_DATA_12 = 0x55,
    EXT_SENS_DATA_13 = 0x56,
    EXT_SENS_DATA_14 = 0x57,
    EXT_SENS_DATA_15 = 0x58,
    EXT_SENS_DATA_16 = 0x59,
    EXT_SENS_DATA_17 = 0x5A,
    EXT_SENS_DATA_18 = 0x5B,
    EXT_SENS_DATA_19 = 0x5C,
    EXT_SENS_DATA_20 = 0x5D,
    EXT_SENS_DATA_21 = 0x5E,
    EXT_SENS_DATA_22 = 0x5F,
    EXT_SENS_DATA_23 = 0x60,
    I2C_SLV0_DO = 0x63,
    I2C_SLV1_DO = 0x64,
    I2C_SLV2_DO = 0x65,
    I2C_SLV3_DO = 0x66,
    I2C_MST_DELAY_CTRL = 0x67,
    SIGNAL_PATH_RESET = 0x68,
    MOT_DETECT_CTRL = 0x69,
    USER_CTRL = 0x6A,
    PWR_MGMT_1 = 0x6B, // non-zero reset
    PWR_MGMT_2 = 0x6C,
    FIFO_COUNT_H = 0x72,
    FIFO_COUNT_L = 0x73,
    FIFO_R_W = 0x74,
    WHO_AM_I = 0x75, // reset 0x73
    XA_OFFSET_H = 0x77,
    XA_OFFSET_L = 0x78,
    YA_OFFSET_H = 0x7A,
    YA_OFFSET_L = 0x7B,
    ZA_OFFSET_H = 0x7D,
    ZA_OFFSET_L = 0x7E,
  };

  enum class Bit : uint8_t {
    // SELF_TEST_n_GYRO
    XG_ST_DATA = 0x07,
    YG_ST_DATA = 0x07,
    ZG_ST_DATA = 0x07,

    // SELF_TEST_n_ACCEL
    XA_ST_DATA = 0x07,
    YA_ST_DATA = 0x07,
    ZA_ST_DATA = 0x07,

    // nG_OFFSET
    X_OFFS_USR = 0x07,
    Y_OFFS_USR = 0x07,
    Z_OFFS_USR = 0x07,

    // SMPLRT_DIV
    SMPLRT_DIV = 0x07,

    // CONFIG
    FIFO_MODE = 6,
    EXT_SYNC_SET = 5,
    DLPF_CFG = 2,

    // GYRO_CONFIG
    XGYRO_CT_EN = 0x07,
    YGYRO_CT_EN = 0x06,
    ZGYRO_CT_EN = 0x05,
    GYRO_FS_SEL = 0x04,
    FCHOICE_B = 0x01,

    // ACCEL_CONFIG
    AX_ST_EN = 0x07,
    AY_ST_EN = 0x06,
    AZ_ST_EN = 0x05,
    ACCEL_FS_SEL = 0x04,

    // ACCEL_CONFIG 2
    ACCEL_FCHOICE_B = 0x03,
    A_DLPF_CFG = 0x01,

    // LP_ACCEL_ODR
    LPOSC_CLKSEL = 0x03,

    // WOM_THR
    WOM_THRESHOLD = 0x07,

    // FIFO_EN
    TEMP_FIFO_EN = 0x07,
    GYRO_XO_UT = 0x06,
    GYRO_YO_UT = 0x05,
    GYRO_ZO_UT = 0x04,
    ACCEL = 0x03,
    SLV2 = 0x02,
    SLV1 = 0x01,
    SLV0 = 0x00,

    // I2C_MST_CTRL
    MULT_MST_EN = 0x07,
    WAIT_FOR_ES = 0x06,
    SLV_3_FIFO_EN = 0x05,
    I2C_MST_P_NSR = 0x04,
    I2C_MST_CLK = 0x03,

    // I2C_SLV0_ADDR
    I2C_SLV0_RNW = 0x07,
    I2C_ID_0 = 0x06,

    // I2C_SLV0_REG
    I2C_SLV0_REG = 0x07,

    // I2C_SLV0_CTRL
    I2C_SLV0_EN = 0x07,
    I2C_SLV0_BYTE_SW = 0x06,
    I2C_SLV0_REG_DIS = 0x05,
    I2C_SLV0_GRP = 0x04,
    I2C_SLV0_LENG = 0x03,

    // I2C_SLV1_ADDR
    I2C_SLV1_RNW = 0x07,
    I2C_ID_1 = 0x06,

    // I2C_SLV1_REG
    I2C_SLV1_REG = 0x07,

    // I2C_SLV1_CTRL
    I2C_SLV1_EN = 0x07,
    I2C_SLV1_BYTE_SW = 0x06,
    I2C_SLV1_REG_DIS = 0x05,
    I2C_SLV1_GRP = 0x04,
    I2C_SLV1_LENG = 0x03,

    // I2C_SLV2_ADDR
    I2C_SLV2_RNW = 0x07,
    I2C_ID_2 = 0x06,

    // I2C_SLV2_REG
    I2C_SLV2_REG = 0x07,

    // I2C_SLV2_CTRL
    I2C_SLV2_EN = 0x07,
    I2C_SLV2_BYTE_SW = 0x06,
    I2C_SLV2_REG_DIS = 0x05,
    I2C_SLV2_GRP = 0x04,
    I2C_SLV2_LENG = 0x03,

    // I2C_SLV3_ADDR
    I2C_SLV3_RNW = 0x07,
    I2C_ID_3 = 0x06,

    // I2C_SLV3_REG
    I2C_SLV3_REG = 0x07,

    // I2C_SLV3_CTRL
    I2C_SLV3_EN = 0x07,
    I2C_SLV3_BYTE_SW = 0x06,
    I2C_SLV3_REG_DIS = 0x05,
    I2C_SLV3_GRP = 0x04,
    I2C_SLV3_LENG = 0x03,

    // I2C_SLV4_ADDR
    I2C_SLV4_RNW = 0x07,
    I2C_ID_4 = 0x06,

    // I2C_SLV4_REG
    I2C_SLV4_REG = 0x07,

    // I2C_SLV4_DO
    I2C_SLV4_DO = 0x07,

    // I2C_SLV4_CTRL
    I2C_SLV4_EN = 0x07,
    I2C_SLV4_DONE_INT_EN = 0x06,
    I2C_SLV4_REG_DIS = 0x05,
    I2C_MST_DLY = 0x04,

    // I2C_SLV4_DI
    I2C_SLV4_DI = 0x07,

    // I2C_MST_STATUS
    PASS_THROUGH = 0x07,
    I2C_SLV4_DONE = 0x06,
    I2C_LOST_ARB = 0x05,
    I2C_SLV4_NACK = 0x04,
    SLV3_NACK = 0x03,
    SLV2_NACK = 0x02,
    SLV1_NACK = 0x01,
    SLV0_NACK = 0x00,

    // INT_PIN_CFG
    ACTL = 0x07,
    OPEN = 0x06,
    LATCH_INT_EN = 0x05,
    INT_ANYRD_2CLEAR = 0x04,
    ACTL_FSYNC = 0x03,
    FSYNC_INT_MODE_EN = 0x02,
    BYPASS_EN = 0x01,

    // INT_ENABLE
    WOM_EN = 0x06,
    FIFO_OFLOW_EN = 0x04,
    FSYNC_INT_EN = 0x03,
    RAW_RDY_EN = 0x00,

    // INT_STATUS
    WOM_INT = 0x06,
    FIFO_OFLOW_INT = 0x04,
    FSYNC_INT_INT = 0x03,
    RAW_DATA_RDY_INT = 0x00,

    // ACCEL_XOUT_H
    ACCEL_XOUT_H = 0x07,
    // ACCEL_XOUT_L
    ACCEL_XOUT_L = 0x07,
    // ACCEL_YOUT_H
    ACCEL_YOUT_H = 0x07,
    // ACCEL_YOUT_L
    ACCEL_YOUT_L = 0x07,
    // ACCEL_ZOUT_H
    ACCEL_ZOUT_H = 0x07,
    // ACCEL_ZOUT_L
    ACCEL_ZOUT_L = 0x07,
    // TEMP_OUT_H
    TEMP_OUT_H = 0x07,
    // TEMP_OUT_L
    TEMP_OUT_L = 0x07,
    // GYRO_XOUT_H
    GYRO_XOUT_H = 0x07,
    // GYRO_XOUT_L
    GYRO_XOUT_L = 0x07,
    // GYRO_YOUT_H
    GYRO_YOUT_H = 0x07,
    // GYRO_YOUT_L
    GYRO_YOUT_L = 0x07,
    // GYRO_ZOUT_H
    GYRO_ZOUT_H = 0x07,
    // GYRO_ZOUT_L
    GYRO_ZOUT_L = 0x07,
    // EXT_SENS_DATA_00
    EXT_SENS_DATA_00 = 0x07,
    // EXT_SENS_DATA_02
    EXT_SENS_DATA_02 = 0x07,
    // EXT_SENS_DATA_03
    EXT_SENS_DATA_03 = 0x07,
    // EXT_SENS_DATA_04
    EXT_SENS_DATA_04 = 0x07,
    // EXT_SENS_DATA_05
    EXT_SENS_DATA_05 = 0x07,
    // EXT_SENS_DATA_06
    EXT_SENS_DATA_06 = 0x07,
    // EXT_SENS_DATA_07
    EXT_SENS_DATA_07 = 0x07,
    // EXT_SENS_DATA_08
    EXT_SENS_DATA_08 = 0x07,
    // EXT_SENS_DATA_09
    EXT_SENS_DATA_09 = 0x07,
    // EXT_SENS_DATA_10
    EXT_SENS_DATA_10 = 0x07,
    // EXT_SENS_DATA_11
    EXT_SENS_DATA_11 = 0x07,
    // EXT_SENS_DATA_12
    EXT_SENS_DATA_12 = 0x07,
    // EXT_SENS_DATA_13
    EXT_SENS_DATA_13 = 0x07,
    // EXT_SENS_DATA_14
    EXT_SENS_DATA_14 = 0x07,
    // EXT_SENS_DATA_15
    EXT_SENS_DATA_15 = 0x07,
    // EXT_SENS_DATA_16
    EXT_SENS_DATA_16 = 0x07,
    // EXT_SENS_DATA_17
    EXT_SENS_DATA_17 = 0x07,
    // EXT_SENS_DATA_18
    EXT_SENS_DATA_18 = 0x07,
    // EXT_SENS_DATA_19
    EXT_SENS_DATA_19 = 0x07,
    // EXT_SENS_DATA_20
    EXT_SENS_DATA_20 = 0x07,
    // EXT_SENS_DATA_21
    EXT_SENS_DATA_21 = 0x07,
    // EXT_SENS_DATA_22
    EXT_SENS_DATA_22 = 0x07,
    // EXT_SENS_DATA_23
    EXT_SENS_DATA_23 = 0x07,
    // I2C_SLV0_DO
    I2C_SLV0_DO = 0x07,
    // I2C_SLV1_DO
    I2C_SLV1_DO = 0x07,
    // I2C_SLV2_DO
    I2C_SLV2_DO = 0x07,
    // I2C_SLV3_DO
    I2C_SLV3_DO = 0x07,

    // I2C_MST_DELAY_CTRL
    DELAY_ES_SHADOW = 0x07,
    I2C_SLV4_DLY_EN = 0x04,
    I2C_SLV3_DLY_EN = 0x03,
    I2C_SLV2_DLY_EN = 0x02,
    I2C_SLV1_DLY_EN = 0x01,
    I2C_SLV0_DLY_EN = 0x00,

    // SIGNAL_PATH_RESET
    GYRO_RST = 0x02,
    ACCEL_RST = 0x01,
    TEMP_RST = 0x00,

    // MOT_DETECT_CTRL
    ACCEL_INTEL_EN = 0x07,
    ACCEL_INTEL_MODE = 0x06,

    // USER_CTRL
    FIFO_EN = 0x06,
    I2C_MST_EN = 0x05,
    I2C_IF_DIS = 0x04,
    FIFO_RST = 0x02,
    I2C_MST_RST = 0x01,
    SIG_COND_RST = 0x00,

    // PWR_MGMT_1
    H_RESET = 0x07,
    SLEEP = 0x06,
    CYCLE = 0x05,
    GYRO_STANDBY = 0x04,
    PD_PTAT = 0x03,
    CLKSEL = 0x02,

    // PWR_MGMT_2
    DIS_XA = 0x05,
    DIS_YA = 0x04,
    DIS_ZA = 0x03,
    DIS_XG = 0x02,
    DIS_YG = 0x01,
    DIS_ZG = 0x00,

    // FIFO_COUNTH
    FIFO_CNT = 0x04,

    // FIFO_R_W
    D = 0x07,

    // WHO_AM_I
    WHOAMI = 0x07,

    // XA_OFFSET_H
    XA_OFFS = 0x07,
    // YA_OFFSET_H
    YA_OFFS = 0x07,
    // ZA_OFFSET_H
    ZA_OFFS = 0x07,
  };
  enum class Length : uint8_t {
    // SELF_TEST_n_GYRO
    XG_ST_DATA = 8,
    YG_ST_DATA = 8,
    ZG_ST_DATA = 8,

    // SELF_TEST_n_ACCEL
    XA_ST_DATA = 8,
    YA_ST_DATA = 8,
    ZA_ST_DATA = 8,

    // nG_OFFSET
    X_OFFS_USR = 16,
    Y_OFFS_USR = 16,
    Z_OFFS_USR = 16,

    // SMPLRT_DIV
    SMPLRT_DIV = 8,

    // CONFIG
    FIFO_MODE = 1,
    EXT_SYNC_SET = 3,
    DLPF_CFG = 3,

    // GYRO_CONFIG
    XGYRO_CT_EN = 1,
    YGYRO_CT_EN = 1,
    ZGYRO_CT_EN = 1,
    GYRO_FS_SEL = 2,
    FCHOICE_B = 2,

    // ACCEL_CONFIG
    AX_ST_EN = 1,
    AY_ST_EN = 1,
    AZ_ST_EN = 1,
    ACCEL_FS_SEL = 2,

    // ACCEL_CONFIG 2
    ACCEL_FCHOICE_B = 1,
    A_DLPF_CFG = 3,

    // LP_ACCEL_ODR
    LPOSC_CLKSEL = 4,

    // WOM_THR
    WOM_THRESHOLD = 8,

    // FIFO_EN
    TEMP_FIFO_EN = 1,
    GYRO_XO_UT = 1,
    GYRO_YO_UT = 1,
    GYRO_ZO_UT = 1,
    ACCEL = 1,
    SLV2 = 1,
    SLV1 = 1,
    SLV0 = 1,

    // I2C_MST_CTRL
    MULT_MST_EN = 1,
    WAIT_FOR_ES = 1,
    SLV_3_FIFO_EN = 1,
    I2C_MST_P_NSR = 1,
    I2C_MST_CLK = 4,

    // I2C_SLV0_ADDR
    I2C_SLV0_RNW = 1,
    I2C_ID_0 = 7,

    // I2C_SLV0_REG
    I2C_SLV0_REG = 8,

    // I2C_SLV0_CTRL
    I2C_SLV0_EN = 1,
    I2C_SLV0_BYTE_SW = 1,
    I2C_SLV0_REG_DIS = 1,
    I2C_SLV0_GRP = 1,
    I2C_SLV0_LENG = 4,

    // I2C_SLV1_ADDR
    I2C_SLV1_RNW = 1,
    I2C_ID_1 = 7,

    // I2C_SLV1_REG
    I2C_SLV1_REG = 8,

    // I2C_SLV1_CTRL
    I2C_SLV1_EN = 1,
    I2C_SLV1_BYTE_SW = 1,
    I2C_SLV1_REG_DIS = 1,
    I2C_SLV1_GRP = 1,
    I2C_SLV1_LENG = 4,

    // I2C_SLV2_ADDR
    I2C_SLV2_RNW = 1,
    I2C_ID_2 = 7,

    // I2C_SLV2_REG
    I2C_SLV2_REG = 8,

    // I2C_SLV2_CTRL
    I2C_SLV2_EN = 1,
    I2C_SLV2_BYTE_SW = 1,
    I2C_SLV2_REG_DIS = 1,
    I2C_SLV2_GRP = 1,
    I2C_SLV2_LENG = 4,

    // I2C_SLV3_ADDR
    I2C_SLV3_RNW = 1,
    I2C_ID_3 = 7,

    // I2C_SLV3_REG
    I2C_SLV3_REG = 8,

    // I2C_SLV3_CTRL
    I2C_SLV3_EN = 1,
    I2C_SLV3_BYTE_SW = 1,
    I2C_SLV3_REG_DIS = 1,
    I2C_SLV3_GRP = 1,
    I2C_SLV3_LENG = 4,

    // I2C_SLV4_ADDR
    I2C_SLV4_RNW = 1,
    I2C_ID_4 = 0x07,

    // I2C_SLV4_REG
    I2C_SLV4_REG = 8,

    // I2C_SLV4_DO
    I2C_SLV4_DO = 8,

    // I2C_SLV4_CTRL
    I2C_SLV4_EN = 1,
    I2C_SLV4_DONE_INT_EN = 1,
    I2C_SLV4_REG_DIS = 1,
    I2C_MST_DLY = 0x05,

    // I2C_SLV4_DI
    I2C_SLV4_DI = 8,

    // I2C_MST_STATUS
    PASS_THROUGH = 1,
    I2C_SLV4_DONE = 1,
    I2C_LOST_ARB = 1,
    I2C_SLV4_NACK = 1,
    SLV3_NACK = 1,
    SLV2_NACK = 1,
    SLV1_NACK = 1,
    SLV0_NACK = 1,

    // INT_PIN_CFG
    ACTL = 1,
    OPEN = 1,
    LATCH_INT_EN = 1,
    INT_ANYRD_2CLEAR = 1,
    ACTL_FSYNC = 1,
    FSYNC_INT_MODE_EN = 1,
    BYPASS_EN = 1,

    // INT_ENABLE
    WOM_EN = 1,
    FIFO_OFLOW_EN = 1,
    FSYNC_INT_EN = 1,
    RAW_RDY_EN = 1,

    // INT_STATUS
    WOM_INT = 1,
    FIFO_OFLOW_INT = 1,
    FSYNC_INT_INT = 1,
    RAW_DATA_RDY_INT = 1,

    // ACCEL_XOUT_H
    ACCEL_XOUT_H = 8,
    // ACCEL_XOUT_L
    ACCEL_XOUT_L = 8,
    // ACCEL_YOUT_H
    ACCEL_YOUT_H = 8,
    // ACCEL_YOUT_L
    ACCEL_YOUT_L = 8,
    // ACCEL_ZOUT_H
    ACCEL_ZOUT_H = 8,
    // ACCEL_ZOUT_L
    ACCEL_ZOUT_L = 8,
    // TEMP_OUT_H
    TEMP_OUT_H = 8,
    // TEMP_OUT_L
    TEMP_OUT_L = 8,
    // GYRO_XOUT_H
    GYRO_XOUT_H = 8,
    // GYRO_XOUT_L
    GYRO_XOUT_L = 8,
    // GYRO_YOUT_H
    GYRO_YOUT_H = 8,
    // GYRO_YOUT_L
    GYRO_YOUT_L = 8,
    // GYRO_ZOUT_H
    GYRO_ZOUT_H = 8,
    // GYRO_ZOUT_L
    GYRO_ZOUT_L = 8,
    // EXT_SENS_DATA_00
    EXT_SENS_DATA_00 = 8,
    // EXT_SENS_DATA_02
    EXT_SENS_DATA_02 = 8,
    // EXT_SENS_DATA_03
    EXT_SENS_DATA_03 = 8,
    // EXT_SENS_DATA_04
    EXT_SENS_DATA_04 = 8,
    // EXT_SENS_DATA_05
    EXT_SENS_DATA_05 = 8,
    // EXT_SENS_DATA_06
    EXT_SENS_DATA_06 = 8,
    // EXT_SENS_DATA_07
    EXT_SENS_DATA_07 = 8,
    // EXT_SENS_DATA_08
    EXT_SENS_DATA_08 = 8,
    // EXT_SENS_DATA_09
    EXT_SENS_DATA_09 = 8,
    // EXT_SENS_DATA_10
    EXT_SENS_DATA_10 = 8,
    // EXT_SENS_DATA_11
    EXT_SENS_DATA_11 = 8,
    // EXT_SENS_DATA_12
    EXT_SENS_DATA_12 = 8,
    // EXT_SENS_DATA_13
    EXT_SENS_DATA_13 = 8,
    // EXT_SENS_DATA_14
    EXT_SENS_DATA_14 = 8,
    // EXT_SENS_DATA_15
    EXT_SENS_DATA_15 = 8,
    // EXT_SENS_DATA_16
    EXT_SENS_DATA_16 = 8,
    // EXT_SENS_DATA_17
    EXT_SENS_DATA_17 = 8,
    // EXT_SENS_DATA_18
    EXT_SENS_DATA_18 = 8,
    // EXT_SENS_DATA_19
    EXT_SENS_DATA_19 = 8,
    // EXT_SENS_DATA_20
    EXT_SENS_DATA_20 = 8,
    // EXT_SENS_DATA_21
    EXT_SENS_DATA_21 = 8,
    // EXT_SENS_DATA_22
    EXT_SENS_DATA_22 = 8,
    // EXT_SENS_DATA_23
    EXT_SENS_DATA_23 = 8,
    // I2C_SLV0_DO
    I2C_SLV0_DO = 8,
    // I2C_SLV1_DO
    I2C_SLV1_DO = 8,
    // I2C_SLV2_DO
    I2C_SLV2_DO = 8,
    // I2C_SLV3_DO
    I2C_SLV3_DO = 8,

    // I2C_MST_DELAY_CTRL
    DELAY_ES_SHADOW = 1,
    I2C_SLV4_DLY_EN = 1,
    I2C_SLV3_DLY_EN = 1,
    I2C_SLV2_DLY_EN = 1,
    I2C_SLV1_DLY_EN = 1,
    I2C_SLV0_DLY_EN = 1,

    // SIGNAL_PATH_RESET
    GYRO_RST = 1,
    ACCEL_RST = 1,
    TEMP_RST = 1,

    // MOT_DETECT_CTRL
    ACCEL_INTEL_EN = 1,
    ACCEL_INTEL_MODE = 1,

    // USER_CTRL
    FIFO_EN = 1,
    I2C_MST_EN = 1,
    I2C_IF_DIS = 1,
    FIFO_RST = 1,
    I2C_MST_RST = 1,
    SIG_COND_RST = 1,

    // PWR_MGMT_1
    H_RESET = 1,
    SLEEP = 1,
    CYCLE = 1,
    GYRO_STANDBY = 1,
    PD_PTAT = 1,
    CLKSEL = 3,

    // PWR_MGMT_2
    DIS_XA = 1,
    DIS_YA = 1,
    DIS_ZA = 1,
    DIS_XG = 1,
    DIS_YG = 1,
    DIS_ZG = 1,

    // FIFO_COUNTH
    FIFO_CNT = 19,

    // FIFO_R_W
    D = 8,

    // WHO_AM_I
    WHOAMI = 8,

    // XA_OFFSET_H
    XA_OFFS = 15,
    // YA_OFFSET_H
    YA_OFFS = 15,
    // ZA_OFFSET_H
    ZA_OFFS = 15,
  };
};
