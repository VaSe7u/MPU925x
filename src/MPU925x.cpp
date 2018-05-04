#include "MPU925x.hpp"

MPU925x::MPU925x(const uint8_t device) : _device(device) {}
MPU925x::MPU925x(const AD0 ad0) : _device((uint8_t)ad0) {}

void MPU925x::attachInterface(bool (*read)(uint8_t device, uint8_t registerAddress, uint8_t *data, int8_t size),
                              bool (*write)(uint8_t device, uint8_t registerAddress, uint8_t *data, int8_t size)) {
  _read = read;
  _write = write;
}

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

void MPU925x::initialize() {
  reset();
  sleepMode(false);
  clockSource(CLOCK_PLL);
  /* I2C master
  enable I2C master
  set I2C bus speed

  */
}

void MPU925x::reset() {
  return _writeBit(Register::PWR_MGMT_1, true, Bit::H_RESET);
  // uint8_t reg = 0;
  // if (_read(_device, (uint8_t)Register::PWR_MGMT_1, &reg, 1)) {
  //   reg |= (1 << (uint8_t)Bit::H_RESET);
  //   return _write(_device, (uint8_t)Register::PWR_MGMT_1, reg, 1);
  // } else {
  //   return false;
  // }
}

uint8_t MPU925x::getDeviceId() const {
  uint8_t deviceId = 0;
  _readBytes(Register::WHO_AM_I, &deviceId);
  return deviceId;
  // uint8_t deviceId = 0;
  // _read(_device, (uint8_t)Register::WHO_AM_I, &deviceId, 1);
  // return deviceId;
}

bool MPU925x::checkConnection() const {
  uint8_t deviceId = getDeviceId();
  if (deviceId == MPU9250_ID || deviceId == MPU9255_ID) return true;
  else return false;
}


void MPU925x::sleepMode(const bool enable) {
  return _writeBit(Register::PWR_MGMT_1, enable, Bit::SLEEP);
  // uint8_t reg = 0;
  // if (_read(_device, (uint8_t)Register::PWR_MGMT_1, &reg, 1)) {
  //   if (enable) {
  //     reg |= (1 << (uint8_t)Bit::SLEEP);
  //   } else {
  //     reg &= ~(1 << (uint8_t)Bit::SLEEP);
  //   }
  //   return _write(_device, (uint8_t)Register::PWR_MGMT_1, reg, 1);
  // } else {
  //   return false;
  // }
}

bool MPU925x::getSleepMode() const {
  bool sleepMode = false;
  if (_readBit(Register::PWR_MGMT_1, &sleepMode, Bit::SLEEP)) {
    return sleepMode;
  }
  // uint8_t reg = 0;
  // _read(_device, (uint8_t)Register::PWR_MGMT_1, &reg, 1);
  // return reg & (1 << (uint8_t)Bit::SLEEP);
}


void MPU925x::clockSource(const ClockSource clockSource) {
  _writeBits(Register::PWR_MGMT_1, (uint8_t)clockSource, Bit::CLKSEL, Length::CLKSEL);
  // uint8_t reg = 0;
  // if (_read(_device, (uint8_t)Register::PWR_MGMT_1, &reg, 1)) {
  //   reg = changeBits(reg, (uint8_t)clockSource, (uint8_t)Bit::CLKSEL, (uint8_t)Length::CLKSEL);
  //   _write(_device, (uint8_t)Register::PWR_MGMT_1, reg, 1);
  // }
}

ClockSource MPU925x::getClockSource() const {
  uint8_t cs = 0;
  _readBits(Register::PWR_MGMT_1, &cs, Bit::CLKSEL, Length::CLKSEL);
  return (ClockSource)cs;
  // uint8_t reg = 0;
  // if (_read(_device, (uint8_t)Register::PWR_MGMT_1, &reg, 1)) {
  //   return (ClockSource)extractBits(reg, (uint8_t)Bit::CLKSEL, (uint8_t)Length::CLKSEL);
  // } else {
  //   return (ClockSource)0;
  // }
}


bool MPU925x::selfTestAccel() const { // enable bits7,6,5 from reg28
  // regs 0 - 2, 13 - 15 self test output during manufacturing
  // response = output w/o st - output w/ st
  // response should be close to st during manufacturing (+-14)
  // only LSByte
  if (_writeBits(Register::ACCEL_CONFIG, 0b111, Bit::AX_ST_EN, (Length)3)) {

  }
  return false;
}

bool MPU925x::seltTestGyro() const { // enable bits7,6,5 from reg27
  // regs 0 - 2, 13 - 15 self test output during manufacturing
  // response = output w/o st - output w/ st
  // response should be close to st during manufacturing (+-14)
  // only LSByte
  return false;
}



bool MPU925x::dlpf(const Bandwidth bandwidth) {
  return (accelDlpf(bandwidth) && gyroDlpf(bandwidth));
}

bool MPU925x::accelDlpf(const Bandwidth bandwidth) {
  uint8_t fchoiceMask = 0b0000;
  uint8_t reg = 0;
  if ((uint8_t)bandwidth > 6 || (uint8_t)bandwidth < -1) {
    return false;
  } else if (bandwidth == BANDWIDTH_1130Hz) {
    fchoiceMask = 0b1000;
    reg = 0;
  } else {
    reg = (uint8_t)bandwidth;
  }
  uint8_t bits = (fchoiceMask | reg) & 0x0f;

  uint8_t reg = 0;
  return _writeBits(Register::ACCEL_CONFIG_2, bits, Bit::ACCEL_FCHOICE_B,
                    (Length)((uint8_t)Length::A_DLPF_CFG + 1));
  // if (_read(_device, (uint8_t)Register::ACCEL_CONFIG_2, &reg, 1)) {
  //   reg = changeBits(reg, (uint8_t)bits, (uint8_t)Bit::ACCEL_FCHOICE_B,
  //                    (uint8_t)Length::A_DLPF_CFG + 1);
  //   return _write(_device, (uint8_t)Register::ACCEL_CONFIG_2, reg, 1);
  // } else {
  //   return false;
  // }
}

Bandwidth MPU925x::getAccelDlpf() const {
  bool fchoice = false;
  if (_readBit(Register::ACCEL_CONFIG_2, &fchoice, Bit::ACCEL_FCHOICE_B)) {
    if (fchoice) {
      return BANDWIDTH_1130Hz;
    } else {
      uint8_t reg = 0;
      if (_readBits(Register::ACCEL_CONFIG_2, &reg, Bit::A_DLPF_CFG, Length::A_DLPF_CFG)) {
        return (Bandwidth)reg;
      }
    }
  }
  return (Bandwidth)0;
  // if (_read(_device, (uint8_t)Register::ACCEL_CONFIG_2, &reg, 1)) {
  //   if (reg & (1 << (uint8_t)Bit::ACCEL_FCHOICE_B)) {
  //     return BANDWIDTH_1130Hz;
  //   } else {
  //     return (Bandwidth)extractBits(reg, (uint8_t)Bit::A_DLPF_CFG,
  //                                   (uint8_t)Length::A_DLPF_CFG);
  //   }
  // } else {
  //   return (Bandwidth)0;
  // }
}

bool MPU925x::gyroDlpf(const Bandwidth bandwidth) {
  uint8_t fchoice = 0b00;
  uint8_t reg = 0;
  if ((uint8_t)bandwidth > 6 || (uint8_t)bandwidth < -2) {
    return false;
  } else if (bandwidth == BANDWIDTH_3600Hz) {
    fchoice = 0b10;
    reg = 0;
  } else if (bandwidth == BANDWIDTH_8800Hz) {
    fchoice = 0b01;
    reg = 0;
  } else {
    reg = (uint8_t)bandwidth;
  }
  if (_writeBits(Register::GYRO_CONFIG, fchoice, Bit::FCHOICE_B, Length::FCHOICE_B)
      && _writeBits(Register::CONFIG, reg, Bit::DLPF_CFG, Length::DLPF_CFG)) {
    return true;
  } else {
    return false;
  }
  // uint8_t fchoice = 0b00;
  // if ((uint8_t)bandwidth > 6 || (uint8_t)bandwidth < -2) {
  //   return false;
  // } else if (bandwidth == BANDWIDTH_3600Hz) {
  //   fchoice = 0b10;
  // } else if (bandwidth == BANDWIDTH_8800Hz) {
  //   fchoice = 0b01;
  // }

  // uint8_t configReg = 0;
  // bool configS = _read(_device, (uint8_t)Register::CONFIG, &configReg, 1);
  // uint8_t gyroReg = 0;
  // bool gyroS = _read(_device, (uint8_t)Register::GYRO_CONFIG, &gyroReg, 1);

  // if (configS && gyroS) {
  //   configReg = changeBits(configReg, (uint8_t)bandwidth, (uint8_t)Bit::DLPF_CFG_SET,
  //                          (uint8_t)Length::DLPF_CFG_SET);
  //   gyroReg = changeBits(gyroReg, (uint8_t)fchoice, (uint8_t)Bit::FCHOICE_B,
  //                        (uint8_t)Length::FCHOICE_B);
  //   if (_write(_device, (uint8_t)Register::CONFIG, configReg, 1)) {
  //     if (_write(_device, (uint8_t)Register::GYRO_CONFIG, gyroReg, 1)) {
  //       return true;
  //     }
  //   }
  // }
  // return false;
}

Bandwidth MPU925x::getGyroDlpf() const {
  uint8_t fchoice = 0b00;
  if (_readBits(Register::GYRO_CONFIG, &fchoice, Bit::FCHOICE_B, Length::FCHOICE_B)) {
    if (fchoice == 0b00) { // 0..7
      if (_readBits(Register::CONFIG, &reg, Bit::DLPF_CFG, Length::DLPF_CFG)) {
        return (Bandwidth)reg;
      }
    } else if (fchoice == 0b10) { // 3600Hz
      return BANDWIDTH_3600Hz;
    } else { // 8800Hz
      return BANDWIDTH_8800Hz;
    }
  }
  return (Bandwidth)0;
  // uint8_t gyroReg = 0;
  // if (_read(_device, (uint8_t)Register::GYRO_CONFIG, gyroReg, 1)) {
  //   uint8_t fchoice = extractBits(gyroReg, (uint8_t)Bit::FCHOICE_B, (uint8_t)Length::FCHOICE_B);
  //   if (fchoice == 0b00) { // 0..7
  //     uint8_t configReg = 0;
  //     if (_read(_device, (uint8_t)Register::CONFIG, configReg, 1)) {
  //       return (Bandwidth)extractBits(configReg, (uint8_t)Bit::DLPF_CFG,
  //                                     (uint8_t)Length::DLPF_CFG);
  //     }
  //   } else if (fchoice == 0b10) { // 3600Hz
  //     return BANDWIDTH_3600Hz;
  //   } else { // 8800Hz
  //     return BANDWIDTH_8800Hz;
  //   }
  // }
  // return (Bandwidth)0;
}


void MPU925x::outputDataRateDivider(const int16_t outputDataRateDivider) {
  // CLOCK/(1+divider), Clk = 1kHz, check if DLPF Fs is 1kHz
  if (outputDataRateDivider <= 256 && outputDataRateDivider >= 0) {
    return _writeBytes(Register::SMPLRT_DIV, outputDataRateDivider - 1);
  } else {
    return false;
  }
}

int16_t MPU925x::getOutputDataRateDivider() const {
  uint8_t reg = 0;
  _readBytes(Register::SMPLRT_DIV, &reg);
  return (int16_t)reg + 1;
}

// bool MPU925x::outputDataRate(const int16_t outputDataRate) { // not ready
//   uint8_t odrDivider = round(1000.0f / outputDataRate - 1.0f);
//   return false;
// }

float MPU925x::getOutputDataRate() const {
  int16_t odrDivider = getOutputDataRateDivider;
  return 1000.0f / odrDivider;
}


bool MPU925x::accelRange(const Range range) {
  if ((int8_t)range < 0 || (uint8_t)range > 0b11) return false;
  if (_writeBits(Register::ACCEL_CONFIG, (uint8_t)reg,
                 Bit::ACCEL_FS_SEL, Length::ACCEL_FS_SEL)) {
    if (range == RANGE_2G) _accelSensitivity_g = 16384;
    else if (range == RANGE_4G) _accelSensitivity_g = 8192;
    else if (range == RANGE_8G) _accelSensitivity_g = 4096;
    else if (range == RANGE_16G) _accelSensitivity_g = 2048;
    _accelSensitivity_mS2 = (float)_accelSensitivity_g / 9.80665f;
    return true;
  } else {
    return false;
  }
}

Range MPU925x::getAccelRange() const {
  uint8_t reg = 0;
  if (_readBits(Register::ACCEL_CONFIG, &reg, Bit::ACCEL_FS_SEL, Length::ACCEL_FS_SEL)) {
    return (Range)reg;
  } else {
    return (Range)0;
  }
}

bool MPU925x::gyroRange(const Range range) {
  if ((int8_t)range < 0 || (uint8_t)range > 0b11) return false;
  if (_writeBits(Register::GYRO_CONFIG, (uint8_t)reg,
                 Bit::GYRO_FS_SEL, Length::GYRO_FS_SEL)) {
    if (range == RANGE_250DPS) _gyroSensitivity_degS = 131.0f;
    else if (range == RANGE_500DPS) _gyroSensitivity_degS = 65.5f;
    else if (range == RANGE_1000DPS) _gyroSensitivity_degS = 32.8f;
    else if (range == RANGE_2000DPS) _gyroSensitivity_degS = 16.4f;
    _gyroSensitivity_radS = _gyroSensitivity_degS / 0.0174533f;
    return true;
  } else {
    return (Range)0;
  }
}

Range MPU925x::getGyroRange() const {
  uint8_t reg = 0;
  if (_readBits(Register::GYRO_CONFIG, &reg, Bit::GYRO_FS_SEL, Length::GYRO_FS_SEL)) {
    return (Range)reg;
  } else {
    return (Range)0;
  }
}


bool MPU925x::offsetAccel(const int16_t x, const int16_t y, const int16_t z) {
  // check if under 15 bits
  if ((uint16_t)x & ~0x8000 > 0x3fff
      || (uint16_t)y & ~0x8000 > 0x3fff
      || (uint16_t)z & ~0x8000 > 0x3fff) {
    return false;
  }
  return (offsetAccelX(x) && offsetAccelY(y) && offsetAccelZ(z));
}

bool MPU925x::getOffsetAccel(int16_t &x, int16_t &y, int16_t &z) const {
  return (getOffsetAccelX(x) && getOffsetAccelY(y) && getOffsetAccelZ(z));
}

bool MPU925x::offsetAccelX(const int16_t xOffset) {
  // 15 bit signed int, 2 registers, last bit should be zero
  uint8_t regs[2];
  if (_offsetToRegister(xOffset, regs)) {
    return _writeBytes(Register::XA_OFFSET_H, regs, 2);
  }
  return false;
}

bool MPU925x::getOffsetAccelX(int16_t &xOffset) const {
  uint8_t reg[2] = {0, 0};
  if (_readBytes(Register::XA_OFFSET_H, reg, 2)) {
    return _registerToOffset(reg, xOffset);
  } else {
    return false;
  }
}

bool MPU925x::offsetAccelY(const int16_t yOffset) {
  uint8_t regs[2];
  if (_offsetToRegister(yOffset, regs)) {
    return _writeBytes(Register::YA_OFFSET_H, regs, 2);
  }
  return false;
}

bool MPU925x::getOffsetAccelY(int16_t &yOffset) const {
  uint8_t reg[2] = {0, 0};
  if (_readBytes(Register::YA_OFFSET_H, reg, 2)) {
    return _registerToOffset(reg, yOffset);
  } else {
    return false;
  }
}

bool MPU925x::offsetAccelZ(const int16_t zOffset) {
  uint8_t regs[2];
  if (_offsetToRegister(zOffset, regs)) {
    return _writeBytes(Register::ZA_OFFSET_H, regs, 2);
  }
  return false;
}

bool MPU925x::getOffsetAccelZ(int16_t &zOffset) const {
  uint8_t reg[2] = {0, 0};
  if (_readBytes(Register::ZA_OFFSET_H, reg, 2)) {
    return _registerToOffset(reg, zOffset);
  } else {
    return false;
  }
}

bool MPU925x::offsetGyro(const int16_t x, const int16_t y, const int16_t z) {
  return (offsetGyroX(x) && offsetGyroY(y) && offsetGyroZ(z));
}

bool MPU925x::getOffsetGyro(int16_t &x, int16_t &y, int16_t &z) const {
  return (getOffsetGyroX(x) && getOffsetGyroY(y) && getOffsetGyroZ(z));
}

bool MPU925x::offsetGyroX(const int16_t xOffset) {
  uint8_t regs[2];
  regs[0] = (uint8_t)xOffset << 8;
  regs[1] |= (uint8_t)xOffset;
  return (_writeBytes(Register::XG_OFFSET_H, regs, 2));
}

bool MPU925x::getOffsetGyroX(int16_t &xOffset) const {
  uint8_t regs[2] = {0, 0};
  if (_readBytes(Register::XG_OFFSET_H, regs, 2)) {
    uint16_t offset = (uint16_t)regs[0] << 8;
    offset |= regs[1];
    xOffset = (int16_t)offset;
    return true;
  } else {
    return false;
  }
}

bool MPU925x::offsetGyroY(const int16_t yOffset) {
  uint8_t regs[2];
  regs[0] = (uint8_t)yOffset << 8;
  regs[1] |= (uint8_t)yOffset;
  return (_writeBytes(Register::YG_OFFSET_H, regs, 2));
}

bool MPU925x::getOffsetGyroY(int16_t &yOffset) const {
  uint8_t regs[2] = {0, 0};
  if (_readBytes(Register::YG_OFFSET_H, regs, 2)) {
    uint16_t offset = (uint16_t)regs[0] << 8;
    offset |= regs[1];
    yOffset = (int16_t)offset;
    return true;
  } else {
    return false;
  }
}

bool MPU925x::offsetGyroZ(const int16_t zOffset) {
  uint8_t regs[2];
  regs[0] = (uint8_t)zOffset << 8;
  regs[1] |= (uint8_t)zOffset;
  return (_writeBytes(Register::ZG_OFFSET_H, regs, 2));
}

bool MPU925x::getOffsetGyroZ(int16_t &zOffset) const {
  uint8_t regs[2] = {0, 0};
  if (_readBytes(Register::ZG_OFFSET_H, regs, 2)) {
    uint16_t offset = (uint16_t)regs[0] << 8;
    offset |= regs[1];
    zOffset = (int16_t)offset;
    return true;
  } else {
    return false;
  }
}


void MPU925x::i2cMaster(const bool enable) {
  _writeBit(Register::USER_CTRL, enable, Bit::I2C_MST_EN);
  _writeBit(Register::INT_PIN_CFG, enable, Bit::BYPASS_EN);
}

void MPU925x::resetI2cMaster() {
  _writeBit(Register::USER_CTRL, true, Bit::I2C_MST_RST);
}

void MPU925x::disableI2c() {
  _writeBit(Register::USER_CTRL, true, Bit::I2C_IF_DIS);
}


bool MPU925x::masterI2cClock(Clock clock) {
  return _writeBits(Register::I2C_MST_CTRL, (uint8_t)clock,
                    Bit::I2C_MST_CLK, Length::I2C_MST_CLK);
}


void MPU925x::fifo(const bool enable) {
  _writeBit(Register::USER_CTRL, enable, Bit::FIFO_EN);
}

void MPU925x::resetFifo() {
  _writeBit(Register::USER_CTRL, enable, Bit::FIFO_RST);
}

void MPU925x::fifoFullOverflow(const bool enable) {
  _writeBit(Register::CONFIG, !enable, Bit::FIFO_MODE);
}

void MPU925x::fifoTemperature(const bool enable) {
  _writeBit(Register::FIFO_EN, enable, Bit::TEMP_FIFO_EN);
}

void MPU925x::fifoGyroX(const bool enable) {
  _writeBit(Register::FIFO_EN, enable, Bit::GYRO_XO_UT);
}

void MPU925x::fifoGyroY(const bool enable) {
  _writeBit(Register::FIFO_EN, enable, Bit::GYRO_YO_UT);
}

void MPU925x::fifoGyroZ(const bool enable) {
  _writeBit(Register::FIFO_EN, enable, Bit::GYRO_ZO_UT);
}

void MPU925x::fifoGyro(const bool enable) {
  uint8_t bits = 0b000;
  if (enable) bits = 0b111;
  _writeBits(Register::FIFO_EN, bits, (Bit)6, (Length)3);
}

void MPU925x::fifoAccel(const bool enable) {
  _writeBit(Register::FIFO_EN, enable, Bit::ACCEL);
}

void MPU925x::fifoMag(const bool enable) {
  _writeBit(Register::FIFO_EN, enable, Bit::SLV0);
}

void MPU925x::fifoSlave0(const bool enable) {
  _writeBit(Register::FIFO_EN, enable, Bit::SLV0);
}

void MPU925x::fifoSlave1(const bool enable) {
  _writeBit(Register::FIFO_EN, enable, Bit::SLV1);
}

void MPU925x::fifoSlave2(const bool enable) {
  _writeBit(Register::FIFO_EN, enable, Bit::SLV2);
}

void MPU925x::fifoSlave3(const bool enable) {
  _writeBit(Register::I2C_MST_CTRL, enable, Bit::SLV_3_FIFO_EN);
}


uint16_t MPU925x::fifoSize() const {
  uint8_t data[2];
  uint16_t fifoSize = 0;
  if (_readBytes(Register::FIFO_COUNTH, data, 2)) {
    fifoSize = (uint16_t)data[0] << 8;
    fifoSize |= data[1];
  }
  return fifoSize;
}

bool MPU925x::readFifo(uint8_t fifo[], const int8_t size) {
  if (_readBytes(Register::FIFO_R_W, fifo, size)) {
    return true;
  } else {
    return false;
  }
}


bool MPU925x::lowPowerAccelOutputDataRate(const OutputDataRate outputDataRate) {
  if ((uint8_t)outputDataRate < 0 || (int8_t)outputDataRate > 11) return false;
  return _writeBits(Register::LP_ACCEL_ODR, (uint8_t)outputDataRate,
                    Bit::LPOSC_CLKSEL, Length::LPOSC_CLKSEL);
}

void MPU925x::cycle(const bool enable) {
  _writeBit(Register::PWR_MGMT_1, enable, Bit::CYCLE);
}

void MPU925x::standbyGyro(const bool enable) {
  _writeBit(Register::PWR_MGMT_1, enable, Bit::GYRO_STANDBY);
}

void MPU925x::powerDownPtat(const bool enable) {
  _writeBit(Register::PWR_MGMT_1, enable, Bit::PD_PTAT);
}



void MPU925x::sensorAccelX(const bool enable) {
  _writeBit(Register::PWR_MGMT_2, !enable, Bit::DISABLE_XA, Length::DISABLE_XA);
}

void MPU925x::sensorAccelY(const bool enable) {
  _writeBit(Register::PWR_MGMT_2, !enable, Bit::DISABLE_YA, Length::DISABLE_YA);
}

void MPU925x::sensorAccelZ(const bool enable) {
  _writeBit(Register::PWR_MGMT_2, !enable, Bit::DISABLE_ZA, Length::DISABLE_ZA);
}

void MPU925x::sensorAccel(const bool state) {
  uint8_t bits = 0b000;
  if (!state) bits = 0b111;
  _writeBits(Register::PWR_MGMT_2, bits, (Bit)5, (Length)3);
}

void MPU925x::sensorGyroX(const bool enable) {
  _writeBit(Register::PWR_MGMT_2, !enable, Bit::DISABLE_XG, Length::DISABLE_XG);
}

void MPU925x::sensorGyroY(const bool enable) {
  _writeBit(Register::PWR_MGMT_2, !enable, Bit::DISABLE_YG, Length::DISABLE_YG);
}

void MPU925x::sensorGyroZ(const bool enable) {
  _writeBit(Register::PWR_MGMT_2, !enable, Bit::DISABLE_ZG, Length::DISABLE_ZG);
}

void MPU925x::sensorGyro(const bool state) {
  uint8_t bits = 0b000;
  if (!state) bits = 0b111;
  _writeBits(Register::PWR_MGMT_2, bits, (Bit)2, (Length)3);
}


void MPU925x::wakeOnMotionInterrupt(const bool enable) {
  _writeBit(Register::INT_ENABLE, enable, Bit::WOM_EN);
}

bool MPU925x::wakeOnMotionTriggered() const {
  bool interrupt = false;
  _readBit(Register::INT_STATUS, &interrupt, Bit::WOM_INT);
  return interrupt;
}

void MPU925x::wakeOnMotionDetection(const bool enable) {
  uint8_t bits = 0b00;
  if (enable) bits = 0b11;
  _writeBits(Register::ACCEL_INTEL_CTRL, bits, (Bit)7, (Length)2);
}

void MPU925x::wakeOnMotionThreshold(const uint16_t mg) {
  uint8_t reg = 0;
  if (mg <= 1020) {
    reg = mg >> 2;
  } else {
    reg = 255;
  }
  _writeBytes(Register::WOM_THR, reg);
}


void MPU925x::fifoOverflowInterrupt(const bool enable) {
  _writeBit(Register::INT_ENABLE, enable, Bit::FIFO_OVERFLOW_EN);
}

bool MPU925x::fifoOverflowTriggered() const {
  bool interrupt = false;
  _readBit(Register::INT_STATUS, &interrupt, Bit::FIFO_OVERFLOW_INT);
  return interrupt;
}


void MPU925x::dataReadyInterrupt(const bool enable) {
  _writeBit(Register::INT_ENABLE, enable, Bit::RAW_RDY_EN);
}

bool MPU925x::dataReadyTriggered() const {
  bool interrupt = false;
  _readBit(Register::INT_STATUS, &interrupt, Bit::RAW_DATA_RDY_INT);
  return interrupt;
}

void MPU925x::waitForExternalDataBeforeTriggering(const bool enable) {
  _writeBit(Register::I2C_MST_CTRL, enable, Bit::WAIT_FOR_ES);
}

void MPU925x::delayShadowingOfExternalData(const bool enable) {
  _writeBit(Register::I2C_MST_DELAY_CTRL, enable, Bit::DELAY_ES_SHADOW);
}


void MPU925x::interruptPinActiveLogicLevel(const bool high) {
  _writeBit(Register::INT_PIN_CFG, !high, Bit::ACTL);
}

void MPU925x::openDrainInterruptPin() {
  _writeBit(Register::INT_PIN_CFG, true, Bit::OPEN);
}

void MPU925x::pushPullInterruptPin() {
  _writeBit(Register::INT_PIN_CFG, false, Bit::OPEN);
}

void MPU925x::latchInterruptPin(const bool enable) {
  _writeBit(Register::INT_PIN_CFG, enable, Bit::LATCH_INT_EN);
}

void MPU925x::anyReadClearsInterruptStatus(const bool enable) {
  _writeBit(Register::INT_PIN_CFG, enable, Bit::INT_ANYRD_2CLEAR);
}

void MPU925x::interruptStatus(bool & wakeOnMotion, bool & fifoOverflow, bool & dataReady) const {
  uint8_t reg = 0;
  _readBytes(Register::INT_STATUS, &reg);
  wakeOnMotion = reg & (1 << Bit::WOM_INT);
  fifoOverflow = reg & (1 << Bit::FIFO_OVERFLOW_INT);
  dataReady = reg & (1 << Bit::RAW_DATA_RDY_INT);
}


void MPU925x::resetAccelDigitalSignalPath() {
  _writeBit(Register::SIGNAL_PATH_RESET, true, Bit::ACCEL_RST);
}

void MPU925x::resetGyroDigitalSignalPath() {
  _writeBit(Register::SIGNAL_PATH_RESET, true, Bit::GYRO_RST);
}

void MPU925x::resetTempDigitalSignalPath() {
  _writeBit(Register::SIGNAL_PATH_RESET, true, Bit::TEMP_RST);
}

void MPU925x::resetAllDigitalSignalPaths() {
  _writeBit(Register::USER_CTRL, true, Bit::SIG_COND_RST);
}


bool MPU925x::readAccel(int16_t &x, int16_t &y, int16_t &z) const {
  uint8_t regs[6];
  if (_readBytes(Register::ACCEL_XOUT_H, regs, 6)) {
    x = ((int16_t)regs[0] << 8) | regs[1];
    // x |= regs[1];
    y = ((int16_t)regs[2] << 8) | regs[3];
    // y |= regs[3];
    z = ((int16_t)regs[4] << 8) | regs[5];
    // z |= regs[5];
    return true;
  } else {
    return false;
  }
}

bool MPU925x::readAccel(int16_t (&accel)[3]) const {
  uint8_t regs[6];
  if (_readBytes(Register::ACCEL_XOUT_H, regs, 6)) {
    accel[0] = (int16_t)regs[0] << 8;
    accel[0] |= regs[1];
    accel[1] = (int16_t)regs[2] << 8;
    accel[1] |= regs[3];
    accel[2] = (int16_t)regs[4] << 8;
    accel[2] |= regs[5];
    return true;
  } else {
    return false;
  }
}

bool MPU925x::readAccelX(int16_t &x) const {
  uint8_t regs[2];
  if (_readBytes(Register::ACCEL_XOUT_H, regs, 2)) {
    x = (int16_t)regs[0] << 8;
    x |= regs[1];
    return true;
  } else {
    return false;
  }
}

bool MPU925x::readAccelY(int16_t &y) const {
  uint8_t regs[2];
  if (_readBytes(Register::ACCEL_YOUT_H, regs, 2)) {
    y = (int16_t)regs[0] << 8;
    y |= regs[1];
    return true;
  } else {
    return false;
  }
}

bool MPU925x::readAccelZ(int16_t &z) const {
  uint8_t regs[2];
  if (_readBytes(Register::ACCEL_ZOUT_H, regs, 2)) {
    z = (int16_t)regs[0] << 8;
    z |= regs[1];
    return true;
  } else {
    return false;
  }
}


bool MPU925x::readGyro(int16_t &x, int16_t &y, int16_t &z) const {
  uint8_t regs[6];
  if (_readBytes(Register::GYRO_XOUT_H, regs, 6)) {
    x = (int16_t)regs[0] << 8;
    x |= regs[1];
    y = (int16_t)regs[2] << 8;
    y |= regs[3];
    z = (int16_t)regs[4] << 8;
    z |= regs[5];
    return true;
  } else {
    return false;
  }
}

bool MPU925x::readGyro(int16_t (&gyro)[3]) const {
  uint8_t regs[6];
  if (_readBytes(Register::GYRO_XOUT_H, regs, 6)) {
    gyro[0] = (int16_t)regs[0] << 8;
    gyro[0] |= regs[1];
    gyro[1] = (int16_t)regs[2] << 8;
    gyro[1] |= regs[3];
    gyro[2] = (int16_t)regs[4] << 8;
    gyro[2] |= regs[5];
    return true;
  } else {
    return false;
  }
}

bool MPU925x::readGyroX(int16_t &x) const {
  uint8_t regs[2];
  if (_readBytes(Register::GYRO_XOUT_H, regs, 2)) {
    x = (int16_t)regs[0] << 8;
    x |= regs[1];
    return true;
  } else {
    return false;
  }
}

bool MPU925x::readGyroY(int16_t &y) const {
  uint8_t regs[2];
  if (_readBytes(Register::GYRO_XOUT_H, regs, 2)) {
    y = (int16_t)regs[0] << 8;
    y |= regs[1];
    return true;
  } else {
    return false;
  }
}

bool MPU925x::readGyroZ(int16_t &z) const {
  uint8_t regs[2];
  if (_readBytes(Register::GYRO_XOUT_H, regs, 2)) {
    z = (int16_t)regs[0] << 8;
    z |= regs[1];
    return true;
  } else {
    return false;
  }
}


bool MPU925x::readMag(int16_t &x, int16_t &y, int16_t &z) const {
  uint8_t regs[6];
  if (_readBytes(Register::EXT_SENS_DATA_00, regs, 6)) {
    x = (int16_t)regs[0] << 8;
    x |= regs[1];
    y = (int16_t)regs[2] << 8;
    y |= regs[3];
    z = (int16_t)regs[4] << 8;
    z |= regs[5];
    return true;
  } else {
    return false;
  }
}

bool MPU925x::readMag(int16_t (&mag)[3]) const {
  uint8_t regs[6];
  if (_readBytes(Register::EXT_SENS_DATA_00, regs, 6)) {
    mag[0] = (int16_t)regs[0] << 8;
    mag[0] |= regs[1];
    mag[1] = (int16_t)regs[2] << 8;
    mag[1] |= regs[3];
    mag[2] = (int16_t)regs[4] << 8;
    mag[2] |= regs[5];
    return true;
  } else {
    return false;
  }
}

bool MPU925x::readMagX(int16_t &x) const {
  uint8_t regs[2];
  if (_readBytes(Register::EXT_SENS_DATA_00, regs, 2)) {
    x = (int16_t)regs[0] << 8;
    x |= regs[1];
    return true;
  } else {
    return false;
  }
}

bool MPU925x::readMagY(int16_t &y) const {
  uint8_t regs[2];
  if (_readBytes(Register::EXT_SENS_DATA_02, regs, 2)) {
    y = (int16_t)regs[0] << 8;
    y |= regs[1];
    return true;
  } else {
    return false;
  }
}

bool MPU925x::readMagZ(int16_t &z) const {
  uint8_t regs[2];
  if (_readBytes(Register::EXT_SENS_DATA_04, regs, 2)) {
    z = (int16_t)regs[0] << 8;
    z |= regs[1];
    return true;
  } else {
    return false;
  }
}


bool MPU925x::readTemp(int16_t &temp) const {
  uint8_t regs[2];
  if (_readBytes(Register::TEMP_OUT_H, regs, 2)) {
    temp = (int16_t)regs[0] << 8;
    temp |= regs[1];
    return true;
  } else {
    return false;
  }
}


bool MPU925x::readExternalData(const int8_t startingAddress, uint8_t externalData[],
                               const int8_t size) const {
  if (size < 0 || size > 24) return false;
  uint8_t regs[size];
  if (read((Register)((uint8_t)Register::EXT_SENS_DATA_00 + startingAddress),
           regs, size)) {
    for (uint8_t i = 0; i < size; ++i) {
      externalData[i] = regs[i];
    }
    return true;
  } else {
    return false;
  }
}

bool MPU925x::readExternalData(const int8_t startingAddress, int8_t externalData[],
                               const int8_t size) const {
  if (size < 0 || size > 24) return false;
  uint8_t regs[size];
  if (read((Register)((uint8_t)Register::EXT_SENS_DATA_00 + startingAddress),
           regs, size)) {
    for (uint8_t i = 0; i < size; ++i) {
      externalData[i] = (int8_t)regs[i];
    }
    return true;
  } else {
    return false;
  }
}

bool MPU925x::readExternalData(const int8_t startingAddress, uint16_t externalData[],
                               const int8_t size) const {
  if (size < 0 || size > 12) return false;
  uint8_t regs[size * 2];
  if (read((Register)((uint8_t)Register::EXT_SENS_DATA_00 + startingAddress),
           regs, size * 2)) {
    // ed0-12, regs0-24
    // ed0 = regs0 regs1
    // ed1 = regs2 regs3
    // ed2 = regs4 regs5
    // ed3 = regs6 regs7
    // ed4 = regs8 regs9
    // ed11 = regs22 regs23
    for (uint8_t i = 0; i < size; ++i) {
      externalData[i] = (uint16_t)regs[i * 2];
      externalData[i] |= regs[i * 2 + 1];
    }
    return true;
  } else {
    return false;
  }
}

bool MPU925x::readExternalData(const int8_t startingAddress, int16_t externalData[],
                               const int8_t size) const {
  if (size < 0 || size > 12) return false;
  uint8_t regs[size * 2];
  if (read((Register)((uint8_t)Register::EXT_SENS_DATA_00 + startingAddress),
           regs, size * 2)) {
    for (uint8_t i = 0; i < size; ++i) {
      externalData[i] = (int16_t)regs[i * 2];
      externalData[i] |= regs[i * 2 + 1];
    }
    return true;
  } else {
    return false;
  }
}


bool MPU925x::readAccelGyro(int16_t &ax, int16_t &ay, int16_t &az,
                            int16_t &gx, int16_t &gy, int16_t &gz) const {
  uint8_t regs[14];
  if (_readBytes(Register::ACCEL_XOUT_H, regs, 14)) {
    ax = (int16_t)regs[0] << 8;
    ax |= regs[1];
    ay = (int16_t)regs[2] << 8;
    ay |= regs[3];
    az = (int16_t)regs[4] << 8;
    az |= regs[5];
    gx = (int16_t)regs[8] << 8;
    gx |= regs[9];
    gy = (int16_t)regs[10] << 8;
    gy |= regs[11];
    gz = (int16_t)regs[12] << 8;
    gz |= regs[13];
    return true;
  } else {
    return false;
  }
}

bool MPU925x::readAccelGyro(int16_t (&accel)[3], int16_t (&gyro)[3]) const {
  uint8_t regs[14];
  if (_readBytes(Register::ACCEL_XOUT_H, regs, 14)) {
    accel[0] = (int16_t)regs[0] << 8;
    accel[0] |= regs[1];
    accel[1] = (int16_t)regs[2] << 8;
    accel[1] |= regs[3];
    accel[2] = (int16_t)regs[4] << 8;
    accel[2] |= regs[5];
    gyro[0] = (int16_t)regs[8] << 8;
    gyro[0] |= regs[9];
    gyro[1] = (int16_t)regs[10] << 8;
    gyro[1] |= regs[11];
    gyro[2] = (int16_t)regs[12] << 8;
    gyro[2] |= regs[13];
    return true;
  } else {
    return false;
  }
}

bool MPU925x::readAccelGyro(int16_t (&accelGyro)[6]) const {
  uint8_t regs[14];
  if (_readBytes(Register::ACCEL_XOUT_H, regs, 14)) {
    accelGyro[0] = (int16_t)regs[0] << 8;
    accelGyro[0] |= regs[1];
    accelGyro[1] = (int16_t)regs[2] << 8;
    accelGyro[1] |= regs[3];
    accelGyro[2] = (int16_t)regs[4] << 8;
    accelGyro[2] |= regs[5];
    accelGyro[3] = (int16_t)regs[8] << 8;
    accelGyro[3] |= regs[9];
    accelGyro[4] = (int16_t)regs[10] << 8;
    accelGyro[4] |= regs[11];
    accelGyro[5] = (int16_t)regs[12] << 8;
    accelGyro[5] |= regs[13];
    return true;
  } else {
    return false;
  }
}


bool MPU925x::readAccelGyroTemp(int16_t &ax, int16_t &ay, int16_t &az,
                                int16_t &gx, int16_t &gy, int16_t &gz,
                                int16_t &temp) const {
  uint8_t regs[14];
  if (_readBytes(Register::ACCEL_XOUT_H, regs, 14)) {
    ax = ((int16_t)regs[0] << 8) | regs[1];
    ay = ((int16_t)regs[2] << 8) | regs[3];
    az = ((int16_t)regs[4] << 8) | regs[5];
    temp = ((int16_t)regs[6] << 8) | regs[7];
    gx = ((int16_t)regs[8] << 8) | regs[9];
    gy = ((int16_t)regs[10] << 8) | regs[11];
    gz = ((int16_t)regs[12] << 8) | regs[13];
    return true;
  } else {
    return false;
  }
}

bool MPU925x::readAccelGyroTemp(int16_t (&accel)[3], int16_t (&gyro)[3], int16_t &temp) const {
  uint8_t regs[14];
  if (_readBytes(Register::ACCEL_XOUT_H, regs, 14)) {
    accel[0] = ((int16_t)regs[0] << 8) | regs[1];
    accel[1] = ((int16_t)regs[2] << 8) | regs[3];
    accel[2] = ((int16_t)regs[4] << 8) | regs[5];
    temp = ((int16_t)regs[6] << 8) | regs[7];
    gyro[0] = ((int16_t)regs[8] << 8) | regs[9];
    gyro[1] = ((int16_t)regs[10] << 8) | regs[11];
    gyro[2] = ((int16_t)regs[12] << 8) | regs[13];
    return true;
  } else {
    return false;
  }
}

bool MPU925x::readAccelGyroTemp(int16_t (&accelGyroTemp)[7]) const {
  uint8_t regs[14];
  if (_readBytes(Register::ACCEL_XOUT_H, regs, 14)) {
    accelGyroTemp[0] = ((int16_t)regs[0] << 8) | regs[1];
    accelGyroTemp[1] = ((int16_t)regs[2] << 8) | regs[3];
    accelGyroTemp[2] = ((int16_t)regs[4] << 8) | regs[5];
    accelGyroTemp[3] = ((int16_t)regs[8] << 8) | regs[9];
    accelGyroTemp[4] = ((int16_t)regs[10] << 8) | regs[11];
    accelGyroTemp[5] = ((int16_t)regs[12] << 8) | regs[13];
    accelGyroTemp[6] = ((int16_t)regs[6] << 8) | regs[7];
    return true;
  } else {
    return false;
  }
}


bool MPU925x::readAccelGyroMag(int16_t &ax, int16_t &ay, int16_t &az,
                               int16_t &gx, int16_t &gy, int16_t &gz,
                               int16_t &mx, int16_t &my, int16_t &mz) const {
  uint8_t regs[20];
  if (_readBytes(Register::ACCEL_XOUT_H, regs, 20)) {
    ax = ((int16_t)regs[0] << 8) | regs[1];
    ay = ((int16_t)regs[2] << 8) | regs[3];
    az = ((int16_t)regs[4] << 8) | regs[5];
    gx = ((int16_t)regs[8] << 8) | regs[9];
    gy = ((int16_t)regs[10] << 8) | regs[11];
    gz = ((int16_t)regs[12] << 8) | regs[13];
    mx = ((int16_t)regs[14] << 8) | regs[15];
    my = ((int16_t)regs[16] << 8) | regs[17];
    mz = ((int16_t)regs[18] << 8) | regs[19];
    return true;
  } else {
    return false;
  }
}

bool MPU925x::readAccelGyroMag(int16_t (&accel)[3], int16_t (&gyro)[3], int16_t (&mag)[3]) const {
  uint8_t regs[20];
  if (_readBytes(Register::ACCEL_XOUT_H, regs, 20)) {
    accel[0] = ((int16_t)regs[0] << 8) | regs[1];
    accel[1] = ((int16_t)regs[2] << 8) | regs[3];
    accel[2] = ((int16_t)regs[4] << 8) | regs[5];
    gyro[0] = ((int16_t)regs[8] << 8) | regs[9];
    gyro[1] = ((int16_t)regs[10] << 8) | regs[11];
    gyro[2] = ((int16_t)regs[12] << 8) | regs[13];
    mag[0] = ((int16_t)regs[14] << 8) | regs[15];
    mag[1] = ((int16_t)regs[16] << 8) | regs[17];
    mag[2] = ((int16_t)regs[18] << 8) | regs[19];
    return true;
  } else {
    return false;
  }
}

bool MPU925x::readAccelGyroMag(int16_t (&accelGyroMag)[9]) const {
  uint8_t regs[20];
  if (_readBytes(Register::ACCEL_XOUT_H, regs, 20)) {
    accelGyroMag[0] = ((int16_t)regs[0] << 8) | regs[1];
    accelGyroMag[1] = ((int16_t)regs[2] << 8) | regs[3];
    accelGyroMag[2] = ((int16_t)regs[4] << 8) | regs[5];
    accelGyroMag[3] = ((int16_t)regs[8] << 8) | regs[9];
    accelGyroMag[4] = ((int16_t)regs[10] << 8) | regs[11];
    accelGyroMag[5] = ((int16_t)regs[12] << 8) | regs[13];
    accelGyroMag[6] = ((int16_t)regs[14] << 8) | regs[15];
    accelGyroMag[7] = ((int16_t)regs[16] << 8) | regs[17];
    accelGyroMag[8] = ((int16_t)regs[18] << 8) | regs[19];
    return true;
  } else {
    return false;
  }
}


bool MPU925x::readAccelGyroMagTemp(int16_t &ax, int16_t &ay, int16_t &az,
                                   int16_t &gx, int16_t &gy, int16_t &gz,
                                   int16_t &mx, int16_t &my, int16_t &mz,
                                   int16_t &temp) const {
  uint8_t regs[20];
  if (_readBytes(Register::ACCEL_XOUT_H, regs, 20)) {
    ax = ((int16_t)regs[0] << 8) | regs[1];
    ay = ((int16_t)regs[2] << 8) | regs[3];
    az = ((int16_t)regs[4] << 8) | regs[5];
    gx = ((int16_t)regs[8] << 8) | regs[9];
    gy = ((int16_t)regs[10] << 8) | regs[11];
    gz = ((int16_t)regs[12] << 8) | regs[13];
    mx = ((int16_t)regs[14] << 8) | regs[15];
    my = ((int16_t)regs[16] << 8) | regs[17];
    mz = ((int16_t)regs[18] << 8) | regs[19];
    temp = ((int16_t)regs[6] << 8) | regs[7];
    return true;
  } else {
    return false;
  }
}

bool MPU925x::readAccelGyroMagTemp(int16_t (&accel)[3], int16_t (&gyro)[3], int16_t (&mag)[3],
                                   int16_t &temp) const {
  uint8_t regs[20];
  if (_readBytes(Register::ACCEL_XOUT_H, regs, 20)) {
    accel[0] = ((int16_t)regs[0] << 8) | regs[1];
    accel[1] = ((int16_t)regs[2] << 8) | regs[3];
    accel[2] = ((int16_t)regs[4] << 8) | regs[5];
    gyro[0] = ((int16_t)regs[8] << 8) | regs[9];
    gyro[1] = ((int16_t)regs[10] << 8) | regs[11];
    gyro[2] = ((int16_t)regs[12] << 8) | regs[13];
    mag[0] = ((int16_t)regs[14] << 8) | regs[15];
    mag[1] = ((int16_t)regs[16] << 8) | regs[17];
    mag[2] = ((int16_t)regs[18] << 8) | regs[19];
    temp = ((int16_t)regs[6] << 8) | regs[7];
    return true;
  } else {
    return false;
  }
}

bool MPU925x::readAccelGyroMagTemp(int16_t (&accelGyroMagTemp)[10]) const {
  uint8_t regs[20];
  if (_readBytes(Register::ACCEL_XOUT_H, regs, 20)) {
    accelGyroMagTemp[0] = ((int16_t)regs[0] << 8) | regs[1];
    accelGyroMagTemp[1] = ((int16_t)regs[2] << 8) | regs[3];
    accelGyroMagTemp[2] = ((int16_t)regs[4] << 8) | regs[5];
    accelGyroMagTemp[3] = ((int16_t)regs[8] << 8) | regs[9];
    accelGyroMagTemp[4] = ((int16_t)regs[10] << 8) | regs[11];
    accelGyroMagTemp[5] = ((int16_t)regs[12] << 8) | regs[13];
    accelGyroMagTemp[6] = ((int16_t)regs[14] << 8) | regs[15];
    accelGyroMagTemp[7] = ((int16_t)regs[16] << 8) | regs[17];
    accelGyroMagTemp[8] = ((int16_t)regs[18] << 8) | regs[19];
    accelGyroMagTemp[9] = ((int16_t)regs[6] << 8) | regs[7];
    return true;
  } else {
    return false;
  }
}


bool MPU925x::read(const int8_t startingAddress, uint8_t data[],
                   const int8_t size) const {
  if (size < 0 || size > 38) return false;
  return _read((Register)((uint8_t)Register::ACCEL_XOUT_H + startingAddress),
               data, size);
}

bool MPU925x::read(const int8_t startingAddress, int16_t data[],
                   const int8_t size) const {
  if (size < 0 || size > 19) return false;
  uint8_t regs[size * 2];
  if (_read((Register)((uint8_t)Register::ACCEL_XOUT_H + startingAddress),
            regs, size * 2)) {
    for (uint8_t i = 0; i < size; ++i) {
      data[i] = (int16_t)regs[i * 2];
      data[i] |= regs[i * 2 + 1];
    }
    return true;
  } else {
    return false;
  }
}
















bool MPU925x::_offsetToRegister(const int16_t offset, uint8_t &registerBytes[2]) const {
  if ((uint16_t)offset & ~0x8000 > 0x3fff) return false; // if bigger than 15-bit unsigned
  uint16_t signMask = (uint16_t)offset & 0x8000; // leave only the sign bit
  uint16_t offset_u16 = offset << 1; // left shift by 1, the sign falls
  offset_u16 &= ~0x8000; // zero the sign bit
  offset_u16 |= signMask; // place the sign bit
  registerBytes[0] = offset_u16 >> 8;
  registerBytes[1] = offset_u16;
  return true;
}

bool MPU925x::_registerToOffset(const uint8_t *const registerBytes, int16_t &offset) const {
  uint16_t value_u16 = (uint16_t)registerBytes[0] << 8; // index 0 is HIGH byte
  value_u16 |= (uint16_t)registerBytes[1];
  uint16_t signMask = value_u16 & 0x8000;
  value_u16 >>= 1; // make it consistent with the setter
  value_u16 &= ~0x8000; // clear the sign bit
  value_u16 |= signMask; // restore the sign bit
  offset = (int16_t)value_u16;
}









bool MPU925x::_readBit(const Register registerAddress, bool *const bit,
                       const Bit position) const {
  uint8_t reg = 0;
  if (_readBytes(registerAddress, &reg)) {
    *bit = reg & (1 << (uint8_t)bit);
    return true;
  } else {
    return false;
  }
}
bool MPU925x::_writeBit(const Register registerAddress, const bool bit,
                        const Bit position) {
  uint8_t reg = 0;
  if (_readBytes(registerAddress, &reg)) {
    if (bit == true) {
      reg |= (1 << (uint8_t)position);
    } else {
      reg &= (1 << (uint8_t)position);
    }
    return _writeBytes(registerAddress, &reg);
  }
}

bool MPU925x::_readBits(const Register registerAddress, uint8_t *const bits,
                        const Bit firstBit, const Length length) const {
  uint8_t reg;
  if (_readBytes(registerAddress, &reg)) {
    uint8_t shift = (bitStart - length) + 1; // shift to correct position
    // bitStart begins from zero, length begins from one
    uint8_t mask = ((1 << length) - 1) << shift; // holds 1s where the bits will be changed
    reg &= mask; // clear the other bits
    reg >>= shift; // shift the target bits to the right
    *bits = reg;
    return true;
  } else {
    return false;
  }
}
bool MPU925x::_writeBits(const Register registerAddress, const uint8_t bits,
                         const Bit firstBit, const Length length) {
  uint8_t reg; // register
  if (_readBytes(registerAddress, &reg)) {
    uint8_t shift = (bitStart - length) + 1;
    uint8_t mask = ((1 << length) - 1) << shift;
    bits <<= shift; // shifts bits to the correct position in the register
    bits &= mask; // clamp numbers bigger than 2^length
    reg &= ~(mask); // clear the bits that will be changed
    reg |= bits; // set bits in register according to bits
    return _writeBytes(registerAddress, reg);
  } else {
    return false;
  }
}

bool MPU925x::_readBytes(const Register registerAddress, uint8_t *const bytes,
                         const int8_t size) const {
  return _read(_device, (uint8_t)registerAddress, byte, size);
}
bool MPU925x::_writeBytes(const Register registerAddress, const uint8_t *const byte,
                          const int8_t size) {
  return _write(_device, (uint8_t)registerAddress, byte, size);
}




float MPU925x::accel_to_g(const int16_t accel) const {
  return accel / (float)_accelSensitivity_g;
}

float MPU925x::accel_to_ms2(const int16_t accel) const {
  return accel / _accelSensitivity_mS2;
}

float MPU925x::gyro_to_radS(const int16_t gyro) const {
  return gyro / _gyroSensitivity_radS;
}

float MPU925x::gyro_to_degS(const int16_t gyro) const {
  return gyro / _gyroSensitivity_degS;
}

float MPU925x::temp_to_c(const int16_t temp) const {
  return temp / 333.87 + 21.0f;
  // Temperature sensitivity = 333.87 LSB/degC
}

float MPU925x::temp_to_f(const int16_t temp) const {
  return ((temp / 333.87 + 21.0f) * 1.8f) + 32.0f;
}
