/*
ICM20602.cpp
Brian R Taylor
brian.taylor@bolderflight.com

Copyright (c) 2017 Bolder Flight Systems

Permission is hereby granted, free of charge, to any person obtaining a copy of this software 
and associated documentation files (the "Software"), to deal in the Software without restriction, 
including without limitation the rights to use, copy, modify, merge, publish, distribute, 
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is 
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or 
substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING 
BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, 
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "Arduino.h"
#include "ICM20602.h"

/* ICM20602 object, input the I2C bus and address */
ICM20602::ICM20602(TwoWire &bus,uint8_t address){
  _i2c = &bus; // I2C bus
  _address = address; // I2C address
  _useSPI = false; // set to use I2C
}

/* ICM20602 object, input the SPI bus and chip select pin */
ICM20602::ICM20602(SPIClass &bus,uint8_t csPin){
  _spi = &bus; // SPI bus
  _csPin = csPin; // chip select pin
  _useSPI = true; // set to use SPI
}

/* starts communication with the MPU-9250 */
int ICM20602::begin(){
  if( _useSPI ) { // using SPI for communication
    // use low speed SPI for register setting
    _useSPIHS = false;
    // setting CS pin to output
    pinMode(_csPin,OUTPUT);
    // setting CS pin high
    digitalWrite(_csPin,HIGH);
    // begin SPI communication
    _spi->begin();
  } else { // using I2C for communication
    // starting the I2C bus
    _i2c->begin();
    // setting the I2C clock
    _i2c->setClock(_i2cRate);
  }
  // select clock source to gyro
  if(writeRegister(PWR_MGMT_1,PWR_CLKSEL_CLK_PLL) < 0){
    return -1;
  }
  // reset the ICM20602
  writeRegister(PWR_MGMT_1,PWR_RESET);
  // wait for MPU-9250 to come back up
  delay(10);
  
  // select clock source to gyro
  if(writeRegister(PWR_MGMT_1,PWR_CLKSEL_CLK_PLL) < 0){
    return -4;
  }
  // check the WHO AM I byte, expected value is 0x12 (decimal 113)
  if(whoAmI() != I_AM_ICM20602){
	Serial1.println(whoAmI());
    return -5;
  }
  // enable accelerometer and gyro
  if(writeRegister(PWR_MGMT_2,ACCEL_GYRO_EN) < 0){
    return -6;
  }
  // setting accel range to 16G as default
  if(writeRegister(ACCEL_CONFIG,ACCEL_FS_SEL_16G) < 0){
    return -7;
  }
  _accelScale = G * 16.0f/32767.5f; // setting the accel scale to 16G
  _accelRange = ACCEL_RANGE_16G;
  // setting the gyro range to 2000DPS as default
  if(writeRegister(GYRO_CONFIG,GYRO_FS_SEL_2000DPS) < 0){
    return -8;
  }
  _gyroScale = 2000.0f/32767.5f * _d2r; // setting the gyro scale to 2000DPS
  _gyroRange = GYRO_RANGE_2000DPS;
  // setting bandwidth to 218Hz as default
  if(writeRegister(ACCEL_CONFIG2, ACCEL_DLPF_EN|A_DLPF_CFG_218) < 0){ 
    return -9;
  }
  // setting gyro bandwidth to 250Hz as default  
  if(writeRegister(CONFIG, FCHOISE_DLPF|GYRO_DLPF_250) < 0){
    return -10;
  }
  _bandwidth = DLPF_BANDWIDTH_250HZ_218HZ;
  // setting the sample rate divider to 0 as default
  if(writeRegister(SMPLRT_DIV,0x00) < 0){ 
    return -11;
  } 
  _srd = 0;

  if(writeRegister(PWR_MGMT_1,PWR_CLKSEL_CLK_PLL) < 0){
    return -19;
  }       
  return 1;
}

/* sets the accelerometer full scale range to values other than default */
int ICM20602::setAccelRange(AccelRange range) {
  // use low speed SPI for register setting
  _useSPIHS = false;
  switch(range) {
    case ACCEL_RANGE_2G: {
      // setting the accel range to 2G
      if(writeRegister(ACCEL_CONFIG,ACCEL_FS_SEL_2G) < 0){
        return -1;
      }
      _accelScale = G * 2.0f/32767.5f; // setting the accel scale to 2G
      break; 
    }
    case ACCEL_RANGE_4G: {
      // setting the accel range to 4G
      if(writeRegister(ACCEL_CONFIG,ACCEL_FS_SEL_4G) < 0){
        return -1;
      }
      _accelScale = G * 4.0f/32767.5f; // setting the accel scale to 4G
      break;
    }
    case ACCEL_RANGE_8G: {
      // setting the accel range to 8G
      if(writeRegister(ACCEL_CONFIG,ACCEL_FS_SEL_8G) < 0){
        return -1;
      }
      _accelScale = G * 8.0f/32767.5f; // setting the accel scale to 8G
      break;
    }
    case ACCEL_RANGE_16G: {
      // setting the accel range to 16G
      if(writeRegister(ACCEL_CONFIG,ACCEL_FS_SEL_16G) < 0){
        return -1;
      }
      _accelScale = G * 16.0f/32767.5f; // setting the accel scale to 16G
      break;
    }
  }
  _accelRange = range;
  return 1;
}

/* sets the gyro full scale range to values other than default */
int ICM20602::setGyroRange(GyroRange range) {
  // use low speed SPI for register setting
  _useSPIHS = false;
  switch(range) {
    case GYRO_RANGE_250DPS: {
      // setting the gyro range to 250DPS
      if(writeRegister(GYRO_CONFIG,GYRO_FS_SEL_250DPS) < 0){
        return -1;
      }
      _gyroScale = 250.0f/32767.5f * _d2r; // setting the gyro scale to 250DPS
      break;
    }
    case GYRO_RANGE_500DPS: {
      // setting the gyro range to 500DPS
      if(writeRegister(GYRO_CONFIG,GYRO_FS_SEL_500DPS) < 0){
        return -1;
      }
      _gyroScale = 500.0f/32767.5f * _d2r; // setting the gyro scale to 500DPS
      break;  
    }
    case GYRO_RANGE_1000DPS: {
      // setting the gyro range to 1000DPS
      if(writeRegister(GYRO_CONFIG,GYRO_FS_SEL_1000DPS) < 0){
        return -1;
      }
      _gyroScale = 1000.0f/32767.5f * _d2r; // setting the gyro scale to 1000DPS
      break;
    }
    case GYRO_RANGE_2000DPS: {
      // setting the gyro range to 2000DPS
      if(writeRegister(GYRO_CONFIG,GYRO_FS_SEL_2000DPS) < 0){
        return -1;
      }
      _gyroScale = 2000.0f/32767.5f * _d2r; // setting the gyro scale to 2000DPS
      break;
    }
  }
  _gyroRange = range;
  return 1;
}

/* sets the DLPF bandwidth to values other than default */
int ICM20602::setDlpfBandwidth(DlpfBandwidth bandwidth) {
  // use low speed SPI for register setting
  _useSPIHS = false;
  switch(bandwidth) {
    case DLPF_BANDWIDTH_250HZ_218HZ: {
      if(writeRegister(ACCEL_CONFIG2,A_DLPF_CFG_218) < 0){ // setting accel bandwidth to 218.1Hz
        return -1;
      } 
      if(writeRegister(CONFIG,GYRO_DLPF_250) < 0){ // setting gyro bandwidth to 250Hz
        return -2;
      }
      break;
    }
    case DLPF_BANDWIDTH_176HZ_218HZ: {
      if(writeRegister(ACCEL_CONFIG2,A_DLPF_CFG_218) < 0){ // setting accel bandwidth to 218.1Hz
        return -1;
      } 
      if(writeRegister(CONFIG,GYRO_DLPF_176) < 0){ // setting gyro bandwidth to 176Hz
        return -2;
      }
      break;
    }
    case DLPF_BANDWIDTH_92HZ_99HZ: {
      if(writeRegister(ACCEL_CONFIG2,A_DLPF_CFG_99) < 0){ // setting accel bandwidth to 99Hz
        return -1;
      } 
      if(writeRegister(CONFIG,GYRO_DLPF_92) < 0){ // setting gyro bandwidth to 41Hz
        return -2;
      }
      break;
    }
    case DLPF_BANDWIDTH_41HZ_45HZ: {
      if(writeRegister(ACCEL_CONFIG2,A_DLPF_CFG_45) < 0){ // setting accel bandwidth to 44.8Hz
        return -1;
      } 
      if(writeRegister(CONFIG,GYRO_DLPF_41) < 0){ // setting gyro bandwidth to 41Hz
        return -2;
      }
      break;
    }
    case DLPF_BANDWIDTH_20HZ_21HZ: {
      if(writeRegister(ACCEL_CONFIG2,A_DLPF_CFG_21) < 0){ // setting accel bandwidth to 21.1Hz
        return -1;
      } 
      if(writeRegister(CONFIG,GYRO_DLPF_20) < 0){ // setting gyro bandwidth to 20Hz
        return -2;
      }
      break;
    }
    case DLPF_BANDWIDTH_10HZ_10HZ: {
      if(writeRegister(ACCEL_CONFIG2,A_DLPF_CFG_10) < 0){ // setting accel bandwidth to 10.2Hz
        return -1;
      } 
      if(writeRegister(CONFIG,GYRO_DLPF_10) < 0){ // setting gyro bandwidth to 10Hz
        return -2;
      }
      break;
    }
    case DLPF_BANDWIDTH_5HZ_5HZ: {
      if(writeRegister(ACCEL_CONFIG2,A_DLPF_CFG_5) < 0){ // setting accel bandwidth to 5.1Hz
        return -1;
      } 
      if(writeRegister(CONFIG,GYRO_DLPF_5) < 0){ // setting gyro bandwidth to 5Hz
        return -2;
      }
      break;
    }
    case DLPF_BANDWIDTH_3281HZ_420HZ: {
      if(writeRegister(ACCEL_CONFIG2,A_DLPF_CFG_420) < 0){ // setting accel bandwidth to 420Hz
        return -1;
      } 
      if(writeRegister(CONFIG,GYRO_DLPF_3281) < 0){ // setting gyro bandwidth to 3281Hz
        return -2;
      }
      break;
    }
  }
  _bandwidth = bandwidth;
  return 1;
}

/* sets the sample rate divider to values other than default */
int ICM20602::setSrd(uint8_t srd) {
  // use low speed SPI for register setting
  _useSPIHS = false;
  /* setting the sample rate divider */
  if(writeRegister(SMPLRT_DIV,srd) < 0){ // setting the sample rate divider
    return -4;
  } 
  _srd = srd;
  return 1; 
}

/* enables the data ready interrupt */
int ICM20602::enableDataReadyInterruptFSYNC() {
  // use low speed SPI for register setting
  _useSPIHS = false;
  /* setting the interrupt */
  if (writeRegister(INT_PIN_CFG,FSYNC_INT_MODE_EN) < 0){ 	// setup interrupt, 50 us pulse
    return -1;
  }  
  if (writeRegister(INT_ENABLE,FSYNC_INT_EN) < 0){       	// FSYNC interrupt
    return -2;
  }
  
  return 1;
}

/* disables the data ready interrupt */
int ICM20602::disableDataReadyInterrupt() {
  // use low speed SPI for register setting
  _useSPIHS = false;
  if(writeRegister(INT_PIN_CFG,INT_DISABLE) < 0){ // disable interrupt
    return -1;
  } 
  if(writeRegister(INT_ENABLE,INT_DISABLE) < 0){ // disable interrupt
    return -2;
  }   
  return 1;
}

/* reads the most current data from ICM20602 and stores in buffer */
int ICM20602::readSensor() {
  _useSPIHS = true; // use the high speed SPI for data readout
  // grab the data from the ICM20602
  if (readRegisters(ACCEL_XOUT_H, 14, _buffer) < 0) {
    return -1;
  }
  // combine into 16 bit values
  _axcounts = (((int16_t)_buffer[0]) << 8) | _buffer[1];  
  _aycounts = (((int16_t)_buffer[2]) << 8) | _buffer[3];
  _azcounts = (((int16_t)_buffer[4]) << 8) | _buffer[5];
  _tcounts = (((int16_t)_buffer[6]) << 8) | _buffer[7];
  _gxcounts = (((int16_t)_buffer[8]) << 8) | _buffer[9];
  _gycounts = (((int16_t)_buffer[10]) << 8) | _buffer[11];
  _gzcounts = (((int16_t)_buffer[12]) << 8) | _buffer[13];
  // transform and convert to float values
  _ax = (((float)(tX[0]*_axcounts + tX[1]*_aycounts + tX[2]*_azcounts) * _accelScale) - _axb)*_axs;
  _ay = (((float)(tY[0]*_axcounts + tY[1]*_aycounts + tY[2]*_azcounts) * _accelScale) - _ayb)*_ays;
  _az = (((float)(tZ[0]*_axcounts + tZ[1]*_aycounts + tZ[2]*_azcounts) * _accelScale) - _azb)*_azs;
  _gx = ((float)(tX[0]*_gxcounts + tX[1]*_gycounts + tX[2]*_gzcounts) * _gyroScale) - _gxb;
  _gy = ((float)(tY[0]*_gxcounts + tY[1]*_gycounts + tY[2]*_gzcounts) * _gyroScale) - _gyb;
  _gz = ((float)(tZ[0]*_gxcounts + tZ[1]*_gycounts + tZ[2]*_gzcounts) * _gyroScale) - _gzb;
  _t = ((((float) _tcounts) - _tempOffset)/_tempScale) + _tempOffset;
  return 1;
}

/* reads the most current data from ICM20602 and stores in buffer */
int ICM20602::readAccelGyro() {
  _useSPIHS = true; // use the high speed SPI for data readout
  // grab the data from the ICM20602
  if (readRegisters(ACCEL_XOUT_H, 14, _buffer) < 0) {
    return -1;
  }
  // combine into 16 bit values
  _axcounts = (((int16_t)_buffer[0]) << 8) | _buffer[1];  
  _aycounts = (((int16_t)_buffer[2]) << 8) | _buffer[3];
  _azcounts = (((int16_t)_buffer[4]) << 8) | _buffer[5];
  _tcounts = (((int16_t)_buffer[6]) << 8) | _buffer[7];
  _gxcounts = (((int16_t)_buffer[8]) << 8) | _buffer[9];
  _gycounts = (((int16_t)_buffer[10]) << 8) | _buffer[11];
  _gzcounts = (((int16_t)_buffer[12]) << 8) | _buffer[13];

  // transform and convert to float values
  _ax = (((float)(tX[0]*_axcounts + tX[1]*_aycounts + tX[2]*_azcounts) * _accelScale) - _axb)*_axs;
  _ay = (((float)(tY[0]*_axcounts + tY[1]*_aycounts + tY[2]*_azcounts) * _accelScale) - _ayb)*_ays;
  _az = (((float)(tZ[0]*_axcounts + tZ[1]*_aycounts + tZ[2]*_azcounts) * _accelScale) - _azb)*_azs;
  _gx = ((float)(tX[0]*_gxcounts + tX[1]*_gycounts + tX[2]*_gzcounts) * _gyroScale) - _gxb;
  _gy = ((float)(tY[0]*_gxcounts + tY[1]*_gycounts + tY[2]*_gzcounts) * _gyroScale) - _gyb;
  _gz = ((float)(tZ[0]*_gxcounts + tZ[1]*_gycounts + tZ[2]*_gzcounts) * _gyroScale) - _gzb;
  _t = ((((float) _tcounts) - _tempOffset)/_tempScale) + _tempOffset;
  return 1;
}

/* returns the accelerometer measurement in the x direction, m/s/s */
float ICM20602::getAccelX_mss() {
  return _ax;
}

/* returns the accelerometer measurement in the y direction, m/s/s */
float ICM20602::getAccelY_mss() {
  return _ay;
}

/* returns the accelerometer measurement in the z direction, m/s/s */
float ICM20602::getAccelZ_mss() {
  return _az;
}

/* returns the gyroscope measurement in the x direction, rad/s */
float ICM20602::getGyroX_rads() {
  return _gx;
}

/* returns the gyroscope measurement in the y direction, rad/s */
float ICM20602::getGyroY_rads() {
  return _gy;
}

/* returns the gyroscope measurement in the z direction, rad/s */
float ICM20602::getGyroZ_rads() {
  return _gz;
}

/* returns the die temperature, C */
float ICM20602::getTemperature_C() {
  return _t;
}

/* estimates the gyro biases */
int ICM20602::calibrateGyro() {
  // set the range, bandwidth, and srd
  if (setGyroRange(GYRO_RANGE_250DPS) < 0) {
    return -1;
  }
  if (setDlpfBandwidth(DLPF_BANDWIDTH_10HZ_10HZ) < 0) {
    return -2;
  }
  if (setSrd(19) < 0) {
    return -3;
  }

  // take samples and find bias
  _gxbD = 0;
  _gybD = 0;
  _gzbD = 0;
  for (size_t i=0; i < _numSamples; i++) {
    readSensor();
    _gxbD += (getGyroX_rads() + _gxb)/((double)_numSamples);
    _gybD += (getGyroY_rads() + _gyb)/((double)_numSamples);
    _gzbD += (getGyroZ_rads() + _gzb)/((double)_numSamples);
    delay(20);
  }
  _gxb = (float)_gxbD;
  _gyb = (float)_gybD;
  _gzb = (float)_gzbD;

  // set the range, bandwidth, and srd back to what they were
  if (setGyroRange(_gyroRange) < 0) {
    return -4;
  }
  if (setDlpfBandwidth(_bandwidth) < 0) {
    return -5;
  }
  if (setSrd(_srd) < 0) {
    return -6;
  }
  return 1;
}

/* returns the gyro bias in the X direction, rad/s */
float ICM20602::getGyroBiasX_rads() {
  return _gxb;
}

/* returns the gyro bias in the Y direction, rad/s */
float ICM20602::getGyroBiasY_rads() {
  return _gyb;
}

/* returns the gyro bias in the Z direction, rad/s */
float ICM20602::getGyroBiasZ_rads() {
  return _gzb;
}

/* sets the gyro bias in the X direction to bias, rad/s */
void ICM20602::setGyroBiasX_rads(float bias) {
  _gxb = bias;
}

/* sets the gyro bias in the Y direction to bias, rad/s */
void ICM20602::setGyroBiasY_rads(float bias) {
  _gyb = bias;
}

/* sets the gyro bias in the Z direction to bias, rad/s */
void ICM20602::setGyroBiasZ_rads(float bias) {
  _gzb = bias;
}

/* finds bias and scale factor calibration for the accelerometer,
this should be run for each axis in each direction (6 total) to find
the min and max values along each */
int ICM20602::calibrateAccel() {
  // set the range, bandwidth, and srd
  if (setAccelRange(ACCEL_RANGE_2G) < 0) {
    return -1;
  }
  if (setDlpfBandwidth(DLPF_BANDWIDTH_20HZ_21HZ) < 0) {
    return -2;
  }
  if (setSrd(19) < 0) {
    return -3;
  }

  // take samples and find min / max 
  _axbD = 0;
  _aybD = 0;
  _azbD = 0;
  for (size_t i=0; i < _numSamples; i++) {
    readSensor();
    _axbD += (getAccelX_mss()/_axs + _axb)/((double)_numSamples);
    _aybD += (getAccelY_mss()/_ays + _ayb)/((double)_numSamples);
    _azbD += (getAccelZ_mss()/_azs + _azb)/((double)_numSamples);
    delay(20);
  }
  if (_axbD > 9.0f) {
    _axmax = (float)_axbD;
  }
  if (_aybD > 9.0f) {
    _aymax = (float)_aybD;
  }
  if (_azbD > 9.0f) {
    _azmax = (float)_azbD;
  }
  if (_axbD < -9.0f) {
    _axmin = (float)_axbD;
  }
  if (_aybD < -9.0f) {
    _aymin = (float)_aybD;
  }
  if (_azbD < -9.0f) {
    _azmin = (float)_azbD;
  }

  // find bias and scale factor
  if ((abs(_axmin) > 9.0f) && (abs(_axmax) > 9.0f)) {
    _axb = (_axmin + _axmax) / 2.0f;
    _axs = G/((abs(_axmin) + abs(_axmax)) / 2.0f);
  }
  if ((abs(_aymin) > 9.0f) && (abs(_aymax) > 9.0f)) {
    _ayb = (_axmin + _axmax) / 2.0f;
    _ays = G/((abs(_aymin) + abs(_aymax)) / 2.0f);
  }
  if ((abs(_azmin) > 9.0f) && (abs(_azmax) > 9.0f)) {
    _azb = (_azmin + _azmax) / 2.0f;
    _azs = G/((abs(_azmin) + abs(_azmax)) / 2.0f);
  }

  // set the range, bandwidth, and srd back to what they were
  if (setAccelRange(_accelRange) < 0) {
    return -4;
  }
  if (setDlpfBandwidth(_bandwidth) < 0) {
    return -5;
  }
  if (setSrd(_srd) < 0) {
    return -6;
  }
  return 1;  
}

/* returns the accelerometer bias in the X direction, m/s/s */
float ICM20602::getAccelBiasX_mss() {
  return _axb;
}

/* returns the accelerometer scale factor in the X direction */
float ICM20602::getAccelScaleFactorX() {
  return _axs;
}

/* returns the accelerometer bias in the Y direction, m/s/s */
float ICM20602::getAccelBiasY_mss() {
  return _ayb;
}

/* returns the accelerometer scale factor in the Y direction */
float ICM20602::getAccelScaleFactorY() {
  return _ays;
}

/* returns the accelerometer bias in the Z direction, m/s/s */
float ICM20602::getAccelBiasZ_mss() {
  return _azb;
}

/* returns the accelerometer scale factor in the Z direction */
float ICM20602::getAccelScaleFactorZ() {
  return _azs;
}

/* sets the accelerometer bias (m/s/s) and scale factor in the X direction */
void ICM20602::setAccelCalX(float bias,float scaleFactor) {
  _axb = bias;
  _axs = scaleFactor;
}

/* sets the accelerometer bias (m/s/s) and scale factor in the Y direction */
void ICM20602::setAccelCalY(float bias,float scaleFactor) {
  _ayb = bias;
  _ays = scaleFactor;
}

/* sets the accelerometer bias (m/s/s) and scale factor in the Z direction */
void ICM20602::setAccelCalZ(float bias,float scaleFactor) {
  _azb = bias;
  _azs = scaleFactor;
}

/* writes a byte to ICM20602 register given a register address and data */
int ICM20602::writeRegister(uint8_t subAddress, uint8_t data){
  /* write data to device */
  if( _useSPI ){
    _spi->beginTransaction(SPISettings(SPI_LS_CLOCK, MSBFIRST, SPI_MODE3)); // begin the transaction
    digitalWrite(_csPin,LOW); // select the ICM20602 chip
    _spi->transfer(subAddress); // write the register address
    _spi->transfer(data); // write the data
    digitalWrite(_csPin,HIGH); // deselect the ICM20602 chip
    _spi->endTransaction(); // end the transaction
  }
  else{
    _i2c->beginTransmission(_address); // open the device
    _i2c->write(subAddress); // write the register address
    _i2c->write(data); // write the data
    _i2c->endTransmission();
  }

  delay(10);
  
  /* read back the register */
  readRegisters(subAddress,1,_buffer);
  /* check the read back register against the written register */
  if(_buffer[0] == data) {
    return 1;
  }
  else{
    return -1;
  }
}

/* reads registers from ICM20602 given a starting register address, number of bytes, and a pointer to store data */
int ICM20602::readRegisters(uint8_t subAddress, uint8_t count, uint8_t* dest){
  if( _useSPI ){
    // begin the transaction
    if(_useSPIHS){
      _spi->beginTransaction(SPISettings(SPI_HS_CLOCK, MSBFIRST, SPI_MODE3));
    }
    else{
      _spi->beginTransaction(SPISettings(SPI_LS_CLOCK, MSBFIRST, SPI_MODE3));
    }
    digitalWrite(_csPin,LOW); // select the ICM20602 chip
    _spi->transfer(subAddress | SPI_READ); // specify the starting register address
    for(uint8_t i = 0; i < count; i++){
      dest[i] = _spi->transfer(0x00); // read the data
    }
    digitalWrite(_csPin,HIGH); // deselect the ICM20602 chip
    _spi->endTransaction(); // end the transaction
    return 1;
  }
  else{
    _i2c->beginTransmission(_address); // open the device
    _i2c->write(subAddress); // specify the starting register address
    _i2c->endTransmission(false);
    _numBytes = _i2c->requestFrom(_address, count); // specify the number of bytes to receive
    if (_numBytes == count) {
      for(uint8_t i = 0; i < count; i++){ 
        dest[i] = _i2c->read();
      }
      return 1;
    } else {
      return -1;
    }
  }
}

/* gets the ICM20602 WHO_AM_I register value, expected to be 0x71 */
int ICM20602::whoAmI(){
  // read the WHO AM I register
  if (readRegisters(WHO_AM_I,1,_buffer) < 0) {
    return -1;
  }
  // return the register value
  return _buffer[0];
}