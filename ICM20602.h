/*
ICM20602.h
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

#ifndef ICM20602_h
#define ICM20602_h

#include "Arduino.h"
#include "Wire.h"    // I2C library
#include "SPI.h"     // SPI library

class ICM20602{
  public:
    enum GyroRange
    {
      GYRO_RANGE_250DPS,
      GYRO_RANGE_500DPS,
      GYRO_RANGE_1000DPS,
      GYRO_RANGE_2000DPS
    };
    enum AccelRange
    {
      ACCEL_RANGE_2G,
      ACCEL_RANGE_4G,
      ACCEL_RANGE_8G,
      ACCEL_RANGE_16G    
    };
    enum DlpfBandwidth
    {
      DLPF_BANDWIDTH_250HZ_218HZ,
      DLPF_BANDWIDTH_176HZ_218HZ,
      DLPF_BANDWIDTH_92HZ_99HZ,
      DLPF_BANDWIDTH_41HZ_45HZ,
      DLPF_BANDWIDTH_20HZ_21HZ,
      DLPF_BANDWIDTH_10HZ_10HZ,
	  DLPF_BANDWIDTH_5HZ_5HZ,
	  DLPF_BANDWIDTH_3281HZ_420HZ
    };
    enum LpAccelOdr
    {
      LP_ACCEL_ODR_0_24HZ = 0,
      LP_ACCEL_ODR_0_49HZ = 1,
      LP_ACCEL_ODR_0_98HZ = 2,
      LP_ACCEL_ODR_1_95HZ = 3,
      LP_ACCEL_ODR_3_91HZ = 4,
      LP_ACCEL_ODR_7_81HZ = 5,
      LP_ACCEL_ODR_15_63HZ = 6,
      LP_ACCEL_ODR_31_25HZ = 7,
      LP_ACCEL_ODR_62_50HZ = 8,
      LP_ACCEL_ODR_125HZ = 9,
      LP_ACCEL_ODR_250HZ = 10,
      LP_ACCEL_ODR_500HZ = 11
    };
    ICM20602(TwoWire &bus,uint8_t address);
    ICM20602(SPIClass &bus,uint8_t csPin);
    int begin();
    int setAccelRange(AccelRange range);
    int setGyroRange(GyroRange range);
    int setDlpfBandwidth(DlpfBandwidth bandwidth);
    int setSrd(uint8_t srd);
    int enableDataReadyInterruptFSYNC();
    int disableDataReadyInterrupt();
    int enableWakeOnMotion(float womThresh_mg,LpAccelOdr odr);
    int readSensor();
	int readAccelGyro();
    float getAccelX_mss();
    float getAccelY_mss();
    float getAccelZ_mss();
    float getGyroX_rads();
    float getGyroY_rads();
    float getGyroZ_rads();
    float getTemperature_C();
    
    int calibrateGyro();
    float getGyroBiasX_rads();
    float getGyroBiasY_rads();
    float getGyroBiasZ_rads();
    void setGyroBiasX_rads(float bias);
    void setGyroBiasY_rads(float bias);
    void setGyroBiasZ_rads(float bias);
    int calibrateAccel();
    float getAccelBiasX_mss();
    float getAccelScaleFactorX();
    float getAccelBiasY_mss();
    float getAccelScaleFactorY();
    float getAccelBiasZ_mss();
    float getAccelScaleFactorZ();
    void setAccelCalX(float bias,float scaleFactor);
    void setAccelCalY(float bias,float scaleFactor);
    void setAccelCalZ(float bias,float scaleFactor);
  protected:
    // i2c
    uint8_t _address;
    TwoWire *_i2c;
    const uint32_t _i2cRate = 400000; // 400 kHz
    size_t _numBytes; // number of bytes received from I2C
    // spi
    SPIClass *_spi;
    uint8_t _csPin;
    bool _useSPI;
    bool _useSPIHS;
    const uint8_t SPI_READ = 0x80;
    const uint32_t SPI_LS_CLOCK = 1000000;  // 1 MHz
    const uint32_t SPI_HS_CLOCK = 10000000; // 10 MHz
    // track success of interacting with sensor
    int _status;
    // buffer for reading from sensor
    uint8_t _buffer[21];
    // data counts
    int16_t _axcounts,_aycounts,_azcounts;
    int16_t _gxcounts,_gycounts,_gzcounts;
    int16_t _tcounts;
    // data buffer
    float _ax, _ay, _az;
    float _gx, _gy, _gz;
    float _t;
    // wake on motion
    uint8_t _womThreshold;
    // scale factors
    float _accelScale;
    float _gyroScale;
    const float _tempScale = 333.87f;
    const float _tempOffset = 21.0f;
    // configuration
    AccelRange _accelRange;
    GyroRange _gyroRange;
    DlpfBandwidth _bandwidth;
    uint8_t _srd;
    // gyro bias estimation
    size_t _numSamples = 1000;
    double _gxbD, _gybD, _gzbD;
    float _gxb, _gyb, _gzb;
    // accel bias and scale factor estimation
    double _axbD, _aybD, _azbD;
    float _axmax, _aymax, _azmax;
    float _axmin, _aymin, _azmin;
    float _axb, _ayb, _azb;
    float _axs = 1.0f;
    float _ays = 1.0f;
    float _azs = 1.0f;
    // transformation matrix
    /* transform the accel and gyro axes to match the magnetometer axes */
    const int16_t tX[3] = {1,  0,  0}; 
    const int16_t tY[3] = {0,  1,  0};
    const int16_t tZ[3] = {0,  0,  1};
    // constants
    const float G = 9.807232f;
    const float _d2r = 3.14159265359f/180.0f;
	
    // ICM20602 registers
	const uint8_t XG_OFFS_TC_H = 0x04;
	const uint8_t XG_OFFS_TC_L = 0x05;
	const uint8_t YG_OFFS_TC_H = 0x07;
	const uint8_t YG_OFFS_TC_L = 0x08;
	const uint8_t ZG_OFFS_TC_H = 0x0A;
	const uint8_t ZG_OFFS_TC_L = 0x0B;
	const uint8_t SELF_TEST_X_ACCEL = 0x0D;
	const uint8_t SELF_TEST_Y_ACCEL = 0x0E;
	const uint8_t SELF_TEST_Z_ACCEL = 0x0F;
	const uint8_t XG_OFFS_USRH = 0x13;
	const uint8_t XG_OFFS_USRL = 0x14;
	const uint8_t YG_OFFS_USRH = 0x15;
	const uint8_t YG_OFFS_USRL = 0x16;
	const uint8_t ZG_OFFS_USRH = 0x17;
	const uint8_t ZG_OFFS_USRL = 0x18;
	const uint8_t SMPLRT_DIV = 0x19;    				// SAMPLE_RATE = INTERNAL_SAMPLE_RATE / (1 + SMPLRT_DIV) where  INTERNAL_SAMPLE_RATE = 1 kHz
	const uint8_t CONFIG = 0x1A;
	const uint8_t GYRO_CONFIG = 0x1B;
	const uint8_t ACCEL_CONFIG = 0x1C;
	const uint8_t ACCEL_CONFIG2 = 0x1D;
	const uint8_t LP_MODE_CFG = 0x1E;
	const uint8_t ACCEL_WOM_X_THR = 0x20;
	const uint8_t ACCEL_WOM_Y_THR = 0x21;
	const uint8_t ACCEL_WOM_Z_THR = 0x22;
	const uint8_t FIFO_EN = 0x23;
	const uint8_t FSYNC_INT = 0x36;
	const uint8_t INT_PIN_CFG = 0x37;
	const uint8_t INT_ENABLE = 0x38;
	const uint8_t FIFO_WM_INT_STATUS = 0x39;
	const uint8_t INT_STATUS = 0x3A;
	const uint8_t ACCEL_XOUT_H = 0x3B;
	const uint8_t ACCEL_XOUT_L = 0x3C;
	const uint8_t ACCEL_YOUT_H = 0x3D;
	const uint8_t ACCEL_YOUT_L = 0x3E;
	const uint8_t ACCEL_ZOUT_H = 0x3F;
	const uint8_t ACCEL_ZOUT_L = 0x40;
	const uint8_t TEMP_OUT_H = 0x41;
	const uint8_t TEMP_OUT_L = 0x42;
	const uint8_t GYRO_XOUT_H = 0x43;
	const uint8_t GYRO_XOUT_L = 0x44;
	const uint8_t GYRO_YOUT_H = 0x45;
	const uint8_t GYRO_YOUT_L = 0x46;
	const uint8_t GYRO_ZOUT_H = 0x47;
	const uint8_t GYRO_ZOUT_L = 0x48;
	const uint8_t SELF_TEST_X_GYRO = 0x50;
	const uint8_t SELF_TEST_Y_GYRO = 0x51;
	const uint8_t SELF_TEST_Z_GYRO = 0x52;
	const uint8_t FIFO_WM_TH1 = 0x60;
	const uint8_t FIFO_WM_TH2 = 0x61;
	const uint8_t SIGNAL_PATH_RESET = 0x68;
	const uint8_t ACCEL_INTEL_CTRL = 0x69;
	const uint8_t USER_CTRL = 0x6A;
	const uint8_t PWR_MGMT_1 = 0x6B;
	const uint8_t PWR_MGMT_2 = 0x6C;
	const uint8_t I2C_IF = 0x70;
	const uint8_t FIFO_COUNTH = 0x72;
	const uint8_t FIFO_COUNTL = 0x73;
	const uint8_t FIFO_R_W = 0x74;
	const uint8_t WHO_AM_I = 0x75;
	const uint8_t XA_OFFSET_H = 0x77;
	const uint8_t XA_OFFSET_L = 0x78;
	const uint8_t YA_OFFSET_H = 0x7A;
	const uint8_t YA_OFFSET_L = 0x7B;
	const uint8_t ZA_OFFSET_H = 0x7D;
	const uint8_t ZA_OFFSET_L = 0x7E;
	
	// Bit definition for configuration registers

	// Define CONFIG
	const uint8_t SET_TO_ZERO_INIT = 0x80;
	const uint8_t FIFO_MODE = 0x40;
	const uint8_t EXT_SYNC_SET_DISABLED = 0x00;
	const uint8_t EXT_SYNC_SET_TEMP_OUT = 0x08;
	const uint8_t EXT_SYNC_SET_GYRO_XOUT = 0x10;
	const uint8_t EXT_SYNC_SET_GYRO_YOUT = 0x18;
	const uint8_t EXT_SYNC_SET_GYRO_ZOUT = 0x20;
	const uint8_t EXT_SYNC_SET_ACCEL_XOUT = 0x28;
	const uint8_t EXT_SYNC_SET_ACCEL_YOUT = 0x30;
	const uint8_t EXT_SYNC_SET_ACCEL_ZOUT = 0x38;
	const uint8_t GYRO_DLPF_250 = 0x00;
	const uint8_t GYRO_DLPF_176 = 0x01;
	const uint8_t GYRO_DLPF_92 = 0x02;
	const uint8_t GYRO_DLPF_41 = 0x03;
	const uint8_t GYRO_DLPF_20 = 0x04;
	const uint8_t GYRO_DLPF_10 = 0x05;
	const uint8_t GYRO_DLPF_5 = 0x06;
	const uint8_t GYRO_DLPF_3281 = 0x07;	
	// End Define CONFIG
	
	// Define GYRO_CONFIG
	const uint8_t X_GYRO_SELF_TEST = 0x80;
	const uint8_t Y_GYRO_SELF_TEST = 0x40;
	const uint8_t Z_GYRO_SELF_TEST = 0x20;
	const uint8_t GYRO_FS_SEL_250DPS = 0x00;
	const uint8_t GYRO_FS_SEL_500DPS = 0x80;
	const uint8_t GYRO_FS_SEL_1000DPS = 0x10;
	const uint8_t GYRO_FS_SEL_2000DPS = 0x18;
	const uint8_t FCHOISE_NO_DLPF_8173 = 0x01;
	const uint8_t FCHOISE_NO_DLPF_3281 = 0x02;
	const uint8_t FCHOISE_DLPF = 0x00;	
	// End Define GYRO_CONFIG
	
	// Define ACCEL_CONFIG
	const uint8_t X_ACCEL_SELF_TEST = 0x80;
	const uint8_t Y_ACCEL_SELF_TEST = 0x40;
	const uint8_t Z_ACCEL_SELF_TEST = 0x20;
	const uint8_t ACCEL_FS_SEL_2G = 0x00;
	const uint8_t ACCEL_FS_SEL_4G = 0x80;
	const uint8_t ACCEL_FS_SEL_8G = 0x10;
	const uint8_t ACCEL_FS_SEL_16G = 0x18;	
	// End Define ACCEL_CONFIG
	
	// Define ACCEL_CONFIG2
	const uint8_t DEC2_CFG_AVG_4 = 0x00;
	const uint8_t DEC2_CFG_AVG_8 = 0x10;
	const uint8_t DEC2_CFG_AVG_16 = 0x20;
	const uint8_t DEC2_CFG_AVG_32 = 0x30;
	const uint8_t ACCEL_DLPF_EN = 0x00;
	const uint8_t ACCEL_DLPF_DIS = 0x04;
	const uint8_t A_DLPF_CFG_218 = 0x01;
	const uint8_t A_DLPF_CFG_99 = 0x02;
	const uint8_t A_DLPF_CFG_45 = 0x03;
	const uint8_t A_DLPF_CFG_21 = 0x04;
	const uint8_t A_DLPF_CFG_10 = 0x05;
	const uint8_t A_DLPF_CFG_5 = 0x06;
	const uint8_t A_DLPF_CFG_420 = 0x07;
	// End Define ACCEL_CONFIG2
	
	// Define FIFO_EN
	const uint8_t GYRO_FIFO_EN = 0x10;
	const uint8_t GYRO_FIFO_DIS = 0x00;
	const uint8_t ACCEL_FIFO_EN = 0x08;
	const uint8_t ACCEL_FIFO_DIS = 0x00;
	// End Define FIFO_EN
	
	// Define INT_PIN_CFG
	const uint8_t INT_LEVEL = 0x80;
	const uint8_t INT_OPEN = 0x40;
	const uint8_t LATCH_INT_EN = 0x20;
	const uint8_t INT_RD_CLEAR = 0x10;
	const uint8_t FSYNC_INT_LEVEL = 0x08;
	const uint8_t FSYNC_INT_MODE_EN = 0x04;
	// End Define INT_PIN_CFG
	
	// Define INT_ENABLE
	const uint8_t WOM_X_INT_EN = 0x80;
	const uint8_t WOM_Y_INT_EN = 0x40;
	const uint8_t WOM_Z_INT_EN = 0x20;
	const uint8_t FIFO_OFLOW_EN = 0x10;
	const uint8_t FSYNC_INT_EN = 0x08;
	const uint8_t GDRIVE_INT_EN  = 0x04;
	const uint8_t DATA_RDY_INT_EN  = 0x01;
	const uint8_t INT_DISABLE = 0x00;
	// End Define INT_ENABLE
	
	// Define USER_CTRL
	const uint8_t FIFO_ENABLE = 0x40;
	const uint8_t FIFO_RST = 0x04;
	const uint8_t SIG_COND_RST = 0x01;
	// End Define USER_CTRL
	
	// Define PWR_MGMT_1
	const uint8_t PWR_RESET = 0x80;
	const uint8_t PWR_SLEEP = 0x40;
	const uint8_t PWR_CYCLE = 0x20;
	const uint8_t PWR_GYRO_STANDBY = 0x10;
	const uint8_t PWR_TEMP_DIS = 0x80;
	const uint8_t PWR_CLKSEL_RST = 0x07;
	const uint8_t PWR_CLKSEL_CLK_20MHZ = 0x00;
	const uint8_t PWR_CLKSEL_CLK_PLL = 0x01;
	// End Define PWR_MGMT_1
	
	// Define PWR_MGMT_2
	const uint8_t PWR_STBY_XA_DIS = 0x20;
	const uint8_t PWR_STBY_YA_DIS = 0x10;
	const uint8_t PWR_STBY_ZA_DIS = 0x08;
	const uint8_t PWR_STBY_XG_DIS = 0x04;
	const uint8_t PWR_STBY_YG_DIS = 0x02;
	const uint8_t PWR_STBY_ZG_DIS = 0x01;
	const uint8_t ACCEL_GYRO_EN = 0x00;
	// End Define PWR_MGMT_2
	
	// Define I2C_IF
	const uint8_t I2C_IF_DIS = 0x40;
	const uint8_t I2C_IF_EN = 0x00;
	// End Define I2C_IF
	
	// Define WHO_AM_I
	const uint8_t I_AM_ICM20602 = 0x12;
	// End Define WHO_AM_I

    // private functions
    int writeRegister(uint8_t subAddress, uint8_t data);
    int readRegisters(uint8_t subAddress, uint8_t count, uint8_t* dest);
    int whoAmI();
};

#endif
