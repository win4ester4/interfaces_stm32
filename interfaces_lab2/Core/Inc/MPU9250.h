#ifndef MPU925_H_
#define MPU925_H_
#endif /* MPU925_H_ */

#include "stm32f4xx_hal.h"
extern SPI_HandleTypeDef hspi1;

#define CS_ON HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET)
#define CS_OFF HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET)

char str1[100];

const uint8_t  GYRO_RANGE_250DPS=0;
const uint8_t  GYRO_RANGE_500DPS=1;
const uint8_t  GYRO_RANGE_1000DPS=2;
const uint8_t  GYRO_RANGE_2000DPS=3;


const uint8_t  ACCEL_RANGE_2G=0;
const uint8_t  ACCEL_RANGE_4G=1;
const uint8_t  ACCEL_RANGE_8G=2;
const uint8_t  ACCEL_RANGE_16G=3;


const uint8_t  DLPF_BANDWIDTH_184HZ=0;
const uint8_t  DLPF_BANDWIDTH_92HZ=1;
const uint8_t  DLPF_BANDWIDTH_41HZ=2;
const uint8_t  DLPF_BANDWIDTH_20HZ=3;
const uint8_t  DLPF_BANDWIDTH_10HZ=4;
const uint8_t  DLPF_BANDWIDTH_5HZ=5;


const uint8_t  LP_ACCEL_ODR_0_24HZ = 0;
const uint8_t  LP_ACCEL_ODR_0_49HZ = 1;
const uint8_t  LP_ACCEL_ODR_0_98HZ = 2;
const uint8_t  LP_ACCEL_ODR_1_95HZ = 3;
const uint8_t  LP_ACCEL_ODR_3_91HZ = 4;
const uint8_t  LP_ACCEL_ODR_7_81HZ = 5;
const uint8_t  LP_ACCEL_ODR_15_63HZ = 6;
const uint8_t  LP_ACCEL_ODR_31_25HZ = 7;
const uint8_t  LP_ACCEL_ODR_62_50HZ = 8;
const uint8_t  LP_ACCEL_ODR_125HZ = 9;
const uint8_t  LP_ACCEL_ODR_250HZ = 10;
const uint8_t  LP_ACCEL_ODR_500HZ = 11;

const uint8_t READWRITE_CMD = 0x80;
const uint8_t MULTIPLEBYTE_CMD = 0x40;
const uint8_t DUMMY_BYTE = 0x00;

const uint8_t _address = 0b11010000;
const uint32_t _i2cRate = 400000; // 400 kHz
size_t _numBytes; // number of bytes received from I2C

// track success of interacting with sensor
int _status;
// buffer for reading from sensor
uint8_t _buffer[21];
// data counts
int16_t _axcounts,_aycounts,_azcounts;
int16_t _gxcounts,_gycounts,_gzcounts;
int16_t _hxcounts,_hycounts,_hzcounts;
int16_t _tcounts;

// MPU9250 registers
const uint8_t ACCEL_OUT = 0x3B;
const uint8_t GYRO_OUT = 0x43;
const uint8_t TEMP_OUT = 0x41;
const uint8_t EXT_SENS_DATA_00 = 0x49;
const uint8_t ACCEL_CONFIG = 0x1C;
const uint8_t ACCEL_FS_SEL_2G = 0x00;
const uint8_t ACCEL_FS_SEL_4G = 0x08;
const uint8_t ACCEL_FS_SEL_8G = 0x10;
const uint8_t ACCEL_FS_SEL_16G = 0x18;
const uint8_t GYRO_CONFIG = 0x1B;
const uint8_t GYRO_FS_SEL_250DPS = 0x00;
const uint8_t GYRO_FS_SEL_500DPS = 0x08;
const uint8_t GYRO_FS_SEL_1000DPS = 0x10;
const uint8_t GYRO_FS_SEL_2000DPS = 0x18;
const uint8_t ACCEL_CONFIG2 = 0x1D;
const uint8_t DLPF_184 = 0x01;
const uint8_t DLPF_92 = 0x02;
const uint8_t DLPF_41 = 0x03;
const uint8_t DLPF_20 = 0x04;
const uint8_t DLPF_10 = 0x05;
const uint8_t DLPF_5 = 0x06;
const uint8_t CONFIG = 0x1A;
const uint8_t SMPDIV = 0x19;
const uint8_t INT_PIN_CFG = 0x37;
const uint8_t INT_ENABLE = 0x38;
const uint8_t INT_DISABLE = 0x00;
const uint8_t INT_PULSE_50US = 0x00;
const uint8_t INT_WOM_EN = 0x40;
const uint8_t INT_RAW_RDY_EN = 0x01;
const uint8_t PWR_MGMNT_1 = 0x6B;
const uint8_t PWR_CYCLE = 0x20;
const uint8_t PWR_RESET = 0x80;
const uint8_t CLOCK_SEL_PLL = 0x01;
const uint8_t PWR_MGMNT_2 = 0x6C;
const uint8_t SEN_ENABLE = 0x00;
const uint8_t DIS_GYRO = 0x07;
const uint8_t USER_CTRL = 0x6A;
const uint8_t I2C_MST_EN = 0x20;
const uint8_t I2C_MST_CLK = 0x0D;
const uint8_t I2C_MST_CTRL = 0x24;
const uint8_t I2C_SLV0_ADDR = 0x25;
const uint8_t I2C_SLV0_REG = 0x26;
const uint8_t I2C_SLV0_DO = 0x63;
const uint8_t I2C_SLV0_CTRL = 0x27;
const uint8_t I2C_SLV0_EN = 0x80;
const uint8_t I2C_READ_FLAG = 0x80;
const uint8_t MOT_DETECT_CTRL = 0x69;
const uint8_t ACCEL_INTEL_EN = 0x80;
const uint8_t ACCEL_INTEL_MODE = 0x40;
const uint8_t LP_ACCEL_ODR = 0x1E;
const uint8_t WOM_THR = 0x1F;
const uint8_t WHO_AM_I = 0x75;
const uint8_t FIFO_EN = 0x23;
const uint8_t FIFO_TEMP = 0x80;
const uint8_t FIFO_GYRO = 0x70;
const uint8_t FIFO_ACCEL = 0x08;
const uint8_t FIFO_MAG = 0x01;
const uint8_t FIFO_COUNT = 0x72;
const uint8_t FIFO_READ = 0x74;
// AK8963 registers
const uint8_t AK8963_I2C_ADDR = 0x0C;
const uint8_t AK8963_HXL = 0x03;
const uint8_t AK8963_CNTL1 = 0x0A;
const uint8_t AK8963_PWR_DOWN = 0x00;
const uint8_t AK8963_CNT_MEAS1 = 0x12;
const uint8_t AK8963_CNT_MEAS2 = 0x16;
const uint8_t AK8963_FUSE_ROM = 0x0F;
const uint8_t AK8963_CNTL2 = 0x0B;
const uint8_t AK8963_RESET = 0x01;
const uint8_t AK8963_ASA = 0x10;
const uint8_t AK8963_WHO_AM_I = 0x00;

//-------------------------------------------

int writeRegister(uint8_t subAddress, uint8_t data);
int readRegisters(uint8_t subAddress, uint8_t count, uint8_t* dest);
int writeAK8963Register(uint8_t subAddress, uint8_t data);
int readAK8963Registers(uint8_t subAddress, uint8_t count, uint8_t* dest);
int whoAmI();
int whoAmIAK8963();

//-------------------------------------------

/* starts communication with the MPU-9250 */
int MPU_begin(){
  // select clock source to gyro
  if(writeRegister(PWR_MGMNT_1,CLOCK_SEL_PLL) < 0){
    return -1;
  }
  // enable I2C master mode
  if(writeRegister(USER_CTRL,I2C_MST_EN) < 0){
    return -2;
  }
  // set the I2C bus speed to 400 kHz
  if(writeRegister(I2C_MST_CTRL,I2C_MST_CLK) < 0){
    return -3;
  }
  // set AK8963 to Power Down
  writeAK8963Register(AK8963_CNTL1,AK8963_PWR_DOWN);
  // reset the MPU9250
  writeRegister(PWR_MGMNT_1,PWR_RESET);
  // wait for MPU-9250 to come back up
  HAL_Delay(10);
  // reset the AK8963
  writeAK8963Register(AK8963_CNTL2,AK8963_RESET);
  // select clock source to gyro
  if(writeRegister(PWR_MGMNT_1,CLOCK_SEL_PLL) < 0){
    return -4;
  }
  // check the WHO AM I byte, expected value is 0x71 (decimal 113) or 0x73 (decimal 115)
  if((whoAmI() != 113)&&(whoAmI() != 115)){
    return -5;
  }
  // enable accelerometer and gyro
  if(writeRegister(PWR_MGMNT_2,SEN_ENABLE) < 0){
    return -6;
  }
  // setting accel range to 16G as default
  if(writeRegister(ACCEL_CONFIG,ACCEL_FS_SEL_16G) < 0){
    return -7;
  }

  // setting the gyro range to 2000DPS as default
  if(writeRegister(GYRO_CONFIG,GYRO_FS_SEL_250DPS) < 0){
    return -8;
  }

  // setting bandwidth to 184Hz as default
  if(writeRegister(ACCEL_CONFIG2,DLPF_184) < 0){
    return -9;
  }
  if(writeRegister(CONFIG,DLPF_184) < 0){ // setting gyro bandwidth to 184Hz
    return -10;
  }

  // setting the sample rate divider to 0 as default
  if(writeRegister(SMPDIV,0x00) < 0){
    return -11;
  }

  // enable I2C master mode
  if(writeRegister(USER_CTRL,I2C_MST_EN) < 0){
  	return -12;
  }
	// set the I2C bus speed to 400 kHz
	if( writeRegister(I2C_MST_CTRL,I2C_MST_CLK) < 0){
		return -13;
	}
	// check AK8963 WHO AM I register, expected value is 0x48 (decimal 72)
	if( whoAmIAK8963() != 72 ){
    return -14;
	}

  /* get the magnetometer calibration */
  // set AK8963 to Power Down
  if(writeAK8963Register(AK8963_CNTL1,AK8963_PWR_DOWN) < 0){
    return -15;
  }
  HAL_Delay(100); // long wait between AK8963 mode changes
  // set AK8963 to FUSE ROM access
  if(writeAK8963Register(AK8963_CNTL1,AK8963_FUSE_ROM) < 0){
    return -16;
  }
  HAL_Delay(100); // long wait between AK8963 mode changes
  // read the AK8963 ASA registers and compute magnetometer scale factors
  readAK8963Registers(AK8963_ASA,3,_buffer);

  // set AK8963 to Power Down
  if(writeAK8963Register(AK8963_CNTL1,AK8963_PWR_DOWN) < 0){
    return -17;
  }
  HAL_Delay(100); // long wait between AK8963 mode changes
  // set AK8963 to 16 bit resolution, 100 Hz update rate
  if(writeAK8963Register(AK8963_CNTL1,AK8963_CNT_MEAS2) < 0){
    return -18;
  }
  HAL_Delay(100); // long wait between AK8963 mode changes
  // select clock source to gyro
  if(writeRegister(PWR_MGMNT_1,CLOCK_SEL_PLL) < 0){
    return -19;
  }
  // instruct the MPU9250 to get 7 bytes of data from the AK8963 at the sample rate
  readAK8963Registers(AK8963_HXL,7,_buffer);

  // successful init, return 1
  return 1;
}


/* gets the MPU9250 WHO_AM_I register value, expected to be 0x71 */
int whoAmI(){
  // read the WHO AM I register
  if (readRegisters(WHO_AM_I,1,_buffer) < 0) {
    return -1;
  }
  // return the register value
  return _buffer[0];
}

/* gets the AK8963 WHO_AM_I register value, expected to be 0x48 */
int whoAmIAK8963(){
  // read the WHO AM I register
  if (readAK8963Registers(AK8963_WHO_AM_I,1,_buffer) < 0) {
    return -1;
  }
  // return the register value
  return _buffer[0];
}


/* writes a byte to MPU9250 register given a register address and data */
int writeRegister(uint8_t subAddress, uint8_t data)
{
  /* write data to device */
	HAL_StatusTypeDef status = HAL_OK;
	//status = HAL_I2C_Mem_Write(&hi2c1, _address, subAddress, I2C_MEMADD_SIZE_8BIT, &data, 1, 0x10000);
	MPU_SPI_Write(&data, subAddress, 1);
	HAL_Delay(10);

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

/* reads registers from MPU9250 given a starting register address, number of bytes, and a pointer to store data */
int readRegisters(uint8_t subAddress, uint8_t count, uint8_t* dest){
	HAL_StatusTypeDef status = HAL_OK;
//	status = HAL_I2C_Mem_Read(&hi2c1, _address, subAddress, I2C_MEMADD_SIZE_8BIT, dest, count, 0x10000);
	MPU_SPI_Read(dest, subAddress, count);
	if (status == HAL_OK) {
      return 1;
    } else {
      return -1;
    }
}

uint8_t SPIx_WriteRead(uint8_t Byte)
{
	uint8_t receivedbyte = 0;
	if(HAL_SPI_TransmitReceive(&hspi1,(uint8_t*) &Byte,(uint8_t*) &receivedbyte,1,0x1000)!=HAL_OK)
	{
		return -1;
	}
	else
	{
	}
	return receivedbyte;
}

void MPU_SPI_Read(uint8_t *pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead)
{
	ReadAddr |= (uint8_t)READWRITE_CMD;
	CS_ON;
	SPIx_WriteRead(ReadAddr);
	while(NumByteToRead>0x00)
	{
		*pBuffer=SPIx_WriteRead(DUMMY_BYTE);
		NumByteToRead--;
		pBuffer++;
	}
	CS_OFF;
}
//--------------------------------------
void MPU_SPI_Write (uint8_t *pBuffer, uint8_t WriteAddr, uint16_t NumByteToWrite)
{
	CS_ON;
	SPIx_WriteRead(WriteAddr);
	while(NumByteToWrite>=0x01)
	{
		SPIx_WriteRead(*pBuffer);
		NumByteToWrite--;
		pBuffer++;
	}
	CS_OFF;
}


/* writes a register to the AK8963 given a register address and data */
int writeAK8963Register(uint8_t subAddress, uint8_t data){
  // set slave 0 to the AK8963 and set for write
	if (writeRegister(I2C_SLV0_ADDR,AK8963_I2C_ADDR) < 0) {
    return -1;
  }
  // set the register to the desired AK8963 sub address
	if (writeRegister(I2C_SLV0_REG,subAddress) < 0) {
    return -2;
  }
  // store the data for write
	if (writeRegister(I2C_SLV0_DO,data) < 0) {
    return -3;
  }
  // enable I2C and send 1 byte
	if (writeRegister(I2C_SLV0_CTRL,I2C_SLV0_EN | (uint8_t)1) < 0) {
    return -4;
  }
	// read the register and confirm
	if (readAK8963Registers(subAddress,1,_buffer) < 0) {
    return -5;
  }
	if(_buffer[0] == data) {
  	return 1;
  } else{
  	return -6;
  }
}

/* reads registers from the AK8963 */
int readAK8963Registers(uint8_t subAddress, uint8_t count, uint8_t* dest){
  // set slave 0 to the AK8963 and set for read
	if (writeRegister(I2C_SLV0_ADDR,AK8963_I2C_ADDR | I2C_READ_FLAG) < 0) {
    return -1;
  }
  // set the register to the desired AK8963 sub address
	if (writeRegister(I2C_SLV0_REG,subAddress) < 0) {
    return -2;
  }
  // enable I2C and request the bytes
	if (writeRegister(I2C_SLV0_CTRL,I2C_SLV0_EN | count) < 0) {
    return -3;
  }
	HAL_Delay(1); // takes some time for these registers to fill
  // read the bytes off the MPU9250 EXT_SENS_DATA registers
	_status = readRegisters(EXT_SENS_DATA_00,count,dest);
  return _status;
}


/* sets the accelerometer full scale range to values other than default */
int setAccelRange(uint8_t range) {
  if(writeRegister(ACCEL_CONFIG, range) < 0){
        return -1;
  }
  return 1;
}

/* sets the gyro full scale range to values other than default */
int setGyroRange(uint8_t range) {
  if(writeRegister(GYRO_CONFIG, range) < 0){
        return -1;
  }
  return 1;
}


/* sets the DLPF bandwidth to values other than default */
int setDlpfBandwidth(uint8_t bandwidth) {
  if(writeRegister(ACCEL_CONFIG2,bandwidth) < 0){
	return -1;
  }
  if(writeRegister(CONFIG,bandwidth) < 0){
	return -2;
  }
 return 1;
}

/* sets the sample rate divider to values other than default */
int setSrd(uint8_t srd) {

  /* setting the sample rate divider to 19 to facilitate setting up magnetometer */
  if(writeRegister(SMPDIV,19) < 0){ // setting the sample rate divider
    return -1;
  }
  if(srd > 9){
    // set AK8963 to Power Down
    if(writeAK8963Register(AK8963_CNTL1,AK8963_PWR_DOWN) < 0){
      return -2;
    }
    HAL_Delay(100); // long wait between AK8963 mode changes
    // set AK8963 to 16 bit resolution, 8 Hz update rate
    if(writeAK8963Register(AK8963_CNTL1,AK8963_CNT_MEAS1) < 0){
      return -3;
    }
    HAL_Delay(100); // long wait between AK8963 mode changes
    // instruct the MPU9250 to get 7 bytes of data from the AK8963 at the sample rate
    readAK8963Registers(AK8963_HXL,7,_buffer);
  } else {
    // set AK8963 to Power Down
    if(writeAK8963Register(AK8963_CNTL1,AK8963_PWR_DOWN) < 0){
      return -2;
    }
    HAL_Delay(100); // long wait between AK8963 mode changes
    // set AK8963 to 16 bit resolution, 100 Hz update rate
    if(writeAK8963Register(AK8963_CNTL1,AK8963_CNT_MEAS2) < 0){
      return -3;
    }
    HAL_Delay(100); // long wait between AK8963 mode changes
    // instruct the MPU9250 to get 7 bytes of data from the AK8963 at the sample rate
    readAK8963Registers(AK8963_HXL,7,_buffer);
  }
  /* setting the sample rate divider */
  if(writeRegister(SMPDIV,srd) < 0){ // setting the sample rate divider
    return -4;
  }
  return 1;
}


/* enables the data ready interrupt */
int enableDataReadyInterrupt() {
  /* setting the interrupt */
  if (writeRegister(INT_PIN_CFG,INT_PULSE_50US) < 0){ // setup interrupt, 50 us pulse
    return -1;
  }
  if (writeRegister(INT_ENABLE,INT_RAW_RDY_EN) < 0){ // set to data ready
    return -2;
  }
  return 1;
}

/* disables the data ready interrupt */
int disableDataReadyInterrupt() {
  if(writeRegister(INT_ENABLE,INT_DISABLE) < 0){ // disable interrupt
    return -1;
  }
  return 1;
}


/* reads the most current data from MPU9250 and stores in buffer */
int readSensor() {
  // grab the data from the MPU9250
  if (readRegisters(ACCEL_OUT, 21, _buffer) < 0) {
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
  _hxcounts = (((int16_t)_buffer[15]) << 8) | _buffer[14];
  _hycounts = (((int16_t)_buffer[17]) << 8) | _buffer[16];
  _hzcounts = (((int16_t)_buffer[19]) << 8) | _buffer[18];
  return 1;
}

int readSensor3(int16_t* AccData, int16_t* MagData, int16_t* GyroData) {
  // grab the data from the MPU9250
  if (readRegisters(ACCEL_OUT, 21, _buffer) < 0) {
    return -1;
  }
  // combine into 16 bit values
  AccData[0] = (((int16_t)_buffer[0]) << 8) | _buffer[1];
  AccData[1] = (((int16_t)_buffer[2]) << 8) | _buffer[3];
  AccData[2] = (((int16_t)_buffer[4]) << 8) | _buffer[5];
  GyroData[0] = (((int16_t)_buffer[8]) << 8) | _buffer[9];
  GyroData[1] = (((int16_t)_buffer[10]) << 8) | _buffer[11];
  GyroData[2] = (((int16_t)_buffer[12]) << 8) | _buffer[13];
  MagData[0] = (((int16_t)_buffer[15]) << 8) | _buffer[14];
  MagData[1] = (((int16_t)_buffer[17]) << 8) | _buffer[16];
  MagData[2] = (((int16_t)_buffer[19]) << 8) | _buffer[18];
  return 1;
}




