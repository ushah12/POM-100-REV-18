#include "BMA253.h"


#define BMA_DEBUG

bool debFlag = false;
/*I2C Write Function for BMA253 using the standard Arduino I2C Communication (Wire)*/
s8 BMA2x2_I2C_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 bytes_to_write)
{
  int8_t ret;
  Wire.beginTransmission(dev_addr);

#if ARDUINO >= 100
  Wire.write(reg_addr);
  Wire.write(*reg_data);
#else
  Wire.send(reg_addr);
  Wire.send(*reg_data);
#endif
  Wire.endTransmission();
  return (ret);
}
/*I2C Read Function for BMA253 using the standard Arduino I2C Communication (Wire)*/
s8 BMA2x2_I2C_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 bytes_to_write)
{
  uint8_t ret;
  uint8_t value;
  Wire.beginTransmission(dev_addr);
#if ARDUINO >= 100
  Wire.write(reg_addr);
#else
  Wire.send(reg_addr);
#endif
  Wire.endTransmission();
  Wire.beginTransmission(dev_addr);
  Wire.requestFrom(dev_addr, bytes_to_write);
  //while(!Wire.available()) {};
  //Serial.println("HERE");
#if ARDUINO >= 100
  *reg_data = (u8)Wire.read();
#else
  reg_data = Wire.receive();
#endif;
  Wire.endTransmission();
  return (ret);
}
void initBMA()
{
  Wire.begin();                //Initializing BMA I2C
  Wire.setClock(400000);            //Initializing BMA BaudRate
  en_low_pwr();                      //Enabling Low Power Mode..
}
void initBMA(struct bma2x2_t bma253)
{
  Wire.begin();								//Initializing BMA I2C
  Wire.setClock(400000);						//Initializing BMA BaudRate
  /*u8 ret;
  u8 low_pwr_mode_val = 0xB6;					//Low Power Mode Value
  ret = BMA2x2_I2C_write(device_addr, 0x14, &low_pwr_mode_val);		//Write to I2C	Register
  BMA2x2_RETURN_FUNCTION_TYPE  bmaRet;				//BMAreturn types 0 for Success//-1 for error
  bma253.power_mode_u8 = 0;										//BMA253 Structure
  bma253.chip_id = 0xFA;											//BMA Chip ID
  bma253.ctrl_mode_reg = 0x11;									//Mode Register
  bma253.low_mode_reg = 0x12;										//Low Power Mode Register
  bma253.dev_addr = 0x18;											//BMA Address
  bma253.fifo_config = 0x00;										//FiFo Config	//Refer Data Sheet?
  bma253.bus_write = BMA2x2_I2C_write;						//Assigning I2C Write Functions to BMA Library
  bma253.bus_read = BMA2x2_I2C_read;							//Assigning I2C Read Functions to BMA Library
  bma253.delay_msec = 0;//BMA2x2_delay_msek;			//Assigning I2C delay Functions to BMA Library No delay at the Moment.
  bmaRet = bma2x2_init(&bma253);									//Initializing BMA253
  bma2x2_set_range(0x03);								//Setting BMA Range*/
  en_low_pwr();											//Enabling Low Power Mode..

  //bma2x2_set_bw(0x0F);
}
/*Setting BMA Interrupts*/
void setBMA_Motioninterrupts()
{
  int8_t bmaRet;
  u8 intr_Val = 0x07;
  u8 reg_data;
  uint8_t retry_rate = 0;
  BMA2x2_I2C_write(device_addr, 0x16, &intr_Val);								//Accelerometer Slope interrupt Enable
  bmaRet = BMA2x2_I2C_read(device_addr, 0x16, &reg_data);				//Added for Dubugging.. No Need in Production
  if (bmaRet != 0) {
    DBMA_PRINTF("bma2x2_set_intr_enable Slope %d\n", bmaRet);
  }
  while ((reg_data != 0x07) && (retry_rate < 6))	// If Value isnt C4 Means Any interrupt isnt Set or None is Set.. Set them again nad Retry Rate is 3 Tries..
  {
    if (reg_data != 0x07)		// If 0x00 means all Interrupts Didnt Set... Set them again
    {
      //bma2x2_set_intr_enable(5|6|7 , intr_en);
      BMA2x2_I2C_write(device_addr, 0x16, &intr_Val);							//Accelerometer Slope interrupt Enable
      //bma2x2_set_intr_slope(1, intr_en);									//Accelerometer Slope interrupt Enable/Disable

      bmaRet = BMA2x2_I2C_read(device_addr, 0x16, &reg_data);				//Added for Dubugging.. No Need in Production
    }
    DBMA_PRINTF("BMA2x2_I2C_read bmaRet  %u = %02X\n", retry_rate, bmaRet);
    DBMA_PRINTF("reg_data %u = %02X\n", retry_rate, reg_data);
    retry_rate++;
  }

  retry_rate = 0;
  intr_Val = 0x04;
  BMA2x2_I2C_write(device_addr, 0x1B, &intr_Val);				//Accelerometer Slope interrupt Enable*/
  if (bmaRet != 0) {
    DBMA_PRINTF("bma2x2_set_intr_slope %d\n", bmaRet);
  }																						//Added for Dubugging.. No Need in Production
  bmaRet = BMA2x2_I2C_read(device_addr, 0x1B, &reg_data);						//Added for Dubugging.. No Need in Production
  DBMA_PRINTF("BMA2x2_I2C_read bmaRet = %02X\n", bmaRet);
  DBMA_PRINTF("reg_data = %02X\n", reg_data);

  while ((reg_data != 0x04) && (retry_rate < 6))	// If Value isnt 04 Means Any interrupt isnt Set or None is Set.. Set them again nad Retry Rate is 3 Tries..
  {
    if (reg_data == 0x00)		// If 0x00 means all Interrupts Didnt Set... Set them again
    {
      BMA2x2_I2C_write(device_addr, 0x1B, &intr_Val);							//Accelerometer Slope interrupt Enable

      bmaRet = BMA2x2_I2C_read(device_addr, 0x1B, &reg_data);				//Added for Dubugging.. No Need in Production
    }
    DBMA_PRINTF("BMA2x2_I2C_read bmaRet  %u = %02X\n", retry_rate, bmaRet);
    DBMA_PRINTF("reg_data %u = %02X\n", retry_rate, reg_data);
    retry_rate++;
  }
  intr_Val = SLOPE_THRES;
  /*Setting BMA253 Threshold for any motion interrupt*/
  BMA2x2_I2C_write(device_addr, 0x28, &intr_Val);
  reg_data = 0;
  bmaRet = BMA2x2_I2C_read(device_addr, 0x28, &reg_data);		//Reading Register to check Wether value is correct or Not...

  DBMA_PRINTF("bma2x2 threshold %d\n", reg_data);
  retry_rate = 0;
  while ((reg_data != SLOPE_THRES) && (retry_rate < 6))
  {
    BMA2x2_I2C_write(device_addr, 0x28, &intr_Val);
    bmaRet = BMA2x2_I2C_read(device_addr, 0x28, &reg_data);

    DBMA_PRINTF("bma2x2 threshold %u = %d\n", retry_rate, reg_data);
    retry_rate++;
  }
}
void setBMA_NoMotioninterrupts(u8 intr_en)
{
  int8_t bmaRet = 0;
  u8 intr_Val = 0x0F;
  u8 reg_data;
  uint8_t retry_rate = 0;
  BMA2x2_I2C_write(device_addr, 0x18, &intr_Val);								//Accelerometer Slope interrupt Enable*/

  if (bmaRet != 0) {

    DBMA_PRINTF("bma2x2_set_intr_enable Slope %d\n", bmaRet);
  }
  bmaRet = BMA2x2_I2C_read(device_addr, 0x18, &reg_data);				//Added for Dubugging.. No Need in Production
  while ((reg_data != 0x0F) && (retry_rate < 3))	// If Value isnt C4 Means Any interrupt isnt Set or None is Set.. Set them again nad Retry Rate is 3 Tries..
  {
    if (reg_data != 0x0F)		// If 0x00 means all Interrupts Didnt Set... Set them again
    {
      //bma2x2_set_intr_enable(5|6|7 , intr_en);							//Accelerometer Slope interrupt Enable									//Accelerometer Orient interrupt Enable
      BMA2x2_I2C_write(device_addr, 0x18, &intr_Val);
      //bma2x2_set_intr_slope(1, intr_en);									//Accelerometer Slope interrupt Enable/Disable

      bmaRet = BMA2x2_I2C_read(device_addr, 0x18, &reg_data);				//Added for Dubugging.. No Need in Production
    }

    DBMA_PRINTF("BMA2x2_I2C_read bmaRet  %u = %02X\n", retry_rate, bmaRet);
    DBMA_PRINTF("reg_data %u = %02X\n", retry_rate, reg_data);
    retry_rate++;
  }
  retry_rate = 0;
  intr_Val = 0x08;
  BMA2x2_I2C_write(device_addr, 0x1B, &intr_Val);
  //bmaRet = bma2x2_set_intr_slope(1, intr_en);									//Accelerometer Slope interrupt Enable*/
  if (bmaRet != 0)
  {
    DBMA_PRINTF("bma2x2_set_intr_slope %d\n", bmaRet);
  }
  //Added for Dubugging.. No Need in Production
  bmaRet = BMA2x2_I2C_read(device_addr, 0x1B, &reg_data);						//Added for Dubugging.. No Need in Production

  DBMA_PRINTF("BMA2x2_I2C_read bmaRet = %02X\n", bmaRet);
  DBMA_PRINTF("reg_data = %02X\n", reg_data);

  while ((reg_data != 0x08) && (retry_rate < 3))	// If Value isnt C4 Means Any interrupt isnt Set or None is Set.. Set them again nad Retry Rate is 3 Tries..
  {
    if (reg_data != 0x08)		// If 0x00 means all Interrupts Didnt Set... Set them again
    {
      BMA2x2_I2C_write(device_addr, 0x1B, &intr_Val);

      bmaRet = BMA2x2_I2C_read(device_addr, 0x1B, &reg_data);				//Added for Dubugging.. No Need in Production
    }
    DBMA_PRINTF("BMA2x2_I2C_read bmaRet  %u = %02X\n", retry_rate, bmaRet);
    DBMA_PRINTF("reg_data %u = %02X\n", retry_rate, reg_data);
    retry_rate++;
  }
  intr_Val = SLOPE_THRES;
  /*Setting BMA253 Threshold for any motion interrupt*/
  BMA2x2_I2C_write(device_addr, 0x29, &intr_Val);
  reg_data = 0;
  bmaRet = BMA2x2_I2C_read(device_addr, 0x29, &reg_data);		//Reading Register to check Wether value is correct or Not...

  DBMA_PRINTF("bma2x2 threshold %d\n", reg_data);
  retry_rate = 0;
  while ((reg_data != 0x05) && (retry_rate < 4))
  {
    BMA2x2_I2C_write(device_addr, 0x29, &intr_Val);
    bmaRet = BMA2x2_I2C_read(device_addr, 0x29, &reg_data);

    DBMA_PRINTF("bma2x2 threshold %u = %d\n", retry_rate, reg_data);
    retry_rate++;
  }
}
/*Reading Chip Id of BMA Chip*/
void readBMAChipID(uint8_t *chipID)
{
  uint8_t bmaRet;
  //u8 reg_data_buf;
  u8 chip_id_reg = 0x00;
  bmaRet = BMA2x2_I2C_read(device_addr, chip_id_reg, chipID);
  //DBMA_PRINTF("%02X\n",reg_data_buf);
}
/*Interrupt reading of BMA253*/
void readInterrupt(uint8_t slopeIntr)
{
  /*if (slopeIntr == 1)
  {
    debFlag = true;
    slopeIntr = 0;
    //interruptDebugFlag = true;
  }
  */
}
/*Func to read BMA253 Registers to Check Interrupts*/
bool readBMAregister()
{
  uint8_t bmaRet;
  uint8_t slopeIntr = 0;
  u8 reg_data_buf = 0;//xC4;
  u8 r_o_reg = 0x09;																													 //Reading Read Only Interrupt Register
  //if(flag)
  {
    bmaRet = BMA2x2_I2C_read(device_addr, r_o_reg, &reg_data_buf);					//Reading Register

    if(reg_data_buf != 0)
    {
      DBMA_PRINTF("The reg_data_buf = %02X\n", reg_data_buf);
    }
    if (bmaRet != 0x00)
    {
      //printf("BMA2x2_I2C_read %02X\r\n", bmaRet);
    }
    //		flag = false;
    slopeIntr = ((reg_data_buf & 0x04) >> 2);						//masking Out Slope/Any Motion Interrupt
    //noSlopeIntr = (uint8_t*)((reg_data_buf&0x08)>>3);
  }
  if (slopeIntr == 1)
  {
    debFlag = true;
    slopeIntr = 0;
    return (true);
  }
  else 
  {
    return (false);
  }
  //readInterrupt(slopeIntr);
}
/*Low Power Mode Enable Func*/
void en_low_pwr(void)
{
  u8 ret;
  u8 reg_data;                                                      //Added for Dubugging.. No Need in Production
  u8 low_pwr_mode_val = 0x00;                                              //LOW Power Mode 1 Selected
  ret = BMA2x2_I2C_write(device_addr, 0x12, &low_pwr_mode_val);   //Writing to register 0x12 for Low Power Mode 1 Value
  ret = BMA2x2_I2C_read(device_addr, 0x12, &reg_data);            //Added for Dubugging... No Need in Production

  DBMA_PRINTF("low_pwr_mode_val Mode 1 reading %02X\n", reg_data);

  uint8_t retry_rate = 0;
  while ((reg_data != 0x00) && (retry_rate < 6))
  {
    ret = BMA2x2_I2C_write(device_addr, 0x12, &low_pwr_mode_val);   //Writing to register 0x12 for Low Power Mode 1 Value
    ret = BMA2x2_I2C_read(device_addr, 0x12, &reg_data);            //Added for Dubugging... No Need in Production

    DBMA_PRINTF("low_pwr_mode_val For Mode 1 reading %u = %02X\n", retry_rate, reg_data);

    retry_rate++;
  }
  //0x5C--> 500 mS Sleep intrval//0x56--> 25 mS Sleep intrval//0x5E--> 1 S Sleep intrval//0x5E--> 1 S Sleep intrval//0x40--> 1mS Sleep intrval
  
  low_pwr_mode_val = 0x5E;//0x56;//0x5E;//0x5C;//0x40;											//Low Power mode enable with 1 s Sleep Duration
  ret = BMA2x2_I2C_write(device_addr, 0x11, &low_pwr_mode_val);			//Writing to register 0x11 for Low power mode value and Sleep Duration
  
  ret = BMA2x2_I2C_read(device_addr, 0x11, &reg_data);						//Added for Dubugging.. No Need in Production
  retry_rate = 0;

  DBMA_PRINTF("low_pwr_mode_val = %02X\n", reg_data);

  while ((reg_data != 0x5E) && (retry_rate < 6))
  {
    ret = BMA2x2_I2C_write(device_addr, 0x11, &low_pwr_mode_val);
    ret = BMA2x2_I2C_read(device_addr, 0x11, &reg_data);							//Added for Dubugging.. No Need in Production

    DBMA_PRINTF("low_pwr_mode_val %u = %02X\n", retry_rate, reg_data);
    retry_rate++;
  }
}

/*Deep Suspended Mode Enable Func*/
void bma_deep_suspended(void)
{
  u8 ret;
  u8 bma_mode_val = 0x20;											 											 //Deep Suspended Mode Value
  ret = BMA2x2_I2C_write(device_addr, 0x11, &bma_mode_val);			//Writing to register 0x11 for Deep Suspended Mode
  u8 reg_data;																											//Added for Dubugging.. No Need in Production
  ret = BMA2x2_I2C_read(device_addr, 0x11, &reg_data);						//Added for Dubugging.. No Need in Production*/

  DBMA_PRINTF("bma_deep_suspended reading = %02X\n", reg_data);

  uint8_t retry_rate = 0;
  while ((reg_data != 0x20) && (retry_rate < 6))
  {
    ret = BMA2x2_I2C_write(device_addr, 0x11, &bma_mode_val);		//Writing to register 0x12 for Low Power Mode 1 Value
    ret = BMA2x2_I2C_read(device_addr, 0x11, &reg_data);						//Added for Dubugging... No Need in Production

    DBMA_PRINTF("bma_deep_suspended reading %u = %02X\n", retry_rate, reg_data);

    retry_rate++;
  }
}
