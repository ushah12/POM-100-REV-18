#ifndef __BMA253_H__
#define __BMA253_H__

#ifdef __cplusplus  
extern "C" { 
#endif 
#include "bma2x2.h"
#ifdef __cplusplus 
} 
#endif
#include "debug.h"

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
#include <Wire.h>
#include "debug.h"
#define device_addr 0x18
#define BMAchipID 0xFA

#define SLOPE_THRES_CHANNEL 2
#define SLOPE_THRES 0x10//10

s8 BMA2x2_I2C_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 bytes_to_write = 1);
s8 BMA2x2_I2C_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 bytes_to_write = 1);

void initBMA(struct bma2x2_t bma253);
void initBMA();
void setBMA_Motioninterrupts();
void setBMA_NoMotioninterrupts(u8 intr_en);

void readBMAChipID(uint8_t *chipID);
void readInterrupt(uint8_t slopeIntr);
bool readBMAregister();
void en_low_pwr(void);
void bma_deep_suspended(void);


#endif
