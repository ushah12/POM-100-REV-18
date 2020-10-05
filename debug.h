#ifndef DEBUG_H
#define DEBUG_H

#include "main.h"



#ifndef DEBUG
	#define DEBUG 1
#endif

#ifndef BMA_DEBUG
	#define BMA_DEBUG 0
#endif

#ifndef SPECIAL_DEBUG
  #define SPECIAL_DEBUG 1
#endif

#if DEBUG == 1
  #define D_init(...)    Serial.begin(__VA_ARGS__)
	#define D_PRINTLN(...)  Serial.println (__VA_ARGS__)
	#define D_PRINT(...)  Serial.print (__VA_ARGS__)
	#define D_PRINTF(...) Serial.printf(__VA_ARGS__)
	#define D_WRITE(...) Serial.write(__VA_ARGS__)
  #define D_PRINTBuffer(...) Serial.printBuffer(__VA_ARGS__)
#else
  #define D_init(...)
	#define D_PRINTLN(...)
	#define D_PRINT(...)
	#define D_PRINTF(...)
	#define D_WRITE(...)
  #define D_PRINTBuffer(...)
#endif

#if BMA_DEBUG == 1
	#define DBMA_PRINTLN(...)  Serial.println (__VA_ARGS__)
	#define DBMA_PRINT(...)  Serial.print (__VA_ARGS__)
	#define DBMA_PRINTF(...) Serial.printf(__VA_ARGS__)
	#define DBMA_WRITE(...) Serial.write(__VA_ARGS__)
#else
	#define DBMA_PRINTLN(...)
	#define DBMA_PRINT(...)
	#define DBMA_PRINTF(...)
	#define DBMA_WRITE(...)
#endif

#if SPECIAL_DEBUG == 1
  #define S_PRINTLN(...)  Serial.println (__VA_ARGS__)
  #define S_PRINT(...)  Serial.print (__VA_ARGS__)
  #define S_PRINTF(...) Serial.printf(__VA_ARGS__)
  #define S_WRITE(...) Serial.write(__VA_ARGS__)
  #define S_PRINTBuffer(...) Serial.printBuffer(__VA_ARGS__)
#else
  #define S_PRINTLN(...)
  #define S_PRINT(...)
  #define S_PRINTF(...)
  #define S_WRITE(...)
  #define S_PRINTBuffer(...)
#endif
#endif
