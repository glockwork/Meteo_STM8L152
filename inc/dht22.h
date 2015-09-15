#include "delay.h"

//#define DHT22
#define DHT11

#define DHT_PORT      GPIOD
#define DHT_PIN       GPIO_Pin_6

typedef enum
{
  RESULT_OK =           0,      // OK
  RESULT_ERRREAD =      1,      // Error reading
  RESULT_ERRCHKSUM =    2,      // Error check sum
} RESULT;

void DHT22_Init(void);
RESULT DHT22_GetData(float* t, float* h);
RESULT DHT11_GetData(unsigned char* t, unsigned char* h);
unsigned char DHT_ReadData(unsigned char* data);
unsigned char DHT_ReadRaw(unsigned char* arr);
unsigned char DHT_CheckSum(unsigned char* data);
