#include "dht22.h"
#include "stm8l15x_gpio.h"

#define NUM_OF_BYTES 5
#ifdef DHT22
 #define START_DELAY  750ul
#endif
#ifdef DHT11
 #define START_DELAY  40000ul
#endif
#define ONE_HIGH     150
#define ONE_LOW      130

/**
 * @brief Initialize port and pin to which DHT is plugged as output open-drain
*/
void DHT22_Init(void)
{
  DHT_PORT->DDR |= DHT_PIN;
  DHT_PORT->ODR |= DHT_PIN;
}

/**
 * @brief Get temperature and humidity of DHT22
 * @param t - pointer to temperature var
 * @param h - pointer to humidity var
 * @returns RESULT
 * @note Period between functon calls should be more then 2 seconds
 * @note While reading interrupts are disable
*/
RESULT DHT22_GetData(float* t, float* h)
{
  unsigned char tmp[5];
  if (DHT_ReadData(tmp) == 1)
    return RESULT_ERRREAD;
  if (tmp[4] == DHT_CheckSum(tmp))
  {
    *h = (float)(tmp[0]  * 256 + tmp[1]) / 10.0;
    *t = (float)(tmp[2]  * 256 + tmp[3]) / 10.0;
  }
  else
    return RESULT_ERRCHKSUM;
  return RESULT_OK;
}

/**
 * @brief Get temperature and humidity of DHT11
 * @param t - pointer to temperature var
 * @param h - pointer to humidity var
 * @returns RESULT
 * @note Period between functon calls should be more then 2 seconds
 * @note While reading interrupts are disable
*/
RESULT DHT11_GetData(unsigned char* t, unsigned char* h)
{
  unsigned char tmp[5];
  if (DHT_ReadData(tmp) == 1)
    return RESULT_ERRREAD;
  if (tmp[4] == DHT_CheckSum(tmp))
  {
    *h = tmp[0];
    *t = tmp[2];
  }
  else
    return RESULT_ERRCHKSUM;
  return RESULT_OK;
}

/**
 * @brief Reads 5 bytes and put them to data
 * @param data - pointer to output array
 * @returns 0 - if result OK, 1 - if error reading
 * @note Period between functon calls should be more then 2 seconds
 * @note While reading interrupts are disable
*/
unsigned char DHT_ReadData(unsigned char* data)
{
  unsigned char byte_num = 0, bit_num = 0, byte = 0, count = 0;
  disableInterrupts();
  /* sending start condition */
  DHT_PORT->ODR &= (~DHT_PIN);              // write "0"
  Delay(START_DELAY);                       // wait 1 ms
  DHT_PORT->ODR |= DHT_PIN;                 // write "1": make pin as input
  /* begin reading */
  while (DHT_PORT->IDR & DHT_PIN);          // wait sensor response 20-40us
  while (!(DHT_PORT->IDR & DHT_PIN));       // pin low 80ms: response
  while (DHT_PORT->IDR & DHT_PIN);          // pin high 80ms: prepare
  for (byte_num = 0; byte_num < NUM_OF_BYTES; byte_num++)
  {
    for (bit_num = 0; bit_num < 8; bit_num++)
    {
      while (!(DHT_PORT->IDR & DHT_PIN));     // pin low 50us
      while (DHT_PORT->IDR & DHT_PIN)         // pin high
      {
        count++;
      }
      if (count > ONE_LOW)
      {
        byte |= (1 << (8 - bit_num));
      }
      count = 0;
    }
    *(data + byte_num) = byte;
    byte = 0;
  }
  enableInterrupts();
  return 0;
}

/**
 * @brief Reads 40 bytes, each byte contains relative time of sensor's "1" or "0"
 * @param data - pointer to output array
 * @returns 0 - if result OK, 1 - if error reading
 * @note Period between functon calls should be more then 2 seconds
 * @note While reading interrupts are disable
*/
unsigned char DHT_ReadRaw(unsigned char* arr)
{
  unsigned char bit_num = 0, count = 0;
  disableInterrupts();
  /* sending start condition */
  DHT_PORT->ODR &= (~DHT_PIN);              // write "0"
  Delay(START_DELAY);                       // wait
  DHT_PORT->ODR |= DHT_PIN;                 // write "1": make pin as input
  /* begin reading */
  while (DHT_PORT->IDR & DHT_PIN);          // wait sensor response 20-40us
  while (!(DHT_PORT->IDR & DHT_PIN));       // pin low 80ms: response
  while (DHT_PORT->IDR & DHT_PIN);          // pin high 80ms: prepare
  for (bit_num = 0; bit_num < 40; bit_num++)
  {
    while (!(DHT_PORT->IDR & DHT_PIN));     // pin low 50us
    while (DHT_PORT->IDR & DHT_PIN)         // pin high
    {
      count++;
    }
    *(arr + bit_num) = count;
    count = 0;
  }
  enableInterrupts();
  return 0;
}

/**
 * @brief Calculate checksum
 * @param data - pointer to input array
 * @returns checksum
*/
unsigned char DHT_CheckSum(unsigned char* data)
{
  unsigned int sum = 0;
  for (int i = 0; i < 4; i++)
    sum += *(data + i);
  return (unsigned char)sum;
}
