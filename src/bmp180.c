#include "bmp180.h"

static uint32_t i2c_timeout;
#define set_tmo_us(time) i2c_timeout = (uint32_t)(SYS_CLOCK * time)
#define set_tmo_ms(time) i2c_timeout = (uint32_t)(SYS_CLOCK * time * 1000)
#define tmo i2c_timeout--
#define wait_event(event, timeout) set_tmo_ms(timeout);\
                                   while((event) && (--i2c_timeout));\
                                   if(!i2c_timeout) return 1;

/* ------ Register addresses ------------------------------------------------ */
#define CONSTS          0xAA    // from 0xAA to 0xBF
#define CTRL_MEAS       0xF4    // this reg starts conversion
#define OUT_MSB         0xF6    // next three regs for converted values
#define OUT_LSB         0xF7    // 
#define OUT_XLSB        0xF8    //
#define SOFT_RESET      0xE0    // soft reset reg
#define ID              0xD0    // id reg
/* -------------------------------------------------------------------------- */

/* ------ BMP180 constants, initialized when BMP180_Init -------------------- */
int16_t AC1, AC2, AC3, B1, B2, MB, MC, MD;
uint16_t AC4, AC5, AC6;
/* -------------------------------------------------------------------------- */

/* ------ BMP180 variables -------------------------------------------------- */
uint8_t oss = 0;
int32_t utemp = 0;
int32_t B5 = 0;
/* -------------------------------------------------------------------------- */

/* ------ Function prototipes ----------------------------------------------- */
uint8_t I2C_ReadByte(uint8_t address);
uint16_t I2C_ReadTwoBytes(uint8_t address);
uint8_t I2C_ReadBytes(uint8_t address, uint8_t* data, uint8_t lenth);
uint8_t I2C_WriteByte(uint8_t address, uint8_t data);
void BMP180_ReadUncompTemp(void);
int32_t BMP180_ReadUncompPress(void);
/* -------------------------------------------------------------------------- */

/* initialize hardware and reading constants */
void BMP180_Init(void)
{
  uint8_t bmp_constants[22] = {0};
  // initialize gpio
  I2C_PORT->DDR |= (I2C_SDA_PIN | I2C_SCL_PIN);
  // initialize clock
  CLK_PeripheralClockConfig(CLK_Peripheral_I2C1, ENABLE);
  // i2c initialization
  I2C_Init(I2C1, I2C_CLOCK_FREQ, I2C_OWN_ADDR, I2C_Mode_I2C, I2C_DutyCycle_16_9, I2C_Ack_Enable, I2C_AcknowledgedAddress_7bit);
  I2C_Cmd(I2C1, ENABLE);
  
  // read constansts
  I2C_ReadBytes(CONSTS, bmp_constants, 22);
  AC1 = (int16_t)(bmp_constants[0] * 256 + bmp_constants[1]);
  AC2 = (int16_t)(bmp_constants[2] * 256 + bmp_constants[3]);
  AC3 = (int16_t)(bmp_constants[4] * 256 + bmp_constants[5]);
  AC4 = bmp_constants[6] * 256 + bmp_constants[7];
  AC5 = bmp_constants[8] * 256 + bmp_constants[9];
  AC6 = bmp_constants[10] * 256 + bmp_constants[11];
  B1 = (int16_t)(bmp_constants[12] * 256 + bmp_constants[13]);
  B2 = (int16_t)(bmp_constants[14] * 256 + bmp_constants[15]);
  MB = (int16_t)(bmp_constants[16] * 256 + bmp_constants[17]);
  MC = (int16_t)(bmp_constants[18] * 256 + bmp_constants[19]);
  MD = (int16_t)(bmp_constants[20] * 256 + bmp_constants[21]);
}

/* force power-on reset */
void BMP180_SoftwareReset(void)
{
  I2C_WriteByte(SOFT_RESET, 0xB6);
}

/* read bmp180 id */
uint8_t BMP180_GetId(void)
{
  return I2C_ReadByte(ID);
}

/* starts measuring temperature */
void BMP180_StartMeasureTemp(void)
{
  I2C_WriteByte(CTRL_MEAS, 0x2E);
}

/* reads uncompensated temperature from bmp180 */
void BMP180_ReadUncompTemp(void)
{
  utemp = I2C_ReadTwoBytes(OUT_MSB);
}

/* 
  temperature from BMP180 in C x10[multiplied by 10] in deg
  @note - call this function after not less then 4.5ms after start measuring 
*/
int16_t BMP180_GetTemp(void)
{
  BMP180_ReadUncompTemp();
  int32_t X1, X2, X3, T;
  X1 = utemp - AC6;
  X1 *= AC5;
  X1 >>= 15;
  X2 = (int32_t)MC << 11;
  X3 = X1 + MD;
  X2 /= X3;
  B5 = X1 + X2;
  T = B5 + 8;
  T >>= 4;
  return T;
}

void BMP180_StartMeasurePressure(PressureMeasurementMode mode)
{
  oss = (uint8_t)mode;
  uint8_t BYTE = 0x34 | (oss << 6);
  I2C_WriteByte(CTRL_MEAS, BYTE);
}

int32_t BMP180_ReadUncompPress(void)
{
  uint8_t p[3] = {0};
  int32_t UP;
  I2C_ReadBytes(OUT_MSB, p, 3);
  UP = (int32_t)p[0] << 16;
  UP |= (int32_t)p[1] << 8;
  UP |= (int32_t)p[2];
  UP >>= (8 - oss);
  return UP;
}

/* 
@note Wait not less then xx.x ms before call this function after start 
pressure measurement
   where xx.x = 4.5ms in ultra low power mode,
   where xx.x = 7.5ms in standard mode,
   where xx.x = 13.5ms in high resolution mode,
   where xx.x = 25.5ms in ultra high resolution mode
@note Measure temperature before measuring pressure for correct data
*/
int32_t BMP180_GetPressureInPa(void)
{
  int32_t UP, B3, B6, B12, X1, X2, X3, P;
  uint32_t B4, B7;
  UP = BMP180_ReadUncompPress();
//  if (B5 == 0)
//  {
//    X1 = utemp - AC6;
//    X1 *= AC5;
//    X1 >>= 15;
//    X2 = (int32_t)MC << 11;
//    X3 = X1 + MD;
//    X2 /= X3;
//    B5 = X1 + X2;
//  }
  B6 = B5 - 4000;
  B12 = B6 * B6;
  B12 >>= 12;
  X1 = B12 * B2;
  X1 >>= 11;
  X2 = AC2 * B6;
  X2 >>= 11;
  X3 = X1 + X2;
  B3 = (AC1 << 2) + X3;
  B3 <<= oss;
  B3 += 2;
  B3 >>= 2;
  X1 = AC3 * B6;
  X1 >>= 13;
  X2 = B12 *  B1;
  X2 >>= 16;
  X3 = X1 + X2 + 2;
  X3 >>= 2;
  B4 = (uint32_t)(X3 + 32768);
  B4 *= (uint32_t)AC4;
  B4 >>= 15;
  B7 = (uint32_t)UP - B3;
  B7 *= (50000 >> oss);
  if (B7 < 0x80000000)
    P = (B7 * 2) / B4;
  else
    P = (B7 / B4) * 2;
  X1 = P >> 8;
  X1 *= X1;
  X1 *= 3038;
  X1 >>= 16;
  X2 = (-7357) * P;
  X2 >>= 16;
  P += ((X1 + X2 + 3791) >> 4);
  B5 = 0;
  return P;
}

/* 
@note Wait not less then xx.x ms before call this function after start 
pressure measurement
   where xx.x = 4.5ms in ultra low power mode,
   where xx.x = 7.5ms in standard mode,
   where xx.x = 13.5ms in high resolution mode,
   where xx.x = 25.5ms in ultra high resolution mode
@note Measure temperature before measuring pressure for correct data
*/
float BMP180_GetPressureInMMHG(void)
{
  int32_t PrPa = BMP180_GetPressureInPa();
  return PrPa/133.3224;
}

float BMP180_GetAltitude(void)
{
  float height = 0;
  int32_t PrPa = BMP180_GetPressureInPa();
  /// TODO calculate altitude acording to atmosphere pressure
  return height;
}

/*----------------------------------------------------------------------------*/
uint8_t I2C_ReadByte(uint8_t address)
{
  uint8_t result;
  wait_event(I2C1->SR3 & I2C_SR3_BUSY, 10);                                     // wait for free bus
  I2C1->CR2 |= I2C_CR2_ACK;                                                     // enable acknowledge
  I2C1->CR2 |= I2C_CR2_START;                                                   // send start
  wait_event(!(I2C1->SR1 & I2C_SR1_SB), 1);                                     // wait start transmitted
  I2C1->DR = 0xEE;                                                              // slave address write
  wait_event(!(I2C1->SR1 & I2C_SR1_ADDR), 1);                                   // wait address transmitted
  I2C1->SR3;                                                                    // clear flag ADDR
  wait_event(!(I2C1->SR1 & I2C_SR1_TXE), 1);                                    // wait RD clear
  I2C1->DR = address;                                                           // send reg address
  wait_event(!( (I2C1->SR1 & I2C_SR1_TXE) && (I2C1->SR1 & I2C_SR1_BTF) ), 1);   // data is in register
  I2C1->CR2 |= I2C_CR2_START;                                                   // send restart
  wait_event(!(I2C1->SR1 & I2C_SR1_SB), 1);                                     // wait start transmitted
  I2C1->DR = 0xEF;                                                              // slave address read
  I2C1->CR2 &= (~I2C_CR2_ACK);                                                  // disable acknowledge
  wait_event(!(I2C1->SR1 & I2C_SR1_ADDR), 1);                                   // wait address transmitted
  disableInterrupts();
  I2C1->SR3;                                                                    // clear flag ADDR
  I2C1->CR2 |= I2C_CR2_STOP;                                                    // generate stop
  enableInterrupts();
  wait_event(!(I2C1->SR1 & I2C_SR1_RXNE), 1);                                   // wait for data
  result = I2C1->DR;                                                            // read data
  wait_event(I2C1->CR2 & I2C_CR2_STOP, 1);                                      // wait for stop
  return result;
}

uint16_t I2C_ReadTwoBytes(uint8_t address)
{
  uint16_t result = 0;
  wait_event(I2C1->SR3 & I2C_SR3_BUSY, 10);                                     // wait for free bus
  I2C1->CR2 |= I2C_CR2_ACK;                                                     // enable acknowledge
  I2C1->CR2 |= I2C_CR2_START;                                                   // send start
  wait_event(!(I2C1->SR1 & I2C_SR1_SB), 1);                                     // wait start transmitted
  I2C1->DR = 0xEE;                                                              // slave address write
  wait_event(!(I2C1->SR1 & I2C_SR1_ADDR), 1);                                   // wait address transmitted
  I2C1->SR3;                                                                    // clear flag ADDR
  wait_event(!(I2C1->SR1 & I2C_SR1_TXE), 1);                                    // wait RD clear
  I2C1->DR = address;                                                           // send reg address
  wait_event(!( (I2C1->SR1 & I2C_SR1_TXE) && (I2C1->SR1 & I2C_SR1_BTF) ), 1);   // data is in register
  I2C1->CR2 |= I2C_CR2_START;                                                   // send restart
  wait_event(!(I2C1->SR1 & I2C_SR1_SB), 1);                                     // wait start transmitted
  I2C1->DR = 0xEF;                                                              // slave address read
  I2C1->CR2 |= I2C_CR2_POS;                                                     // enable NACK on next transmission
  wait_event(!(I2C1->SR1 & I2C_SR1_ADDR), 1);                                   // wait address transmitted
  disableInterrupts();
  I2C1->SR3;                                                                    // clear flag ADDR
  I2C1->CR2 &= (~I2C_CR2_ACK);                                                  // disable acknowledge
  enableInterrupts();
  wait_event(!(I2C1->SR1 & I2C_SR1_BTF), 1);                                    // wait first byte in DR, second in sheeft reg
  disableInterrupts();
  I2C1->CR2 |= I2C_CR2_STOP;                                                    // generate stop
  result = (I2C1->DR) << 8;                                                     // first byte
  enableInterrupts();
  result |= I2C1->DR;                                                           // second byte
  wait_event(I2C1->CR2 & I2C_CR2_STOP, 1);                                      // wait for stop
  I2C1->CR2 &= (~I2C_CR2_POS);                                                  // disable NACK on next transmission
  return result;
}

uint8_t I2C_ReadBytes(uint8_t address, uint8_t* data, uint8_t lenth)
{
  wait_event(I2C1->SR3 & I2C_SR3_BUSY, 10);                                     // wait for free bus
  I2C1->CR2 |= I2C_CR2_ACK;                                                     // enable acknowledge
  I2C1->CR2 |= I2C_CR2_START;                                                   // send start
  wait_event(!(I2C1->SR1 & I2C_SR1_SB), 1);                                     // wait start transmitted
  I2C1->DR = 0xEE;                                                              // slave address write
  wait_event(!(I2C1->SR1 & I2C_SR1_ADDR), 1);                                   // wait address transmitted
  I2C1->SR3;                                                                    // clear flag ADDR
  wait_event(!(I2C1->SR1 & I2C_SR1_TXE), 1);                                    // wait RD clear
  I2C1->DR = address;                                                           // send reg address
  wait_event(!( (I2C1->SR1 & I2C_SR1_TXE) && (I2C1->SR1 & I2C_SR1_BTF) ), 1);   // data is in register
  I2C1->CR2 |= I2C_CR2_START;                                                   // send restart
  wait_event(!(I2C1->SR1 & I2C_SR1_SB), 1);                                     // wait start transmitted
  I2C1->DR = 0xEF;                                                              // slave address read
  if (lenth == 1)
  {
    I2C1->CR2 &= (~I2C_CR2_ACK);                                                // disable acknowledge
    wait_event(!(I2C1->SR1 & I2C_SR1_ADDR), 1);                                 // wait address transmitted
    disableInterrupts();
    I2C1->SR3;                                                                  // clear flag ADDR
    I2C1->CR2 |= I2C_CR2_STOP;                                                  // generate stop
    enableInterrupts();
    wait_event(!(I2C1->SR1 & I2C_SR1_RXNE), 1);                                 // wait for data
    *data = I2C1->DR;                                                           // read data
  }
  else if (lenth == 2)
  {
    I2C1->CR2 |= I2C_CR2_POS;                                                   // enable NACK on next transmission
    wait_event(!(I2C1->SR1 & I2C_SR1_ADDR), 1);                                 // wait address transmitted
    disableInterrupts();
    I2C1->SR3;                                                                  // clear flag ADDR
    I2C1->CR2 &= (~I2C_CR2_ACK);                                                // disable acknowledge
    enableInterrupts();
    wait_event(!(I2C1->SR1 & I2C_SR1_BTF), 1);                                  // wait first byte in DR, second in sheeft reg
    disableInterrupts();
    I2C1->CR2 |= I2C_CR2_STOP;                                                  // generate stop
    *data++ = I2C1->DR;                                                         // first byte
    enableInterrupts();
    *data = I2C1->DR;                                                           // second byte
  }
  else if (lenth > 2)
  {
    wait_event(!(I2C1->SR1 & I2C_SR1_ADDR), 1);                                 // wait address transmitted
    disableInterrupts();
    I2C1->SR3;                                                                  // clear flag ADDR
    enableInterrupts();
    while (lenth-- > 3 && tmo)
    {
      wait_event(!(I2C1->SR1 & I2C_SR1_BTF), 1);                                // wait first byte in DR, second in sheeft reg
      *data++ = I2C1->DR;                                                       // read byte
    }
    if (!tmo)
      return 1;
    wait_event(!(I2C1->SR1 & I2C_SR1_BTF), 1);                                  // wait first byte in DR, second in sheeft reg
    I2C1->CR2 &= (~I2C_CR2_ACK);                                                // disable acknowledge
    disableInterrupts();
    *data++ = I2C1->DR;                                                         // read N-2 byte
    I2C1->CR2 |= I2C_CR2_STOP;                                                  // generate stop
    *data++ = I2C1->DR;                                                         // read N-1 byte
    enableInterrupts();
    wait_event(!(I2C1->SR1 & I2C_SR1_RXNE), 1);                                 // wait data from sheeft reg
    *data = I2C1->DR;                                                           // read N byte
  }
  wait_event(I2C1->CR2 & I2C_CR2_STOP, 1);                                      // wait for stop
  I2C1->CR2 &= (~I2C_CR2_POS);                                                  // disable NACK on next transmission
  return 0;
}

uint8_t I2C_WriteByte(uint8_t address, uint8_t data)
{
  wait_event(I2C1->SR3 & I2C_SR3_BUSY, 10);                                     // wait for free bus
  I2C1->CR2 |= I2C_CR2_START;                                                   // send start
  wait_event(!(I2C1->SR1 & I2C_SR1_SB), 1);                                     // wait start transmitted
  I2C1->DR = 0xEE;                                                              // slave address write
  wait_event(!(I2C1->SR1 & I2C_SR1_ADDR), 1);                                   // wait address transmitted
  I2C1->SR3;                                                                    // clear flag ADDR
  wait_event(!(I2C1->SR1 & I2C_SR1_TXE), 1);                                    // wait RD clear
  I2C1->DR = address;                                                           // send reg address
  wait_event(!(I2C1->SR1 & I2C_SR1_TXE), 1);                                    // wait RD clear
  I2C1->DR = data;                                                              // send data
  wait_event(!( (I2C1->SR1 & I2C_SR1_TXE) && (I2C1->SR1 & I2C_SR1_BTF) ), 1);   // data is in register
  I2C1->CR2 |= I2C_CR2_STOP;                                                    // generate stop
  wait_event(I2C1->CR2 & I2C_CR2_STOP, 1);                                      // wait for stop
  return 0;
}
