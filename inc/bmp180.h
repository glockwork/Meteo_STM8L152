#include "stm8l15x.h"
#include "stm8l15x_i2c.h"

#define SYS_CLOCK       16ul            // sys_clock in MHz
#define I2C_CLOCK_FREQ  320000ul        // i2c clock in Hz
#define I2C_OWN_ADDR    0x01            // 
#define I2C_PORT        GPIOC
#define I2C_SDA_PIN     GPIO_Pin_0
#define I2C_SCL_PIN     GPIO_Pin_1
#define P_AT_SEA_LEVEL  100800l

typedef enum
{
  PressureMeasurementMode_ULP = (uint8_t)0,      // ultra low power mode
  PressureMeasurementMode_ST =  (uint8_t)1,      // standard mode
  PressureMeasurementMode_HR =  (uint8_t)2,      // high resolution mode
  PressureMeasurementMode_UHR = (uint8_t)3       // ultra high resolution mode
} PressureMeasurementMode;

/* Initialization & reset */
void BMP180_Init(void);
void BMP180_SoftwareReset(void);
uint8_t BMP180_GetId(void);
/* Measuring temperature */
void BMP180_StartMeasureTemp(void);
int16_t BMP180_GetTemp(void);
/* Measuring pressure & altitude*/
void BMP180_StartMeasurePressure(PressureMeasurementMode mode);
int32_t BMP180_GetPressureInPa(void);
float BMP180_GetPressureInMMHG(void);
float BMP180_GetAltitude(void);
