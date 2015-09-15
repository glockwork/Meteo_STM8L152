#include "stm8l15x.h"

#define S8  0x0000000000000100ul
#define S9  0x0000000000000200ul
#define S10 0x0000000000000400ul
#define S11 0x0000000000000800ul
#define S12 0x0000000000001000ul
#define S13 0x0000000000002000ul
#define S14 0x0000000000004000ul
#define S15 0x0000000000008000ul
#define S16 0x0000000000010000ul
#define S17 0x0000000000020000ul
#define S18 0x0000000000040000ul
#define S19 0x0000000000080000ul
#define S20 0x0000000000100000ul
#define S21 0x0000000000200000ul
#define S22 0x0000000000400000ul
#define S23 0x0000000000800000ul
#define S24 0x0000000001000000ul

void LCD_HardwareInit(void);
void LCD_WriteError(void);
void LCD_PutDigit(char dig, char pos);
void LCD_WriteInt(int num);
void LCD_WriteFloat(float num, char prec);
