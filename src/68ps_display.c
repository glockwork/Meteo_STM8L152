#include "68ps_display.h"
#include <math.h>

const unsigned char digits[18][3] = {
	{0x03, 0x06, 0x05}, 	// 0	index = 0
	{0x00, 0x04, 0x04}, 	// 1	index = 1
	{0x02, 0x02, 0x07}, 	// 2	index = 2
	{0x02, 0x06, 0x06}, 	// 3	index = 3
	{0x01, 0x04, 0x06}, 	// 4	index = 4
	{0x03, 0x06, 0x02}, 	// 5	index = 5
	{0x03, 0x06, 0x03}, 	// 6	index = 6
	{0x02, 0x04, 0x04}, 	// 7	index = 7
	{0x03, 0x06, 0x07}, 	// 8	index = 8
	{0x03, 0x06, 0x06}, 	// 9	index = 9
	{0x03, 0x04, 0x07},	// a	index = 10
	{0x01, 0x06, 0x03},	// b	index = 11
	{0x03, 0x02, 0x01},	// c	index = 12
	{0x00, 0x06, 0x07},	// d	index = 13
	{0x03, 0x02, 0x03},	// e	index = 14
	{0x03, 0x00, 0x03},	// f	index = 15
	{0x00, 0x00, 0x02},	// -	index = 16
	{0x00, 0x00, 0x00}	// empty        index = 17
};

void LCD_HardwareInit(void)
{
  unsigned long segments = S8 | S9 | S10 | S11 | S12 | S13 | S14 | S15 | S16 | S17 | S18 | S19;
  CLK_RTCClockConfig(CLK_RTCCLKSource_LSI, CLK_RTCCLKDiv_2);
  CLK_PeripheralClockConfig(CLK_Peripheral_RTC, ENABLE);
  
  CLK_PeripheralClockConfig(CLK_Peripheral_LCD, ENABLE);
  LCD_Init(LCD_Prescaler_4, LCD_Divider_24, LCD_Duty_1_3, LCD_Bias_1_3, LCD_VoltageSource_External);
  LCD_PortMaskConfig(LCD_PortMaskRegister_1, (uint8_t)(segments >> 8));
  LCD_PortMaskConfig(LCD_PortMaskRegister_2, (uint8_t)(segments >> 16));
  LCD_PortMaskConfig(LCD_PortMaskRegister_3, (uint8_t)(segments >> 24));
  LCD_PulseOnDurationConfig(LCD_PulseOnDuration_0);
  LCD_ContrastConfig(LCD_Contrast_Level_7);
  LCD_Cmd(ENABLE);
}

void LCD_WriteError(void)
{ 
  LCD->RAM[1] = 0x03;
  LCD->RAM[2] = 0x00;
  LCD->RAM[4] = 0x20;
  LCD->RAM[5] = 0xC0;
  LCD->RAM[8] = 0xDB;
  LCD->RAM[9] = 0x06;
}

void LCD_PutDigit(char dig, char pos)
{
  if (dig > 9)
    dig %= 10;
  unsigned char shift;
  switch (pos)
  {
    case 1: shift = 0; break;
    case 2: shift = 3; break;
    case 3: shift = 6; break;
    case 4: shift = 9; break;
    default: shift = 0;
  }
  unsigned int COM0 = digits[dig][0] << shift;
  unsigned int COM1 = digits[dig][1] << shift;
  unsigned int COM2 = digits[dig][2] << shift;
  
  LCD->RAM[1] = (unsigned char)COM0;
  LCD->RAM[2] = (unsigned char)(COM0 >> 8);
  LCD->RAM[4] = (unsigned char)(COM1 << 4);
  LCD->RAM[5] = (unsigned char)(COM1 >> 4);
  LCD->RAM[8] = (unsigned char)COM2;
  LCD->RAM[9] = (unsigned char)(COM2 >> 8);
}

void LCD_WriteInt(int num)
{
  unsigned int COM0, COM1, COM2;
  
  int s1, s2, s3, s4;
  if ((num < -999) || (num > 9999)) {
    LCD_WriteError();
    return;
  }
	
  if (num >= 0) {
    if (num < 1000)
      s1 = 17;
    else
      s1 = num / 1000;
    if (num < 100)
      s2 = 17;
    else
      s2 = (num / 100) % 10;
    if (num < 10)
      s3 = 17;
    else
      s3 = (num / 10) % 10;
  }
  else {
    num = -num;
    if (num < 10) {
      s1 = 17;
      s2 = 17;
      s3 = 16;
    }
    else if (num < 100) {
      s1 = 17;
      s2 = 16;
      s3 = (num / 10) % 10;
    }
    else if (num < 1000) {
      s1 = 16;
      s2 = (num / 100) % 10;
      s3 = (num / 10) % 10;
    }
  }
	
  s4 = (num) % 10;
		
  COM0 = digits[s1][0] | (digits[s2][0]<<3) | (digits[s3][0]<<6) | (digits[s4][0]<<9);
  COM1 = digits[s1][1] | (digits[s2][1]<<3) | (digits[s3][1]<<6) | (digits[s4][1]<<9);
  COM2 = digits[s1][2] | (digits[s2][2]<<3) | (digits[s3][2]<<6) | (digits[s4][2]<<9);
  
  LCD->RAM[1] = (unsigned char)COM0;
  LCD->RAM[2] = (unsigned char)(COM0 >> 8);
  LCD->RAM[4] = (unsigned char)(COM1 << 4);
  LCD->RAM[5] = (unsigned char)(COM1 >> 4);
  LCD->RAM[8] = (unsigned char)COM2;
  LCD->RAM[9] = (unsigned char)(COM2 >> 8);
}

void LCD_WriteFloat(float num, char prec)
{
  unsigned int COM0, COM1, COM2;
  int point, s1, s2, s3, s4;
  int mult = (int)pow(10, prec);        // something strange in pow(10, 2)
  if (prec == 2)                        // from result num*mult is subtracted 2 and if to do this in one line subtract 1
    mult = 100;                         // so in the case of pow(10, 2) mult is equal to 100 forcibly
  int i_num = (int)(num * mult);
	
  if (i_num >= 0) {
    while (i_num > 10000) {
      if (prec > 0) {
        prec--;
        i_num = (int)(num * pow(10, prec));
      }
      else {
        LCD_WriteError();
        return;
      }
    }

    if (i_num < 1000 && prec < 3)
      s1 = 17;
    else
      s1 = i_num / 1000;
    if (i_num < 100 && prec < 2)
      s2 = 17;
    else
      s2 = (i_num / 100) % 10;
    if (i_num < 10 && prec < 1)
      s3 = 17;
    else
      s3 = (i_num / 10) % 10;
    
    s4 = (i_num) % 10;
  }
  else {
    i_num = -i_num;
		
    while (i_num > 1000) {
      if (prec > 0) {
        prec--;
        i_num = (int)(-num * pow(10, prec));
      }
      else {
        LCD_WriteError();
        return;
      }
    }
		
    if (i_num < 10) {
      s1 = 17;
      s2 = 17;
      s3 = 16;
    }
    else if (i_num < 100) {
      s1 = 17;
      s2 = 16;
      s3 = (i_num / 10) % 10;
    }
    else if (i_num < 1000) {
      s1 = 16;
      s2 = (i_num / 100) % 10;
      s3 = (i_num / 10) % 10;
    }
    s4 = (i_num) % 10;
  }
	
  switch (prec)
  {
    case 1: point = 1<<9; break;
    case 2: point = 1<<6; break;
    case 3: point = 1<<3; break;
    case 4: point = 1; break;
    default: point = 0;
  }
  
  COM0 = digits[s1][0] | (digits[s2][0]<<3) | (digits[s3][0]<<6) | (digits[s4][0]<<9);
  COM1 = digits[s1][1] | (digits[s2][1]<<3) | (digits[s3][1]<<6) | (digits[s4][1]<<9) | point;
  COM2 = digits[s1][2] | (digits[s2][2]<<3) | (digits[s3][2]<<6) | (digits[s4][2]<<9);
        
  LCD->RAM[1] = (unsigned char)COM0;
  LCD->RAM[2] = (unsigned char)(COM0 >> 8);
  LCD->RAM[4] = (unsigned char)(COM1 << 4);
  LCD->RAM[5] = (unsigned char)(COM1 >> 4);
  LCD->RAM[8] = (unsigned char)COM2;
  LCD->RAM[9] = (unsigned char)(COM2 >> 8);
}
