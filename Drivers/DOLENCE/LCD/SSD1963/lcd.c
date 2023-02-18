/*
 * lcd.c
 *
 *  Created on: May 15, 2020
 *      Author: willi
 */

#include "lcd.h"



/*
 * lcd.c
 *
 *  Created on: May 15, 2020
 *      Author: willi
 */

#include "lcd.h"
#include "cmsis_os.h"


void SSD1963_WriteCommand(volatile uint16_t regval)
{
	LCD->LCD_REG=regval;
}

void SSD1963_WriteData(volatile uint16_t data)
{
	LCD->LCD_RAM=data;
}

void SSD1963_WriteMultipleData(uint16_t* pData, uint32_t Size)
{
	uint32_t  i;

	for (i = 0; i < Size; i++)
	{
		LCD->LCD_RAM=pData[i];
	}
}


inline uint16_t RGB(uint8_t r, uint8_t g, uint8_t b)
{
	return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

uint16_t H24_RGB565(uint8_t reverse, uint32_t color24)
{
	uint8_t b = (color24 >> 16) & 0xFF;
	uint8_t g = (color24 >> 8) & 0xFF;
	uint8_t r = color24 & 0xFF;
	if (reverse) return ((b / 8) << 11) | ((g / 4) << 5) | (r / 8);
	else return ((r / 8) << 11) | ((g / 4) << 5) | (b / 8);
}


void LCD_Window(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
	SSD1963_WriteCommand(SSD1963_SET_COLUMN_ADDRESS);
	SSD1963_WriteData(x1 >> 8);
	SSD1963_WriteData(x1 & 0x00FF);
	SSD1963_WriteData(x2 >> 8);
	SSD1963_WriteData(x2 & 0x00FF);
	SSD1963_WriteCommand(SSD1963_SET_PAGE_ADDRESS);
	SSD1963_WriteData(y1 >> 8);
	SSD1963_WriteData(y1 & 0x00FF);
	SSD1963_WriteData(y2 >> 8);
	SSD1963_WriteData(y2 & 0x00FF);
	SSD1963_WriteCommand(SSD1963_WRITE_MEMORY_START);
}

void LCD_Pixel(uint16_t x, uint16_t y, uint32_t color24)
{
	LCD_Window(x, y, x, y);
	SSD1963_WriteData(H24_RGB565(1, color24));
}

void LCD_Rect_Fill(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint32_t color24)
{
	uint32_t i = 0;
	color24 = H24_RGB565(1, color24);
	LCD_Window(y, x, y + h - 1, x + w - 1);
	for (i = 0; i < w * h; i++)  {
		SSD1963_WriteData(color24);
	}
}



void LCD_Init(void)
{
	/*
	LCD_RST_SET
	HAL_Delay(300);
	LCD_RST_RESET
	HAL_Delay(200);
	*/

	osDelay(150);
	SSD1963_WriteCommand(SSD1963_SOFT_RESET);
	SSD1963_WriteCommand(SSD1963_SOFT_RESET);
	osDelay(150);

	//LCD_Send_Cmd(LCD_DISPLAY_ON);
	SSD1963_WriteCommand(SSD1963_SET_PLL_MN);  //set frequency
	SSD1963_WriteData(49);   // PLLclk = REFclk * 50 (500MHz)
	SSD1963_WriteData(4);	// SYSclk = PLLclk / 5  (100MHz)
	SSD1963_WriteData(4);	// dummy

	SSD1963_WriteCommand(SSD1963_SET_PLL);
	SSD1963_WriteData(0x01);
	osDelay(100);  // Wait for 100us to let the PLL stable and read the PLL lock status bit.
	SSD1963_WriteCommand(SSD1963_SET_PLL);
	SSD1963_WriteData(0x03);  // 5. Switch the clock source to PLL

	SSD1963_WriteCommand(SSD1963_SET_LCD_MODE);
	//SSD1963_WriteData(0x24);
	SSD1963_WriteData(0x20);
	SSD1963_WriteData(0x00);
	SSD1963_WriteData(mHIGH((TFT_WIDTH - 1)));
	SSD1963_WriteData(mLOW((TFT_WIDTH - 1)));
	SSD1963_WriteData(mHIGH((TFT_HEIGHT - 1)));
	SSD1963_WriteData(mLOW((TFT_HEIGHT - 1)));
	SSD1963_WriteData(0x00);

	SSD1963_WriteCommand(SSD1963_SET_ADDRESS_MODE);
	SSD1963_WriteData(0x08); //0x28

	SSD1963_WriteCommand(SSD1963_SET_PIXEL_DATA_INTERFACE);
	SSD1963_WriteData(SSD1963_PDI_16BIT565);
	//SSD1963_WriteCommand(SSD1963_SET_PIXEL_FORMAT);
	//SSD1963_WriteData(0x50);

	//SSD1963_WriteCommand(SSD1963_ENTER_NORMAL_MODE);


	SSD1963_WriteCommand(SSD1963_SET_LSHIFT_FREQ);
	SSD1963_WriteData((LCD_FPR >> 16) & 0xFF);
	SSD1963_WriteData((LCD_FPR >> 8) & 0xFF);
	SSD1963_WriteData(LCD_FPR & 0xFF);

	SSD1963_WriteCommand(SSD1963_SET_HORI_PERIOD);
	SSD1963_WriteData(mHIGH(TFT_HSYNC_PERIOD));
	SSD1963_WriteData(mLOW(TFT_HSYNC_PERIOD));
	SSD1963_WriteData(mHIGH((TFT_HSYNC_PULSE + TFT_HSYNC_BACK_PORCH)));
	SSD1963_WriteData(mLOW((TFT_HSYNC_PULSE + TFT_HSYNC_BACK_PORCH)));
	SSD1963_WriteData(TFT_HSYNC_PULSE);
	SSD1963_WriteData(0x00);
	SSD1963_WriteData(0x00);
	SSD1963_WriteData(0x00);

	SSD1963_WriteCommand(SSD1963_SET_VERT_PERIOD);
	SSD1963_WriteData(mHIGH(TFT_VSYNC_PERIOD));
	SSD1963_WriteData(mLOW(TFT_VSYNC_PERIOD));
	SSD1963_WriteData(mHIGH((TFT_VSYNC_PULSE + TFT_VSYNC_BACK_PORCH)));
	SSD1963_WriteData(mLOW((TFT_VSYNC_PULSE + TFT_VSYNC_BACK_PORCH)));
	SSD1963_WriteData(TFT_VSYNC_PULSE);
	SSD1963_WriteData(0x00);
	SSD1963_WriteData(0x00);

	SSD1963_WriteCommand(SSD1963_SET_TEAR_ON);
	SSD1963_WriteData(0x00);


	SSD1963_WriteCommand(SSD1963_SET_DISPLAY_ON); 		//SET display on
	//LCD_Rect_Fill(0, 	0, 	 400, 240, BLACK);
	//LCD_Rect_Fill(0, 	240, 400, 240, SILVER);
	//LCD_Rect_Fill(400, 	0, 	 400, 240, WHITE);
	//LCD_Rect_Fill(400, 	240, 400, 240, RED);
	//osDelay(2000);
	//LCD_Rect_Fill(0, 	0, 	 800, 480, BLACK);
}
