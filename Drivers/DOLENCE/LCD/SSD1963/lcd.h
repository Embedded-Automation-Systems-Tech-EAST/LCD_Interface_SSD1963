/*
 * lcd.h
 *
 *  Created on: May 15, 2020
 *      Author: willi
 */

#ifndef DOLENCE_LCD_SSD1963_LCD_H_
#define DOLENCE_LCD_SSD1963_LCD_H_

#include <stdint.h>
#include "stm32f4xx_hal.h"


typedef struct
{
	volatile uint16_t LCD_REG;
	volatile uint16_t LCD_RAM;
} LCD_TypeDef;

uint16_t H24_RGB565(uint8_t reverse, uint32_t color24);
void SSD1963_WriteCommand(volatile uint16_t regval);
void SSD1963_WriteData(volatile uint16_t data);
void SSD1963_WriteMultipleData(uint16_t* pData, uint32_t Size);
void LCD_Window(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
void LCD_Rect_Fill(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint32_t color24);
void LCD_Init(void);

#define LCD_BASE        ((uint32_t)(0x6C000000 | 0x0000007E))
#define LCD             ((LCD_TypeDef *) LCD_BASE)

#define TFT_FPS 60ULL

#define TFT_WIDTH				800ULL
#define TFT_HSYNC_BACK_PORCH	46LL
#define TFT_HSYNC_FRONT_PORCH	210ULL
#define TFT_HSYNC_PULSE			1ULL

#define TFT_HEIGHT				480ULL
#define TFT_VSYNC_BACK_PORCH	23ULL
#define TFT_VSYNC_FRONT_PORCH	22ULL
#define TFT_VSYNC_PULSE			1ULL


#define	TFT_HSYNC_PERIOD	(TFT_HSYNC_PULSE + TFT_HSYNC_BACK_PORCH + TFT_WIDTH  + TFT_HSYNC_FRONT_PORCH)
#define	TFT_VSYNC_PERIOD	(TFT_VSYNC_PULSE + TFT_VSYNC_BACK_PORCH + TFT_HEIGHT + TFT_VSYNC_FRONT_PORCH)

#define TFT_PCLK	(TFT_HSYNC_PERIOD * TFT_VSYNC_PERIOD * TFT_FPS)
#define LCD_FPR		((TFT_PCLK * 1048576)/100000000)

#define mHIGH(x) (x >> 8)
#define mLOW(x) (x & 0xFF)

#define MIRROR_V 0
#define MIRROR_H 0

#define LCD_RST_SET   HAL_GPIO_WritePin(GPIOF, GPIO_PIN_4, GPIO_PIN_SET);
#define LCD_RST_RESET HAL_GPIO_WritePin(GPIOF, GPIO_PIN_4, GPIO_PIN_RESET);

#define BLACK 	0x000000 /*   0,   0,   0 */
#define WHITE 	0xFFFFFF /* 255, 255, 255 */
#define RED 	0xFF0000 /* 255,   0,   0 */
#define BLUE 	0x0000FF /*   0,   0, 255 */
#define BLUE_D 	0x0000A0 /*   0,   0, 160 */
#define CYAN 	0x00FFFF /*   0, 255, 255 */
#define CYAN_D	0x008080 /*   0, 128, 128 */
#define YELLOW 	0xFFFF00 /* 255, 255,   0 */
#define MAGENTA 0xFF00FF /* 255,   0, 255 */
#define GREEN 	0x00FF00 /*   0, 255,   0 */
#define GREEN_D 0x008000 /*   0, 128,   0 */
#define PURPLE 	0x800080 /* 128,   0, 128 */
#define TEAL 	0x008080
#define NAVY 	0x000080 /*   0,   0, 128 */
#define SILVER 	0xC0C0C0 /* 192, 192, 192 */
#define GRAY 	0x808080 /* 128, 128, 128 */
#define ORANGE 	0xFFA500 /* 255, 165,   0 */
#define BROWN 	0xA52A2A
#define MAROON 	0x800000 /* 128,   0,   0 */
#define OLIVE 	0x808000 /* 128, 128,   0 */
#define LIME 	0xBFFF00 /* 191, 255,   0 */

//=============================================================================
// SSD1963 commands
//=============================================================================
#define SSD1963_NOP						0x00
#define SSD1963_SOFT_RESET  			0x01
#define SSD1963_GET_POWER_MODE 			0x0A
#define SSD1963_GET_ADDRESS_MODE		0x0B
#define SSD1963_GET_DISPLAY_MODE		0x0D
#define SSD1963_GET_TEAR_EFFECT_STATUS 	0x0E
#define SSD1963_ENTER_SLEEP_MODE		0x10
#define SSD1963_EXIT_SLEEP_MODE			0x11
#define SSD1963_ENTER_PARTIAL_MODE		0x12
#define SSD1963_ENTER_NORMAL_MODE		0x13
#define SSD1963_EXIT_INVERT_MODE		0x20
#define SSD1963_ENTER_INVERT_MODE		0x21
#define SSD1963_SET_GAMMA_CURVE			0x26
#define SSD1963_SET_DISPLAY_OFF			0x28
#define SSD1963_SET_DISPLAY_ON			0x29
#define SSD1963_SET_COLUMN_ADDRESS		0x2A
#define SSD1963_SET_PAGE_ADDRESS		0x2B
#define SSD1963_WRITE_MEMORY_START		0x2C
#define SSD1963_READ_MEMORY_START		0x2E
#define SSD1963_SET_PARTIAL_AREA		0x30
#define SSD1963_SET_SCROLL_AREA			0x33
#define SSD1963_SET_TEAR_OFF			0x34
#define SSD1963_SET_TEAR_ON				0x35
#define SSD1963_SET_ADDRESS_MODE		0x36
#define SSD1963_SET_SCROLL_START		0x37
#define SSD1963_EXIT_IDLE_MODE			0x38
#define SSD1963_ENTER_IDLE_MODE			0x39
#define SSD1963_SET_PIXEL_FORMAT		0x3A
#define SSD1963_WRITE_MEMORY_CONTINUE	0x3C
#define SSD1963_READ_MEMORY_CONTINUE	0x3E
#define SSD1963_SET_TEAR_SCANLINE		0x44
#define SSD1963_GET_SCANLINE			0x45
#define SSD1963_READ_DDB				0xA1
#define SSD1963_SET_LCD_MODE			0xB0
#define SSD1963_GET_LCD_MODE			0xB1
#define SSD1963_SET_HORI_PERIOD			0xB4
#define SSD1963_GET_HORI_PERIOD			0xB5
#define SSD1963_SET_VERT_PERIOD			0xB6
#define SSD1963_GET_VERT_PERIOD			0xB7
#define SSD1963_SET_GPIO_CONF			0xB8
#define SSD1963_GET_GPIO_CONF			0xB9
#define SSD1963_SET_GPIO_VALUE			0xBA
#define SSD1963_GET_GPIO_STATUS			0xBB
#define SSD1963_SET_POST_PROC			0xBC
#define SSD1963_GET_POST_PROC			0xBD
#define SSD1963_SET_PWM_CONF			0xBE
#define SSD1963_GET_PWM_CONF			0xBF
#define SSD1963_GET_LCD_GEN0			0xC0
#define SSD1963_SET_LCD_GEN0			0xC1
#define SSD1963_GET_LCD_GEN1			0xC2
#define SSD1963_SET_LCD_GEN1			0xC3
#define SSD1963_GET_LCD_GEN2			0xC4
#define SSD1963_SET_LCD_GEN2			0xC5
#define SSD1963_GET_LCD_GEN3			0xC6
#define SSD1963_SET_LCD_GEN3			0xC7
#define SSD1963_SET_GPIO0_ROP			0xC8
#define SSD1963_GET_GPIO0_ROP			0xC9
#define SSD1963_SET_GPIO1_ROP			0xCA
#define SSD1963_GET_GPIO1_ROP			0xCB
#define SSD1963_SET_GPIO2_ROP			0xCC
#define SSD1963_GET_GPIO2_ROP			0xCD
#define SSD1963_SET_GPIO3_ROP			0xCE
#define SSD1963_GET_GPIO3_ROP			0xCF
#define SSD1963_SET_DBC_CONF			0xD0
#define SSD1963_GET_DBC_CONF			0xD1
#define SSD1963_SET_DBC_TH				0xD4
#define SSD1963_GET_DBC_TH				0xD5
#define SSD1963_SET_PLL					0xE0
#define SSD1963_SET_PLL_MN				0xE2
#define SSD1963_GET_PLL_MN				0xE3
#define SSD1963_GET_PLL_STATUS			0xE4
#define SSD1963_SET_DEEP_SLEEP			0xE5
#define SSD1963_SET_LSHIFT_FREQ			0xE6
#define SSD1963_GET_LSHIFT_FREQ			0xE7
#define SSD1963_SET_PIXEL_DATA_INTERFACE 0xF0
#define SSD1963_PDI_8BIT				0
#define SSD1963_PDI_12BIT				1
#define SSD1963_PDI_16BIT				2
#define SSD1963_PDI_16BIT565			3
#define SSD1963_PDI_18BIT				4
#define SSD1963_PDI_24BIT				5
#define SSD1963_PDI_9BIT				6
#define SSD1963_GET_PIXEL_DATA_INTERFACE 0xF1


#define SSD1963_ADDR_MODE_FLIP_VERT           (1 << 0)
#define SSD1963_ADDR_MODE_FLIP_HORZ           (1 << 1)
#define SSD1963_ADDR_MODE_LATCH_RIGHT_TO_LEFT (1 << 2)
#define SSD1963_ADDR_MODE_BGR                 (1 << 3)
#define SSD1963_ADDR_MODE_REFRESH_BOTTOM_UP   (1 << 4)
#define SSD1963_ADDR_MODE_PAG_COL_ADDR_ORDER  (1 << 5)
#define SSD1963_ADDR_MODE_COL_ADDR_ORDER      (1 << 6)
#define SSD1963_ADDR_MODE_PAGE_ADDR_ORDER     (1 << 7)

/* Set the pannel data width */
#define LCD_PANEL_DATA_WIDTH_24BIT 				(1<<5)						// 18bit default
/* Set the color deeph enhancement */
#define LCD_PANEL_ENABLE_FRC					((1<<3) | (1<<4))
#define LCD_PANEL_ENABLE_DITHERING				(1<<4)						// no enhancement default
/* Set the dot clock pulse polarity */
#define LCD_PANEL_LSHIFT_FALLING_EDGE			(1<<2)						// default rising edge
/* Set the horizontal sync pulse polarity */
#define LCD_PANEL_LLINE_ACTIVE_HIGH				(1<<1)						// default active low
/* Set the vertical sync pulse polarity */
#define LCD_PANEL_LFRAME_ACTIVE_HIGH			(1<0)						// default active low
/* Set the lcd panel mode */
#define LCD_PANEL_MODE_TTL						((1<<7) << 8)				// default mode is Hsync+Vsync +DE
/* Set the lcd panel interface type */										// default TFT mode
#define LCD_PANEL_TYPE_SERIAL_RGB_MODE			((1<<6) << 8)				// Serial RGB mode
#define LCD_PANEL_TYPE_SERIAL_RGB_DUMMY_MODE	(((1<<5) | (1<<6)) << 8)	// Serial RGB+dummy mode




#endif /* DOLENCE_LCD_SSD1963_LCD_H_ */
