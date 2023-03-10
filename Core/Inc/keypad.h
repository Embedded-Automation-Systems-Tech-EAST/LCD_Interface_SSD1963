/**
 * @author  Tilen Majerle
 * @email   tilen@majerle.eu
 * @website http://stm32f4-discovery.com
 * @link    http://stm32f4-discovery.com/2014/09/library-32-matrix-keypad-on-stm32f4xx
 * @version v2.0
 * @ide     Keil uVision
 * @license GNU GPL v3
 * @brief   Matrix keyboard for STM32F4xx device
 *	
@verbatim
   ----------------------------------------------------------------------
    Copyright (C) Tilen Majerle, 2015
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    any later version.
     
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    
    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
   ----------------------------------------------------------------------
@endverbatim
 */
#ifndef _KEYPAD_H
#define _KEYPAD_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @addtogroup TM_STM32F4xx_Libraries
 * @{
 */

/**
 * @defgroup KEYPAD
 * @brief    Matrix keyboard for STM32
 * @{
 *
 * Supported Versions:
 * 		4x4 = 4 rows and 4 columns
 *		
 * 		3x4 = 4 rows and 3 columns
 *
 * \par Default pinout
 *
@verbatim
Keypad		STM32F4xx		Description

C1			PD0				Column 1
C2			PD1				Column 2
C3			PD2				Column 3
C4			PD3				Column 4
R1			PC1				Row 1
R2			PC2				Row 2
R3			PC3				Row 3
R4			PC5				Row 4
@endverbatim
 *
 * You can change pinout. Open defines.h file and change lines below:
 *
@verbatim
//Change rows pinout. Change X from 1 to 4, according to ROW you want to change
#define KEYPAD_ROW_X_PORT			GPIOC
#define KEYPAD_ROW_X_PIN			GPIO_PIN_0

//Change columns pinout. Change X from 1 to 4, according to COLUMN you want to change
#define KEYPAD_COLUMN_X_PORT		GPIOD
#define KEYPAD_COLUMN_X_PIN			GPIO_PIN_0
@endverbatim
 *
 * You can set this for all 4 columns and all 4 rows.
 *
 * \par Changelog
 *
@verbatim 
 Version 2.0
  - April 11, 2015
  - Library rewritten, now with more accuracy and interrupt based
  
 Version 1.1
  - March 11, 2015
  - Added support for my new GPIO library
  
 Version 1.0
  - First release
@endverbatim
 *
 * \par Dependencies
 *
@verbatim
 - STM32F4xx
 - STM32F4xx RCC
 - defines.h
@endverbatim
 */
/**
 * Library dependencies
 * - STM32F4xx
 * - defines.h
 * - TM GPIO
 */
/**
 * Includes
 */
#include "stm32f4xx.h"


/**
 * @defgroup KEYPAD_Macros
 * @brief    Library defines
 * @{
 */
 

/* Interval between 2 reads */
#ifndef KEYPAD_INTERVAL
#define KEYPAD_INTERVAL        100
#endif

/* Keypad no pressed */
#define KEYPAD_NOT_PRESSED			(uint8_t)0xFF

/**
 * @}
 */

/**
 * @brief  Keypad Button Enum
 */
typedef enum {
	KEYPAD_Button_0 = 0x00,                     /*!< Button 0 code */
	KEYPAD_Button_1 = 0x01,                     /*!< Button 1 code */
	KEYPAD_Button_2 = 0x02,                     /*!< Button 2 code */
	KEYPAD_Button_3 = 0x03,                     /*!< Button 3 code */
	KEYPAD_Button_4 = 0x04,                     /*!< Button 4 code */
	KEYPAD_Button_5 = 0x05,                     /*!< Button 5 code */
	KEYPAD_Button_6 = 0x06,                     /*!< Button 6 code */
	KEYPAD_Button_7 = 0x07,                     /*!< Button 7 code */
	KEYPAD_Button_8 = 0x08,                     /*!< Button 8 code */
	KEYPAD_Button_9 = 0x09,                     /*!< Button 9 code */
	KEYPAD_Button_STAR = 0x0A,                  /*!< Button STAR code */
	KEYPAD_Button_HASH = 0x0B,                  /*!< Button HASH code */
	KEYPAD_Button_A = 0x0C,	                   /*!< Button A code. Only on large size */
	KEYPAD_Button_B = 0x0D,	                   /*!< Button B code. Only on large size */
	KEYPAD_Button_C = 0x0E,	                   /*!< Button C code. Only on large size */
	KEYPAD_Button_D = 0x0F,	                   /*!< Button D code. Only on large size */
	KEYPAD_Button_NOT_PRESSED = KEYPAD_NOT_PRESSED /*!< No button pressed */
} KEYPAD_Button_t;

/**
 * @brief  Keypad Size Enum
 */
typedef enum {
	KEYPAD_Type_Large = 0x00, /*!< Keypad 4x4 size */
	KEYPAD_Type_Small         /*!< Keypad 3x4 size */
} KEYPAD_Type_t;

/**
 * @}
 */

/**
 * @defgroup KEYPAD_Functions
 * @brief    Library Functions
 * @{
 */
 
/**
 * @brief  Initializes keypad
 * @input  type: Keypad type you will use. This parameter will be a value of KEYPAD_Type_t Enum
 * @retval None
 */
void KEYPAD_Init(KEYPAD_Type_t type);

/**
 * @brief  Read the keypad
 * @input  None
 * @retval Button status. This parameter will be a value of KEYPAD_Button_t Enum
 */
KEYPAD_Button_t KEYPAD_Read(void);

/**
 * @brief  Updates keypad
 * @warning   This function must be called from interrupt routine every 1ms
 * @input  None
 * @retval None
 */
KEYPAD_Button_t KEYPAD_Update(void);

/**
 * @}
 */
 
/**
 * @}
 */
 
/**
 * @}
 */

/* C++ detection */
#ifdef __cplusplus
}
#endif

#endif

