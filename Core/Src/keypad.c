/**	
 * |----------------------------------------------------------------------
 * | Copyright (C) Tilen Majerle, 2014
 * | 
 * | This program is free software: you can redistribute it and/or modify
 * | it under the terms of the GNU General Public License as published by
 * | the Free Software Foundation, either version 3 of the License, or
 * | any later version.
 * |  
 * | This program is distributed in the hope that it will be useful,
 * | but WITHOUT ANY WARRANTY; without even the implied warranty of
 * | MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * | GNU General Public License for more details.
 * | 
 * | You should have received a copy of the GNU General Public License
 * | along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * |----------------------------------------------------------------------
 */
#include "keypad.h"
#include <main.h>
/* Pins configuration, columns are outputs */
#define KEYPAD_COL_1_SET			HAL_GPIO_WritePin(KEYPAD_COL_1_GPIO_Port, KEYPAD_COL_1_Pin, GPIO_PIN_SET);
#define KEYPAD_COL_1_RESET		HAL_GPIO_WritePin(KEYPAD_COL_1_GPIO_Port, KEYPAD_COL_1_Pin, GPIO_PIN_RESET);
#define KEYPAD_COL_2_SET			HAL_GPIO_WritePin(KEYPAD_COL_2_GPIO_Port, KEYPAD_COL_2_Pin, GPIO_PIN_SET);  
#define KEYPAD_COL_2_RESET		HAL_GPIO_WritePin(KEYPAD_COL_2_GPIO_Port, KEYPAD_COL_2_Pin, GPIO_PIN_RESET);
#define KEYPAD_COL_3_SET			HAL_GPIO_WritePin(KEYPAD_COL_3_GPIO_Port, KEYPAD_COL_3_Pin, GPIO_PIN_SET);  
#define KEYPAD_COL_3_RESET		HAL_GPIO_WritePin(KEYPAD_COL_3_GPIO_Port, KEYPAD_COL_3_Pin, GPIO_PIN_RESET);
#define KEYPAD_COL_4_SET			HAL_GPIO_WritePin(KEYPAD_COL_4_GPIO_Port, KEYPAD_COL_4_Pin, GPIO_PIN_SET);  
#define KEYPAD_COL_4_RESET		HAL_GPIO_WritePin(KEYPAD_COL_4_GPIO_Port, KEYPAD_COL_4_Pin, GPIO_PIN_RESET);
																																												 
/* Read input pins */
#define KEYPAD_ROW_1_POLL			(!HAL_GPIO_ReadPin(KEYPAD_ROW_1_GPIO_Port, KEYPAD_ROW_1_Pin))
#define KEYPAD_ROW_2_POLL			(!HAL_GPIO_ReadPin(KEYPAD_ROW_2_GPIO_Port, KEYPAD_ROW_2_Pin))
#define KEYPAD_ROW_3_POLL			(!HAL_GPIO_ReadPin(KEYPAD_ROW_3_GPIO_Port, KEYPAD_ROW_3_Pin))
#define KEYPAD_ROW_4_POLL			(!HAL_GPIO_ReadPin(KEYPAD_ROW_4_GPIO_Port, KEYPAD_ROW_4_Pin))

uint8_t KEYPAD_INT_Buttons[4][4] = {
	{KEYPAD_Button_1, KEYPAD_Button_2, KEYPAD_Button_3, KEYPAD_Button_A},
	{KEYPAD_Button_4, KEYPAD_Button_5, KEYPAD_Button_6, KEYPAD_Button_B},
	{KEYPAD_Button_7, KEYPAD_Button_8, KEYPAD_Button_9, KEYPAD_Button_C},
	{KEYPAD_Button_STAR, KEYPAD_Button_0, KEYPAD_Button_HASH, KEYPAD_Button_D},
};

/* Private functions */
void KEYPAD_INT_SetColumn(uint8_t column);
uint8_t KEYPAD_INT_CheckRow(uint8_t column);
uint8_t KEYPAD_INT_Read(void);

/* Private variables */
KEYPAD_Type_t KEYPAD_INT_KeypadType;
static KEYPAD_Button_t KeypadStatus = KEYPAD_Button_NOT_PRESSED;

void KEYPAD_Init(KEYPAD_Type_t type) {
	/* Set keyboard type */
	KEYPAD_INT_KeypadType = type;
	/* All columns high */
	KEYPAD_INT_SetColumn(0);
}

KEYPAD_Button_t KEYPAD_Read(void) {
	KEYPAD_Button_t temp;
	
	/* Get keypad status */
	temp = KeypadStatus;
	
	/* Reset keypad status */
	KeypadStatus = KEYPAD_Button_NOT_PRESSED;
	
	return temp;
}

/* Private */
void KEYPAD_INT_SetColumn(uint8_t column) {
	/* Set rows high */
	KEYPAD_COL_1_SET;
	KEYPAD_COL_2_SET;
	KEYPAD_COL_3_SET;
	if (KEYPAD_INT_KeypadType == KEYPAD_Type_Large) {
		KEYPAD_COL_4_SET;
	}
	
	/* Set column low */
	if (column == 1) {
		KEYPAD_COL_1_RESET;
	}
	if (column == 2) {
		KEYPAD_COL_2_RESET;
	}
	if (column == 3) {
		KEYPAD_COL_3_RESET;
	}
	if (column == 4) {
		KEYPAD_COL_4_RESET;
	}
}

uint8_t KEYPAD_INT_CheckRow(uint8_t column) {
	/* Read rows */
	
	/* Scan row 1 */
	if (KEYPAD_ROW_1_POLL) {
		return KEYPAD_INT_Buttons[0][column - 1];	
	}
	/* Scan row 2 */
	if (KEYPAD_ROW_2_POLL) {
		return KEYPAD_INT_Buttons[1][column - 1];
	}
	/* Scan row 3 */
	if (KEYPAD_ROW_3_POLL) {
		return KEYPAD_INT_Buttons[2][column - 1];
	}
	/* Scan row 4 */
	if (KEYPAD_INT_KeypadType == KEYPAD_Type_Large && KEYPAD_ROW_4_POLL) {
		return KEYPAD_INT_Buttons[3][column - 1];
	}
	
	/* Not pressed */
	return KEYPAD_NOT_PRESSED;
}

uint8_t KEYPAD_INT_Read(void) {
	uint8_t check;
	/* Set row 1 to LOW */
	KEYPAD_INT_SetColumn(1);
	/* Check rows */
	check = KEYPAD_INT_CheckRow(1);
	if (check != KEYPAD_NOT_PRESSED) {
		return check;
	}
	
	/* Set row 2 to LOW */
	KEYPAD_INT_SetColumn(2);
	/* Check columns */
	check = KEYPAD_INT_CheckRow(2);
	if (check != KEYPAD_NOT_PRESSED) {
		return check;
	}
	
	/* Set row 3 to LOW */
	KEYPAD_INT_SetColumn(3);
	/* Check columns */
	check = KEYPAD_INT_CheckRow(3);
	if (check != KEYPAD_NOT_PRESSED) {
		return check;
	}

	if (KEYPAD_INT_KeypadType == KEYPAD_Type_Large) {
		/* Set column 4 to LOW */
		KEYPAD_INT_SetColumn(4);
		/* Check rows */
		check = KEYPAD_INT_CheckRow(4);
		if (check != KEYPAD_NOT_PRESSED) {
			return check;
		}
	}
	
	/* Not pressed */
	return KEYPAD_NOT_PRESSED;
}

KEYPAD_Button_t KEYPAD_Update(void) {
	static uint16_t millis = 0;
	
	/* Every X ms read */
	if (++millis >= KEYPAD_INTERVAL && KeypadStatus == KEYPAD_Button_NOT_PRESSED) {
		/* Reset */
		millis = 0;
		
		/* Read keyboard */
		KeypadStatus = (KEYPAD_Button_t) KEYPAD_INT_Read();
	}
	return KeypadStatus;
}


