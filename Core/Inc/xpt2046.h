/*
 * xpt2046.h
 *
 *  Created on: May 19, 2022
 *      Author: tantrum
 */

#ifndef INC_XPT2046_H_
#define INC_XPT2046_H_

#include <stdbool.h>

extern SPI_HandleTypeDef hspi2;

#define TOUCH_SPI_PTR 		 &hspi2

#define TOUCH_CS_SELECT      HAL_GPIO_WritePin(TOUCH_CS_GPIO_Port, TOUCH_CS_Pin, GPIO_PIN_RESET)
#define TOUCH_CS_UNSELECT    HAL_GPIO_WritePin(TOUCH_CS_GPIO_Port, TOUCH_CS_Pin, GPIO_PIN_SET)

// change depending on screen orientation
#define ILI9341_TOUCH_SCALE_X 240
#define ILI9341_TOUCH_SCALE_Y 320

// to calibrate uncomment UART_Printf line in ili9341_touch.c
#define ILI9341_TOUCH_MIN_RAW_X 1500
#define ILI9341_TOUCH_MAX_RAW_X 31000
#define ILI9341_TOUCH_MIN_RAW_Y 3276
#define ILI9341_TOUCH_MAX_RAW_Y 30110

bool ILI9341_TouchGetCoordinates(uint16_t *x, uint16_t *y);

#endif /* INC_XPT2046_H_ */
