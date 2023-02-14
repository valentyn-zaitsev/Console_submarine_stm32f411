/*
 * xpt2046_touch.c
 *
 *  Created on: 5 апр. 2020 г.
 *      Author: dima
 */


#include "main.h"
#include "xpt2046.h"
#include "ili9341.h"



#define READ_X 0xD0
#define READ_Y 0x90

static void ILI9341_TouchSelect() {
	//HAL_GPIO_WritePin(TOUCH_CS_GPIO_Port, TOUCH_CS_Pin, GPIO_PIN_RESET);
}

void ILI9341_TouchUnselect() {
	//HAL_GPIO_WritePin(TOUCH_CS_GPIO_Port, TOUCH_CS_Pin, GPIO_PIN_SET);
}

bool ILI9341_TouchGetCoordinates(uint16_t* x, uint16_t* y) {
    static const uint8_t cmd_read_x[] = { READ_X };
    static const uint8_t cmd_read_y[] = { READ_Y };
    static const uint8_t zeroes_tx[] = { 0x00, 0x00 };

    ILI9341_TouchSelect();

    uint32_t avg_x = 0;
    uint32_t avg_y = 0;
    uint8_t nsamples = 0;
    for(uint8_t i = 0; i < 2; i++) {

        nsamples++;

        HAL_SPI_Transmit(TOUCH_SPI_PTR, (uint8_t*)cmd_read_y, sizeof(cmd_read_y), 1000);
        uint8_t y_raw[2];
        HAL_SPI_TransmitReceive(TOUCH_SPI_PTR, (uint8_t*)zeroes_tx, y_raw, sizeof(y_raw), 1000);

        HAL_SPI_Transmit(TOUCH_SPI_PTR, (uint8_t*)cmd_read_x, sizeof(cmd_read_x), 1000);
        uint8_t x_raw[2];
        HAL_SPI_TransmitReceive(TOUCH_SPI_PTR, (uint8_t*)zeroes_tx, x_raw, sizeof(x_raw), 1000);

        avg_x += (((uint16_t)x_raw[0]) << 8) | ((uint16_t)x_raw[1]);
        avg_y += (((uint16_t)y_raw[0]) << 8) | ((uint16_t)y_raw[1]);
    }

    ILI9341_TouchUnselect();

    if(nsamples < 2)
        return false;

    uint32_t raw_x = (avg_x / 16);
    if(raw_x < ILI9341_TOUCH_MIN_RAW_X) raw_x = ILI9341_TOUCH_MIN_RAW_X;
    if(raw_x > ILI9341_TOUCH_MAX_RAW_X) raw_x = ILI9341_TOUCH_MAX_RAW_X;

    uint32_t raw_y = (avg_y / 16);
    if(raw_y < ILI9341_TOUCH_MIN_RAW_X) raw_y = ILI9341_TOUCH_MIN_RAW_Y;
    if(raw_y > ILI9341_TOUCH_MAX_RAW_Y) raw_y = ILI9341_TOUCH_MAX_RAW_Y;

    // Uncomment this line to calibrate touchscreen:
    // UART_Printf("raw_x = %d, raw_y = %d\r\n", x, y);

    *x = (raw_x - ILI9341_TOUCH_MIN_RAW_X) * ILI9341_TOUCH_SCALE_X / (ILI9341_TOUCH_MAX_RAW_X - ILI9341_TOUCH_MIN_RAW_X);
    *y = (raw_y - ILI9341_TOUCH_MIN_RAW_Y) * ILI9341_TOUCH_SCALE_Y / (ILI9341_TOUCH_MAX_RAW_Y - ILI9341_TOUCH_MIN_RAW_Y);

    return true;
}













//
//#define TOUCH_SCALE_X 320
//#define TOUCH_SCALE_Y 240
//
//#define TOUCH_MIN_RAW_X 1500
//#define TOUCH_MAX_RAW_X 30000
//#define TOUCH_MIN_RAW_Y 2500
//#define TOUCH_MAX_RAW_Y 30000
//
//static const uint8_t cmd_read_x = 0x90;
//static const uint8_t cmd_read_y = 0xD0;
//static const uint8_t zeroes_tx[] = {0x00, 0x00};
//
//// калибровка
///*#include "string.h"
//#include "stdio.h"
//extern UART_HandleTypeDef huart1;*/
//
//
//uint8_t ILI9341_TouchGetCoordinates(uint16_t *x, uint16_t *y)
//{
///*	if(HAL_GPIO_ReadPin(TOUCH_IRQ_GPIO_Port, TOUCH_IRQ_Pin) != GPIO_PIN_RESET) return 0;
//
//	DISP_CS_UNSELECT;
//    TOUCH_CS_SELECT;
//*/
//    uint32_t avg_x = 0;
//    uint32_t avg_y = 0;
//
//	HAL_SPI_Transmit(TOUCH_SPI_PTR, (uint8_t*)&cmd_read_y, 1, 1000);
//
//	uint8_t y_raw[2] = {0,};
//	HAL_SPI_TransmitReceive(TOUCH_SPI_PTR, (uint8_t*)zeroes_tx, y_raw, 2, 1000);
//
//	HAL_SPI_Transmit(TOUCH_SPI_PTR, (uint8_t*)&cmd_read_x, 1, 1000);
//
//	uint8_t x_raw[2] = {0,};
//	HAL_SPI_TransmitReceive(TOUCH_SPI_PTR, (uint8_t*)zeroes_tx, x_raw, 2, 1000);
//
//	avg_x = (((uint16_t)x_raw[0]) << 8) | ((uint16_t)x_raw[1]);
//	avg_y = (((uint16_t)y_raw[0]) << 8) | ((uint16_t)y_raw[1]);
//
//	// калибровка
//	/*char buf[64] = {0,};
//	snprintf(buf, 64, "ADC_X = %lu, ADC_Y = %lu\n", avg_x, avg_y);
//	HAL_UART_Transmit(&huart1, (uint8_t*)buf, strlen(buf), 100);*/
///*
//    TOUCH_CS_UNSELECT;
//    __HAL_SPI_ENABLE(DISP_SPI_PTR);
//    DISP_CS_SELECT;
//*/
//
//    if(avg_x < TOUCH_MIN_RAW_X) avg_x = TOUCH_MIN_RAW_X;
//    if(avg_x > TOUCH_MAX_RAW_X) avg_x = TOUCH_MAX_RAW_X;
//
//    if(avg_y < TOUCH_MIN_RAW_X) avg_y = TOUCH_MIN_RAW_Y;
//    if(avg_y > TOUCH_MAX_RAW_Y) avg_y = TOUCH_MAX_RAW_Y;
//
//    *x = (avg_x - TOUCH_MIN_RAW_X) * TOUCH_SCALE_X / (TOUCH_MAX_RAW_X - TOUCH_MIN_RAW_X);
//    *y = (avg_y - TOUCH_MIN_RAW_Y) * TOUCH_SCALE_Y / (TOUCH_MAX_RAW_Y - TOUCH_MIN_RAW_Y);
//
//    return 1;
//}
//




