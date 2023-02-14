/*
 * lv_impl.h
 *
 *  Created on: Jul 3, 2022
 *      Author: tantrum
 */

#ifndef INC_LV_IMPL_H_
#define INC_LV_IMPL_H_

#include "../lv_conf.h"
#include "../lvgl/lvgl.h"
//#include "../../lvgl/examples/lv_examples.h"

extern lv_obj_t * l_xy_label;
extern lv_obj_t * send_data_label;
extern lv_obj_t * radio_label;
extern lv_obj_t * lx_bar;
extern lv_obj_t * ly_bar;
extern lv_obj_t * rx_bar;
extern lv_obj_t * ry_bar;
extern lv_obj_t * battery_label;



void lv_menu(void);

void lv_bar(void);
void set_bar(void * bar, int32_t value);








#endif /* INC_LV_IMPL_H_ */
