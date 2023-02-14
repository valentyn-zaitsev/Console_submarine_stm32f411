#include "lv_impl.h"

lv_obj_t * l_xy_label;
lv_obj_t * send_data_label;
lv_obj_t * xy_label;
lv_obj_t * radio_label;
lv_obj_t * lx_bar;
lv_obj_t * ly_bar;
lv_obj_t * rx_bar;
lv_obj_t * ry_bar;
lv_obj_t * battery_label;

void lv_menu(void)
{
    /*Create a menu object*/
    lv_obj_t * menu = lv_menu_create(lv_scr_act());
    lv_obj_set_size(menu, lv_disp_get_hor_res(NULL), lv_disp_get_ver_res(NULL));
    lv_obj_center(menu);

    lv_obj_t * cont;

    /*Create a sub page*/
/*    lv_obj_t * sub_page = lv_menu_page_create(menu, NULL);

    cont = lv_menu_cont_create(sub_page);
    label = lv_label_create(cont);
    lv_label_set_text(label, "Robot will investigate underwater area");
*/
    /*Create a main page*/
    lv_obj_t * main_page = lv_menu_page_create(menu, NULL);
/*
    cont = lv_menu_cont_create(main_page);
    label = lv_label_create(cont);
    lv_label_set_text(label, "Temperature: 22");
*/
    cont = lv_menu_cont_create(main_page);
    battery_label = lv_label_create(cont);
    lv_obj_set_align(battery_label, LV_ALIGN_TOP_LEFT);
    lv_label_set_text(battery_label, "0mv");

    cont = lv_menu_cont_create(main_page);
    l_xy_label = lv_label_create(cont);
    lv_label_set_text(l_xy_label, "LX: 0 | LY: 0 | RX: 0 | RY: 0");

    cont = lv_menu_cont_create(main_page);
    radio_label = lv_label_create(cont);
    lv_label_set_text(radio_label, "Radio data");

    cont = lv_menu_cont_create(main_page);
    send_data_label = lv_label_create(cont);
    lv_label_set_text(send_data_label, "");

    cont = lv_menu_cont_create(main_page);
    xy_label = lv_label_create(cont);
    lv_label_set_text(xy_label, "");
/*
    cont = lv_menu_cont_create(main_page);
    label = lv_label_create(cont);
    lv_label_set_text(label, "Setting*");
    lv_menu_set_load_page_event(menu, cont, sub_page);
*/

    lv_menu_set_page(menu, main_page);


    lv_bar();




    //    arc = lv_arc_create(lv_scr_act());
    //	lv_arc_set_rotation(arc, 270);
    //	lv_arc_set_bg_angles(arc, 0, 360);
    //	lv_obj_remove_style(arc, NULL, LV_PART_KNOB);   /*Be sure the knob is not displayed*/
    //	lv_obj_clear_flag(arc, LV_OBJ_FLAG_CLICKABLE);  /*To not allow adjusting by click*/
    //	lv_obj_center(arc);
    //
    //	lv_anim_t a;
    //	lv_anim_init(&a);
    //	lv_anim_set_var(&a, arc);
    //	//lv_anim_set_exec_cb(&a, set_angle);
    //	lv_anim_set_time(&a, 1000);
    //	lv_anim_set_repeat_count(&a, LV_ANIM_REPEAT_INFINITE);    /*Just for the demo*/
    //	lv_anim_set_repeat_delay(&a, 500);
    //	lv_anim_set_values(&a, 0, 100);
    //	lv_anim_start(&a);
    //
    //	label1 = lv_label_create(lv_scr_act());
    //	lv_label_set_long_mode(label1, LV_LABEL_LONG_WRAP);     /*Break the long lines*/
    //	lv_label_set_recolor(label1, true);                      /*Enable re-coloring by commands in the text*/
    //	lv_label_set_text(label1, "00m");
    //
    //	lv_obj_set_width(label1, 150);  /*Set smaller width to make the lines wrap*/
    //	lv_obj_set_style_text_align(label1, LV_TEXT_ALIGN_CENTER, 0);
    //	lv_obj_align(label1, LV_ALIGN_CENTER, 0, 0);

    /*
        lv_obj_t * btn = lv_btn_create(lv_scr_act());
    	lv_obj_set_pos(btn, 10, 10);
    	lv_obj_set_size(btn, 120, 50);
    	lv_obj_add_event_cb(btn, btn_event_cb, LV_EVENT_ALL, NULL);

    	lv_obj_t * label = lv_label_create(btn);
    	lv_label_set_text(label, "Click me");
    	lv_obj_center(label);
    */
}

void set_bar(void * bar, int32_t value)
{
    lv_bar_set_value(bar, value, LV_ANIM_ON);
}


void lv_bar(void)
{
    static lv_style_t style_indic;

    lv_style_init(&style_indic);
    lv_style_set_bg_opa(&style_indic, LV_OPA_COVER);
    lv_style_set_bg_color(&style_indic, lv_palette_main(LV_PALETTE_RED));
    lv_style_set_bg_grad_color(&style_indic, lv_palette_main(LV_PALETTE_BLUE));
    lv_style_set_bg_grad_dir(&style_indic, LV_GRAD_DIR_VER);

    ry_bar = lv_bar_create(lv_scr_act());
    lv_obj_add_style(ry_bar, &style_indic, LV_PART_INDICATOR);
    lv_obj_set_size(ry_bar, 10, 100);
    lv_obj_set_align(ry_bar, LV_ALIGN_TOP_RIGHT);
    lv_bar_set_range(ry_bar, 0, 100);

    rx_bar = lv_bar_create(lv_scr_act());
	lv_obj_add_style(rx_bar, &style_indic, LV_PART_INDICATOR);
	lv_obj_set_size(rx_bar, 10, 100);
	lv_obj_align_to(rx_bar, ry_bar, LV_ALIGN_OUT_LEFT_MID, -5, 0);
	lv_bar_set_range(rx_bar, 0, 100);

	ly_bar = lv_bar_create(lv_scr_act());
	lv_obj_add_style(ly_bar, &style_indic, LV_PART_INDICATOR);
	lv_obj_set_size(ly_bar, 10, 100);
	lv_obj_align_to(ly_bar, rx_bar, LV_ALIGN_OUT_LEFT_MID, -5, 0);
	lv_bar_set_range(ly_bar, 0, 100);

	lx_bar = lv_bar_create(lv_scr_act());
	lv_obj_add_style(lx_bar, &style_indic, LV_PART_INDICATOR);
	lv_obj_set_size(lx_bar, 10, 100);
	lv_obj_align_to(lx_bar, ly_bar, LV_ALIGN_OUT_LEFT_MID, -5, 0);
	lv_bar_set_range(lx_bar, 0, 100);

}

