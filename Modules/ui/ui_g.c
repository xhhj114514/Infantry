//
// Created by RM UI Designer
// Dynamic Edition
//

#include "robot_def.h"
#include "string.h"
#include "ui_interface.h"
#include "ui_g.h"

#define TOTAL_FIGURE 13
#define TOTAL_STRING 8

ui_interface_figure_t ui_g_now_figures[TOTAL_FIGURE];
uint8_t ui_g_dirty_figure[TOTAL_FIGURE];
ui_interface_string_t ui_g_now_strings[TOTAL_STRING];
uint8_t ui_g_dirty_string[TOTAL_STRING];

uint8_t ui_g_max_send_count[TOTAL_FIGURE + TOTAL_STRING] = {
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
    1,
};

#ifndef MANUAL_DIRTY
ui_interface_figure_t ui_g_last_figures[TOTAL_FIGURE];
ui_interface_string_t ui_g_last_strings[TOTAL_STRING];
#endif

#define SCAN_AND_SEND() ui_scan_and_send(ui_g_now_figures, ui_g_dirty_figure, ui_g_now_strings, ui_g_dirty_string, TOTAL_FIGURE, TOTAL_STRING)

void ui_init_g() {
    ui_g_Ungroup_blood1->figure_type = 1;
    ui_g_Ungroup_blood1->operate_type = 1;
    ui_g_Ungroup_blood1->layer = 0;
    ui_g_Ungroup_blood1->color = 0;
    ui_g_Ungroup_blood1->start_x = 634;
    ui_g_Ungroup_blood1->start_y = 850;
    ui_g_Ungroup_blood1->width = 1;
    ui_g_Ungroup_blood1->end_x = 1256;
    ui_g_Ungroup_blood1->end_y = 890;

    ui_g_Ungroup_blood_line->figure_type = 0;
    ui_g_Ungroup_blood_line->operate_type = 1;
    ui_g_Ungroup_blood_line->layer = 0;
    ui_g_Ungroup_blood_line->color = 0;
    ui_g_Ungroup_blood_line->start_x = 633;
    ui_g_Ungroup_blood_line->start_y = 870;
    ui_g_Ungroup_blood_line->width = 40;
    ui_g_Ungroup_blood_line->end_x = 1262;
    ui_g_Ungroup_blood_line->end_y = 870;

    ui_g_Ungroup_cap->figure_type = 1;
    ui_g_Ungroup_cap->operate_type = 1;
    ui_g_Ungroup_cap->layer = 0;
    ui_g_Ungroup_cap->color = 1;
    ui_g_Ungroup_cap->start_x = 633;
    ui_g_Ungroup_cap->start_y = 775;
    ui_g_Ungroup_cap->width = 1;
    ui_g_Ungroup_cap->end_x = 1257;
    ui_g_Ungroup_cap->end_y = 815;

    ui_g_Ungroup_cap1->figure_type = 1;
    ui_g_Ungroup_cap1->operate_type = 1;
    ui_g_Ungroup_cap1->layer = 0;
    ui_g_Ungroup_cap1->color = 1;
    ui_g_Ungroup_cap1->start_x = 632;
    ui_g_Ungroup_cap1->start_y = 795;
    ui_g_Ungroup_cap1->width = 40;
    ui_g_Ungroup_cap1->end_x = 1262;
    ui_g_Ungroup_cap1->end_y = 795;

    ui_g_Ungroup_max_blood->figure_type = 6;
    ui_g_Ungroup_max_blood->operate_type = 1;
    ui_g_Ungroup_max_blood->layer = 0;
    ui_g_Ungroup_max_blood->color = 0;
    ui_g_Ungroup_max_blood->start_x = 1428;
    ui_g_Ungroup_max_blood->start_y = 869;
    ui_g_Ungroup_max_blood->width = 2;
    ui_g_Ungroup_max_blood->font_size = 20;
    ui_g_Ungroup_max_blood->number = 100;

    ui_g_Ungroup_blood->figure_type = 6;
    ui_g_Ungroup_blood->operate_type = 1;
    ui_g_Ungroup_blood->layer = 0;
    ui_g_Ungroup_blood->color = 0;
    ui_g_Ungroup_blood->start_x = 1323;
    ui_g_Ungroup_blood->start_y = 869;
    ui_g_Ungroup_blood->width = 2;
    ui_g_Ungroup_blood->font_size = 20;
    ui_g_Ungroup_blood->number = 100;

    ui_g_Ungroup_Line1->figure_type = 0;
    ui_g_Ungroup_Line1->operate_type = 1;
    ui_g_Ungroup_Line1->layer = 0;
    ui_g_Ungroup_Line1->color = 1;
    ui_g_Ungroup_Line1->start_x = 961;
    ui_g_Ungroup_Line1->start_y = 443;
    ui_g_Ungroup_Line1->width = 1;
    ui_g_Ungroup_Line1->end_x = 1000;
    ui_g_Ungroup_Line1->end_y = 443;



    ui_g_Ungroup_aim_rect->figure_type = 1;
    ui_g_Ungroup_aim_rect->operate_type = 1;
    ui_g_Ungroup_aim_rect->layer = 0;
    ui_g_Ungroup_aim_rect->color = 8;
    ui_g_Ungroup_aim_rect->start_x = 500;
    ui_g_Ungroup_aim_rect->start_y = 238;
    ui_g_Ungroup_aim_rect->width = 1;
    ui_g_Ungroup_aim_rect->end_x = 1444;
    ui_g_Ungroup_aim_rect->end_y = 742;

    ui_g_Ungroup_Line2->figure_type = 0;
    ui_g_Ungroup_Line2->operate_type = 1;
    ui_g_Ungroup_Line2->layer = 0;
    ui_g_Ungroup_Line2->color = 1;
    ui_g_Ungroup_Line2->start_x = 1026;
    ui_g_Ungroup_Line2->start_y = 373;
    ui_g_Ungroup_Line2->width = 1;
    ui_g_Ungroup_Line2->end_x = 955;
    ui_g_Ungroup_Line2->end_y = 373;

    ui_g_Ungroup_Line3->figure_type = 0;
    ui_g_Ungroup_Line3->operate_type = 1;
    ui_g_Ungroup_Line3->layer = 0;
    ui_g_Ungroup_Line3->color = 1;
    ui_g_Ungroup_Line3->start_x = 1027;
    ui_g_Ungroup_Line3->start_y = 323;
    ui_g_Ungroup_Line3->width = 1;
    ui_g_Ungroup_Line3->end_x = 951;
    ui_g_Ungroup_Line3->end_y = 323;

    ui_g_Ungroup_Line4->figure_type = 0;
    ui_g_Ungroup_Line4->operate_type = 1;
    ui_g_Ungroup_Line4->layer = 0;
    ui_g_Ungroup_Line4->color = 1;
    ui_g_Ungroup_Line4->start_x = 955;
    ui_g_Ungroup_Line4->start_y = 412;
    ui_g_Ungroup_Line4->width = 1;
    ui_g_Ungroup_Line4->end_x = 994;
    ui_g_Ungroup_Line4->end_y = 412;

    ui_g_Ungroup_Line5->figure_type = 0;
    ui_g_Ungroup_Line5->operate_type = 1;
    ui_g_Ungroup_Line5->layer = 0;
    ui_g_Ungroup_Line5->color = 1;
    ui_g_Ungroup_Line5->start_x = 1026;
    ui_g_Ungroup_Line5->start_y = 350;
    ui_g_Ungroup_Line5->width = 1;
    ui_g_Ungroup_Line5->end_x = 955;
    ui_g_Ungroup_Line5->end_y = 350;

    ui_g_Ungroup_Line0->figure_type = 0;
    ui_g_Ungroup_Line0->operate_type = 1;
    ui_g_Ungroup_Line0->layer = 0;
    ui_g_Ungroup_Line0->color = 1;
    ui_g_Ungroup_Line0->start_x = 950;
    ui_g_Ungroup_Line0->start_y = 264;
    ui_g_Ungroup_Line0->width = 3;
    ui_g_Ungroup_Line0->end_x = 950;
    ui_g_Ungroup_Line0->end_y = 445;

    ui_g_Ungroup_loader_mode->figure_type = 7;
    ui_g_Ungroup_loader_mode->operate_type = 1;
    ui_g_Ungroup_loader_mode->layer = 0;
    ui_g_Ungroup_loader_mode->color = 7;
    ui_g_Ungroup_loader_mode->start_x = 86;
    ui_g_Ungroup_loader_mode->start_y = 760;
    ui_g_Ungroup_loader_mode->width = 2;
    ui_g_Ungroup_loader_mode->font_size = 25;
    ui_g_Ungroup_loader_mode->str_length = 7;
    strcpy(ui_g_Ungroup_loader_mode->string, "loader:");

    ui_g_Ungroup_Loader_normal->figure_type = 7;
    ui_g_Ungroup_Loader_normal->operate_type = 1;
    ui_g_Ungroup_Loader_normal->layer = 0;
    ui_g_Ungroup_Loader_normal->color = 1;
    ui_g_Ungroup_Loader_normal->start_x = 296;
    ui_g_Ungroup_Loader_normal->start_y = 760;
    ui_g_Ungroup_Loader_normal->width = 2;
    ui_g_Ungroup_Loader_normal->font_size = 25;
    ui_g_Ungroup_Loader_normal->str_length = 6;
    strcpy(ui_g_Ungroup_Loader_normal->string, "normal");

    ui_g_Ungroup_chassis_mode->figure_type = 7;
    ui_g_Ungroup_chassis_mode->operate_type = 1;
    ui_g_Ungroup_chassis_mode->layer = 0;
    ui_g_Ungroup_chassis_mode->color = 7;
    ui_g_Ungroup_chassis_mode->start_x = 78;
    ui_g_Ungroup_chassis_mode->start_y = 670;
    ui_g_Ungroup_chassis_mode->width = 2;
    ui_g_Ungroup_chassis_mode->font_size = 25;
    ui_g_Ungroup_chassis_mode->str_length = 8;
    strcpy(ui_g_Ungroup_chassis_mode->string, "CHASSIS:");

    ui_g_Ungroup_chassis_follow->figure_type = 7;
    ui_g_Ungroup_chassis_follow->operate_type = 1;
    ui_g_Ungroup_chassis_follow->layer = 0;
    ui_g_Ungroup_chassis_follow->color = 1;
    ui_g_Ungroup_chassis_follow->start_x = 290;
    ui_g_Ungroup_chassis_follow->start_y = 670;
    ui_g_Ungroup_chassis_follow->width = 2;
    ui_g_Ungroup_chassis_follow->font_size = 25;
    ui_g_Ungroup_chassis_follow->str_length = 6;
    strcpy(ui_g_Ungroup_chassis_follow->string, "follow");

    ui_g_Ungroup_Aim->figure_type = 7;
    ui_g_Ungroup_Aim->operate_type = 1;
    ui_g_Ungroup_Aim->layer = 0;
    ui_g_Ungroup_Aim->color = 7;
    ui_g_Ungroup_Aim->start_x = 84;
    ui_g_Ungroup_Aim->start_y = 580;
    ui_g_Ungroup_Aim->width = 2;
    ui_g_Ungroup_Aim->font_size = 25;
    ui_g_Ungroup_Aim->str_length = 4;
    strcpy(ui_g_Ungroup_Aim->string, "AIM:");

    ui_g_Ungroup_AIM_MODE->figure_type = 7;
    ui_g_Ungroup_AIM_MODE->operate_type = 1;
    ui_g_Ungroup_AIM_MODE->layer = 0;
    ui_g_Ungroup_AIM_MODE->color = 0;
    ui_g_Ungroup_AIM_MODE->start_x = 280;
    ui_g_Ungroup_AIM_MODE->start_y = 580;
    ui_g_Ungroup_AIM_MODE->width = 2;
    ui_g_Ungroup_AIM_MODE->font_size = 25;
    ui_g_Ungroup_AIM_MODE->str_length = 3;
    strcpy(ui_g_Ungroup_AIM_MODE->string, "OFF");


    ui_g_Ungroup_cap_name->figure_type = 7;
    ui_g_Ungroup_cap_name->operate_type = 1;
    ui_g_Ungroup_cap_name->layer = 0;
    ui_g_Ungroup_cap_name->color = 1;
    ui_g_Ungroup_cap_name->start_x = 510;
    ui_g_Ungroup_cap_name->start_y = 803;
    ui_g_Ungroup_cap_name->width = 2;
    ui_g_Ungroup_cap_name->font_size = 23;
    ui_g_Ungroup_cap_name->str_length = 3;
    strcpy(ui_g_Ungroup_cap_name->string, "cap");

    ui_g_Ungroup_hp->figure_type = 7;
    ui_g_Ungroup_hp->operate_type = 1;
    ui_g_Ungroup_hp->layer = 0;
    ui_g_Ungroup_hp->color = 0;
    ui_g_Ungroup_hp->start_x = 510;
    ui_g_Ungroup_hp->start_y = 881;
    ui_g_Ungroup_hp->width = 2;
    ui_g_Ungroup_hp->font_size = 25;
    ui_g_Ungroup_hp->str_length = 2;
    strcpy(ui_g_Ungroup_hp->string, "HP");

    uint32_t idx = 0;
    for (int i = 0; i < TOTAL_FIGURE; i++) {
        ui_g_now_figures[i].figure_name[2] = idx & 0xFF;
        ui_g_now_figures[i].figure_name[1] = (idx >> 8) & 0xFF;
        ui_g_now_figures[i].figure_name[0] = (idx >> 16) & 0xFF;
        ui_g_now_figures[i].operate_type = 1;
#ifndef MANUAL_DIRTY
        ui_g_last_figures[i] = ui_g_now_figures[i];
#endif
        ui_g_dirty_figure[i] = 1;
        idx++;
    }
    for (int i = 0; i < TOTAL_STRING; i++) {
        ui_g_now_strings[i].figure_name[2] = idx & 0xFF;
        ui_g_now_strings[i].figure_name[1] = (idx >> 8) & 0xFF;
        ui_g_now_strings[i].figure_name[0] = (idx >> 16) & 0xFF;
        ui_g_now_strings[i].operate_type = 1;
#ifndef MANUAL_DIRTY
        ui_g_last_strings[i] = ui_g_now_strings[i];
#endif
        ui_g_dirty_string[i] = 1;
        idx++;
    }

    SCAN_AND_SEND();

    for (int i = 0; i < TOTAL_FIGURE; i++) {
        ui_g_now_figures[i].operate_type = 2;
    }
    for (int i = 0; i < TOTAL_STRING; i++) {
        ui_g_now_strings[i].operate_type = 2;
    }
}

void ui_update_hp(uint16_t current_hp, uint16_t max_hp) 
{
    // 更新现血量数字
    ui_g_Ungroup_blood->number = current_hp;
    
    // 更新总血量数字
    ui_g_Ungroup_max_blood->number = max_hp;
    
    if (max_hp > 0) 
    {
        float hp_ratio = (float)current_hp / max_hp;
        int32_t blood_length = (int32_t)(622 * hp_ratio);
        if (blood_length < 2) 
        {
            blood_length = 2;
        }
        ui_g_Ungroup_blood_line->end_x = ui_g_Ungroup_blood_line->start_x + blood_length;
    }
    else 
    {
        ui_g_Ungroup_blood_line->end_x = ui_g_Ungroup_blood_line->start_x;  // 没血了
    }
}

void ui_update_loader_mode(loader_mode_e loader_mode) 
{
    switch (loader_mode) {
        case LOAD_STOP:
            ui_g_Ungroup_Loader_normal->color = 0;
            ui_g_Ungroup_Loader_normal->str_length = 7;
            strcpy(ui_g_Ungroup_Loader_normal->string, "off    ");
            break;
        case LOAD_BURSTFIRE:
            ui_g_Ungroup_Loader_normal->color = 2;
            ui_g_Ungroup_Loader_normal->str_length = 7;
            strcpy(ui_g_Ungroup_Loader_normal->string, "normal ");
            break;
        case LOAD_REVERSE:
            ui_g_Ungroup_Loader_normal->color = 1;
            ui_g_Ungroup_Loader_normal->str_length = 7;
            strcpy(ui_g_Ungroup_Loader_normal->string, "reverse");
            break;
    }
}

void ui_update_aim_mode(Aim_Mode_e aim_mode) 
{
    switch (aim_mode) {
        case NOTARGET:
            ui_g_Ungroup_AIM_MODE->color = 0;
            ui_g_Ungroup_AIM_MODE->str_length = 7;
            strcpy(ui_g_Ungroup_AIM_MODE->string, "NONE  ");
            break;
        case TARGET_FOUND:
            ui_g_Ungroup_AIM_MODE->color = 2;
            ui_g_Ungroup_AIM_MODE->str_length = 7;
            strcpy(ui_g_Ungroup_AIM_MODE->string, "FOUND ");
            break;
        case TARGET_LOCKED:
            ui_g_Ungroup_AIM_MODE->color = 1;
            ui_g_Ungroup_AIM_MODE->str_length = 7;
            strcpy(ui_g_Ungroup_AIM_MODE->string, "LOCKED");
            break;
    }
}

void ui_update_chassis_mode(chassis_mode_e chassis_mode) 
{
    switch (chassis_mode) {
        case CHASSIS_FOLLOW_GIMBAL_YAW:
            ui_g_Ungroup_chassis_follow->color = 2;
            ui_g_Ungroup_chassis_follow->str_length = 6;
            strcpy(ui_g_Ungroup_chassis_follow->string, "follow");
            break;
        case CHASSIS_ROTATE:
            ui_g_Ungroup_chassis_follow->color = 1;
            ui_g_Ungroup_chassis_follow->str_length = 6;
            strcpy(ui_g_Ungroup_chassis_follow->string, "rotate");
            break;
    }
}

void ui_update_cap_msg(SuperCap_Msg_s cap_msg) 
{
    // 计算电压百分比，11为0%，24V为100%
    float voltage = (float)cap_msg.vol; // 假设单位是mV，转换为V
    float min_voltage = 8.0f; // 最小电压12V
    float max_voltage = 24.0f; // 最大电压24V
    
    // 限制电压范围
    if (voltage < min_voltage) 
    {
        voltage = min_voltage;
    } else if (voltage > max_voltage) 
    {
        voltage = max_voltage;
    }
    
    // 计算进度百分比
    float percentage = (voltage - min_voltage) / (max_voltage - min_voltage);
    
    // 获取初始坐标
    int16_t start_x = 632; // 进度条起始X坐标
    int16_t end_x = 1262;  // 进度条最大结束X坐标
    int16_t base_length = end_x - start_x; // 进度条总长度
    
    // 计算当前进度条应达到的X坐标
    int16_t current_end_x = start_x + (int16_t)(base_length * percentage);
    
    // 更新进度条
    ui_g_Ungroup_cap1->end_x = current_end_x;
}

void ui_update_g() {
#ifndef MANUAL_DIRTY
    for (int i = 0; i < TOTAL_FIGURE; i++) {
        if (memcmp(&ui_g_now_figures[i], &ui_g_last_figures[i], sizeof(ui_g_now_figures[i])) != 0) {
            ui_g_dirty_figure[i] = ui_g_max_send_count[i];
            ui_g_last_figures[i] = ui_g_now_figures[i];
        }
    }
    for (int i = 0; i < TOTAL_STRING; i++) {
        if (memcmp(&ui_g_now_strings[i], &ui_g_last_strings[i], sizeof(ui_g_now_strings[i])) != 0) {
            ui_g_dirty_string[i] = ui_g_max_send_count[TOTAL_FIGURE + i];
            ui_g_last_strings[i] = ui_g_now_strings[i];
        }
    }
#endif
    SCAN_AND_SEND();
}
