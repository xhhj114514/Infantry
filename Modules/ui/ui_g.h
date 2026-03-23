//
// Created by RM UI Designer
// Dynamic Edition
//

#ifndef UI_g_H
#define UI_g_H

#include "ui_interface.h"
#include "robot_def.h"
#include "super_cap.h"
extern ui_interface_figure_t ui_g_now_figures[13];
extern uint8_t ui_g_dirty_figure[13];
extern ui_interface_string_t ui_g_now_strings[8];
extern uint8_t ui_g_dirty_string[8];

extern uint8_t ui_g_max_send_count[21];

#define ui_g_Ungroup_blood1 ((ui_interface_rect_t*)&(ui_g_now_figures[0]))
#define ui_g_Ungroup_max_blood ((ui_interface_number_t*)&(ui_g_now_figures[1]))
#define ui_g_Ungroup_blood ((ui_interface_number_t*)&(ui_g_now_figures[2]))
#define ui_g_Ungroup_blood_line ((ui_interface_line_t*)&(ui_g_now_figures[3]))
#define ui_g_Ungroup_cap ((ui_interface_rect_t*)&(ui_g_now_figures[4]))
#define ui_g_Ungroup_cap1 ((ui_interface_rect_t*)&(ui_g_now_figures[5]))
#define ui_g_Ungroup_aim_rect ((ui_interface_rect_t*)&(ui_g_now_figures[6]))

#define ui_g_Ungroup_Line1 ((ui_interface_line_t*)&(ui_g_now_figures[7]))
#define ui_g_Ungroup_Line2 ((ui_interface_line_t*)&(ui_g_now_figures[8]))
#define ui_g_Ungroup_Line3 ((ui_interface_line_t*)&(ui_g_now_figures[9]))
#define ui_g_Ungroup_Line4 ((ui_interface_line_t*)&(ui_g_now_figures[10]))
#define ui_g_Ungroup_Line5 ((ui_interface_line_t*)&(ui_g_now_figures[11]))
#define ui_g_Ungroup_Line0 ((ui_interface_line_t*)&(ui_g_now_figures[12]))

#define ui_g_Ungroup_loader_mode (&(ui_g_now_strings[0]))
#define ui_g_Ungroup_Loader_normal (&(ui_g_now_strings[1]))
#define ui_g_Ungroup_chassis_mode (&(ui_g_now_strings[2]))
#define ui_g_Ungroup_chassis_follow (&(ui_g_now_strings[3]))
#define ui_g_Ungroup_Aim (&(ui_g_now_strings[4]))
#define ui_g_Ungroup_AIM_MODE (&(ui_g_now_strings[5]))
#define ui_g_Ungroup_cap_name (&(ui_g_now_strings[6]))
#define ui_g_Ungroup_hp (&(ui_g_now_strings[7]))

#define ui_g_Ungroup_blood1_max_send_count (ui_g_max_send_count[0])
#define ui_g_Ungroup_max_blood_max_send_count (ui_g_max_send_count[1])
#define ui_g_Ungroup_blood_max_send_count (ui_g_max_send_count[2])
#define ui_g_Ungroup_blood_line_max_send_count (ui_g_max_send_count[3])
#define ui_g_Ungroup_cap_max_send_count (ui_g_max_send_count[4])
#define ui_g_Ungroup_cap1_max_send_count (ui_g_max_send_count[5])
#define ui_g_Ungroup_aim_rect_max_send_count (ui_g_max_send_count[6])

#define ui_g_Ungroup_Line1_max_send_count (ui_g_max_send_count[7])
#define ui_g_Ungroup_Line2_max_send_count (ui_g_max_send_count[8])
#define ui_g_Ungroup_Line3_max_send_count (ui_g_max_send_count[9])
#define ui_g_Ungroup_Line4_max_send_count (ui_g_max_send_count[10])
#define ui_g_Ungroup_Line5_max_send_count (ui_g_max_send_count[11])
#define ui_g_Ungroup_Line0_max_send_count (ui_g_max_send_count[12])

#define ui_g_Ungroup_loader_mode_max_send_count (ui_g_max_send_count[13])
#define ui_g_Ungroup_Loader_normal_max_send_count (ui_g_max_send_count[14])

#define ui_g_Ungroup_chassis_mode_max_send_count (ui_g_max_send_count[15])
#define ui_g_Ungroup_chassis_follow_max_send_count (ui_g_max_send_count[16])
#define ui_g_Ungroup_Aim_max_send_count (ui_g_max_send_count[17])
#define ui_g_Ungroup_AIM_MODE_max_send_count (ui_g_max_send_count[18])

#ifdef MANUAL_DIRTY
#define ui_g_Ungroup_blood1_dirty (ui_g_dirty_figure[0])
#define ui_g_Ungroup_cap_dirty (ui_g_dirty_figure[1])
#define ui_g_Ungroup_cap1_dirty (ui_g_dirty_figure[2])
#define ui_g_Ungroup_max_blood_dirty (ui_g_dirty_figure[3])
#define ui_g_Ungroup_blood_dirty (ui_g_dirty_figure[4])
#define ui_g_Ungroup_Line1_dirty (ui_g_dirty_figure[5])
#define ui_g_Ungroup_blood_line_dirty (ui_g_dirty_figure[6])
#define ui_g_Ungroup_aim_rect_dirty (ui_g_dirty_figure[7])
#define ui_g_Ungroup_Line2_dirty (ui_g_dirty_figure[8])
#define ui_g_Ungroup_Line3_dirty (ui_g_dirty_figure[9])
#define ui_g_Ungroup_Line4_dirty (ui_g_dirty_figure[10])
#define ui_g_Ungroup_Line5_dirty (ui_g_dirty_figure[11])
#define ui_g_Ungroup_Line0_dirty (ui_g_dirty_figure[12])

#define ui_g_Ungroup_loader_mode_dirty (ui_g_dirty_string[0])
#define ui_g_Ungroup_chassis_mode_dirty (ui_g_dirty_string[1])
#define ui_g_Ungroup_chassis_follow_dirty (ui_g_dirty_string[2])
#define ui_g_Ungroup_Aim_dirty (ui_g_dirty_string[3])
#define ui_g_Ungroup_AIM_MODE_dirty (ui_g_dirty_string[4])
#define ui_g_Ungroup_Loader_normal_dirty (ui_g_dirty_string[5])
#endif

void ui_init_g();
void ui_update_g();
void ui_update_hp(uint16_t current_hp, uint16_t max_hp) ;
void ui_update_loader_mode(loader_mode_e loader_mode) ;
// void ui_update_aim_mode(Aim_Mode_e aim_mode) ;
void ui_update_chassis_mode(chassis_mode_e chassis_mode) ;
void ui_update_cap_msg(SuperCap_Msg_s cap_msg); 

#endif // UI_g_H
