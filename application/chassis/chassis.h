#ifndef CHASSIS_H
#define CHASSIS_H

#include "rm_referee.h"
#include "referee_task.h"
/**
 * @brief 底盘应用初始化,请在开启rtos之前调用(目前会被RobotInit()调用)
 * 
 */
void ChassisInit();

void SendJudgeData(referee_info_t* referee_Data);
/**
 * @brief 底盘应用任务,放入实时系统以一定频率运行
 * 
 */
void ChassisTask();

#endif // CHASSIS_H