#include "chassis.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "super_cap.h"
#include "message_center.h"
#include "general_def.h"
#include "bsp_dwt.h"
#include "arm_math.h"
#include "ui_g.h"

// #include "rm_referee.h"
// #include "referee_task.h"

/* 根据robot_def.h中的macro自动计算的参数 */
#define HALF_WHEEL_BASE (WHEEL_BASE / 2.0f)     // 半轴距
#define HALF_TRACK_WIDTH (TRACK_WIDTH / 2.0f)   // 半轮距
#define PERIMETER_WHEEL (RADIUS_WHEEL * 2 * PI) // 轮子周长

/* 底盘应用包含的模块和信息存储,底盘是单例模式,因此不需要为底盘建立单独的结构体 */
static Publisher_t *chassis_pub;                    // 用于发布底盘的数据
static Subscriber_t *chassis_sub;                   // 用于订阅底盘的控制命令
static Chassis_Ctrl_Cmd_s chassis_cmd_recv;         // 底盘接收到的控制命令
static Chassis_Upload_Data_s chassis_feedback_data; // 底盘回传的反馈数据
static float sin_theta, cos_theta;//麦轮解算用

static float chassis_rotate_buff;

static SuperCapInstance *cap;                                       // 超级电容
static uint16_t power_data;
static DJIMotorInstance *motor_lf, *motor_rf, *motor_lb, *motor_rb; // left right forward back
/* 用于自旋变速策略的时间变量 */
static float t;

/* 私有函数计算的中介变量,设为静态避免参数传递的开销 */
static float chassis_vx, chassis_vy;     // 将云台系的速度投影到底盘
static float vt_lf, vt_rf, vt_lb, vt_rb; // 底盘速度解算后的临时输出,待进行限幅
static Cal_Chassis_Info_t chassis_info;                             // 底盘速度计算信息
static Referee_Interactive_info_t ui_data;              // UI数据，将底盘中的数据传入此结构体的对应变量中，UI会自动检测是否变化，对应显示UI
static referee_info_t* referee_data;                    // 用于获取裁判系统的数据
void ChassisInit()
{
    Motor_Init_Config_s chassis_motor_config = {
        .can_init_config.can_handle = &hcan1,
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 10, // 4.5
                .Ki = 0,  // 0
                .Kd = 0,  // 0
                .IntegralLimit = 3000,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = 12000,
            },
            .current_PID = {
                .Kp = 0.5, // 0.4
                .Ki = 0,   // 0
                .Kd = 0,
                .IntegralLimit = 3000,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = 15000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP | CURRENT_LOOP,
        },
        .motor_type = M3508,
    };

    chassis_motor_config.can_init_config.tx_id = 4;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_lf = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id = 3;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_rf = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id = 1;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_lb = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id = 2;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_rb = DJIMotorInit(&chassis_motor_config);

    SuperCap_Init_Config_s capconfig = {
            .can_config = {
                .can_handle = &hcan1,
                .rx_id = 0x311,
                .tx_id = 0x310,
            },
            .recv_data_len = sizeof(int16_t),
            .send_data_len = sizeof(uint16_t),
        };
     cap=SuperCapInit(&capconfig);
    chassis_sub = SubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_pub = PubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
}
#define LF_CENTER ((HALF_TRACK_WIDTH + CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE - CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define RF_CENTER ((HALF_TRACK_WIDTH - CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE - CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define LB_CENTER ((HALF_TRACK_WIDTH + CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE + CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define RB_CENTER ((HALF_TRACK_WIDTH - CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE + CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)

static void ChassisStateSet()
{
    if (chassis_cmd_recv.chassis_mode == CHASSIS_ZERO_FORCE)
    { // 如果出现重要模块离线或遥控器设置为急停,让电机停止
        DJIMotorStop(motor_lf);
        DJIMotorStop(motor_rf);
        DJIMotorStop(motor_lb);
        DJIMotorStop(motor_rb);
    }
    else
    { // 正常工作
        DJIMotorEnable(motor_lf);
        DJIMotorEnable(motor_rf);
        DJIMotorEnable(motor_lb);
        DJIMotorEnable(motor_rb);
    }
}

static uint16_t TESTPWR;
// 9 lowest
static void SendPowerData()
{
#ifdef SentryMode
    power_data= 100;//chassis_cmd_recv.power_limit;//Sentry--100Watt  Infantry--120Watt
#endif

#ifdef InfantryMode
    power_data = 120;
#endif


}

/**
 * @brief 计算每个底盘电机的输出,正运动学解算
 *        
 */
static void MecanumCalculate()
{   
    chassis_info.cnt = (float32_t)DWT_GetTimeline_s();//用于变速小陀螺
    chassis_info.cos_theta = arm_cos_f32(chassis_cmd_recv.offset_angle * DEGREE_2_RAD);
    chassis_info.sin_theta = arm_sin_f32(chassis_cmd_recv.offset_angle * DEGREE_2_RAD);


    chassis_info.chassis_vx = chassis_cmd_recv.vx * chassis_info.cos_theta - chassis_cmd_recv.vy * chassis_info.sin_theta; 
    chassis_info.chassis_vy = chassis_cmd_recv.vx * chassis_info.sin_theta + chassis_cmd_recv.vy * chassis_info.cos_theta;
    
    chassis_info.wz = (1200+100*(float32_t)sin(chassis_info.cnt))*4.75*chassis_cmd_recv.wz/100;

    chassis_info.vt_lf = chassis_info.chassis_vx - chassis_info.chassis_vy - chassis_cmd_recv.wz ;
    chassis_info.vt_lb = chassis_info.chassis_vx + chassis_info.chassis_vy - chassis_cmd_recv.wz ;
    chassis_info.vt_rb = chassis_info.chassis_vx - chassis_info.chassis_vy + chassis_cmd_recv.wz ;
    chassis_info.vt_rf = chassis_info.chassis_vx + chassis_info.chassis_vy + chassis_cmd_recv.wz ;

}
/**
 * @brief 根据裁判系统和电容剩余容量对输出进行限制并设置电机参考值
 *
 */
static void LimitChassisOutput()
{

    if(cap->cap_msg.vol<=24&&cap->cap_msg.vol>13)
    {
        chassis_feedback_data.power_flag=1; 
    }
    else
    {
        chassis_feedback_data.power_flag=0; 
    }


    // // 完成功率限制后进行电机参考输入设定
    DJIMotorSetRef(motor_lf, chassis_info.vt_lf);
    DJIMotorSetRef(motor_rf, chassis_info.vt_rf);
    DJIMotorSetRef(motor_lb, chassis_info.vt_lb);
    DJIMotorSetRef(motor_rb, chassis_info.vt_rb);
}

/*****************************************SendData********************************************/
/*
measure.speed_aps`是电机未经过减速箱的速度，
所以要除以减速箱系数`REDUCTION_RATIO_WHEEL`，
四个轮子，所以将合速度除四（eg.假设四个轮子速度为10m/s，那么就是先将轮子速度叠加成40m/s，再除4，得到车体的速度是10m/s），
/360（转成弧度制）*轮子周长（将角速度转换成线速度）除1000（将cm/s转换成m/s）。然后再得到底盘坐标系转换到云台坐标系上。（去年漏了这步）
@brief 根据每个轮子的速度反馈,计算底盘的实际运动速度,逆运动解算，并发给巡航底盘实时数据             
 */
static void SendChassisData()
{
    //to  巡航   
    // 步骤1：计算底盘坐标系下的速度（vx, vy）巡航
    chassis_info.vx = (motor_lf->measure.speed_aps +motor_lb->measure.speed_aps - motor_rb->measure.speed_aps - motor_rf->measure.speed_aps) / 4.0f / REDUCTION_RATIO_WHEEL / 360.0f * PERIMETER_WHEEL/1000 ;
    chassis_info.vy = (-motor_lf->measure.speed_aps +motor_lb->measure.speed_aps + motor_rb->measure.speed_aps - motor_rf->measure.speed_aps) / 4.0f / REDUCTION_RATIO_WHEEL / 360.0f * PERIMETER_WHEEL/1000  ;
    // 步骤2：底盘坐标系→云台坐标系转换（关键步骤）
    chassis_feedback_data.real_vx = chassis_info.vx * chassis_info.cos_theta + chassis_info.vy * chassis_info.sin_theta;
    chassis_feedback_data.real_vy = -chassis_info.vx * chassis_info.sin_theta + chassis_info.vy * chassis_info.cos_theta;
    ui_update_cap_msg(cap->cap_msg);
}


void SendJudgeData(referee_info_t* referee_Data)
{
        // 没有装甲板数据时使用裁判系统数据
        // chassis_feedback_data.Occupation = (referee_Data->EventData.event_type >> 21) & 0x03;
        chassis_feedback_data.remain_time = referee_Data->GameState.stage_remain_time;
        chassis_feedback_data.game_progress = referee_Data->GameState.game_progress;
        
        if(referee_data->GameRobotState.robot_id > 7) {
            chassis_feedback_data.enemy_color = COLOR_RED;
            chassis_feedback_data.remain_HP = referee_data->GameRobotHP.self_sentry_HP ;
            chassis_feedback_data.self_hero_HP = referee_data->GameRobotHP.self_Hero_HP;
            chassis_feedback_data.self_infantry_HP = referee_data->GameRobotHP.self_3_fantry_HP;
            // chassis_feedback_data.enemy_hero_HP = referee_data->GameRobotHP.red_1_robot_HP;
            // chassis_feedback_data.enemy_infantry_HP = referee_data->GameRobotHP.red_3_robot_HP;
            // chassis_feedback_data.enemy_sentry_HP = referee_data->GameRobotHP.red_7_robot_HP;
        } 
        else
        {
            chassis_feedback_data.enemy_color = COLOR_BLUE;
            chassis_feedback_data.remain_HP = referee_data->GameRobotHP.self_sentry_HP ;
            chassis_feedback_data.self_hero_HP = referee_data->GameRobotHP.self_Hero_HP;
            chassis_feedback_data.self_infantry_HP = referee_data->GameRobotHP.self_3_fantry_HP;
            // chassis_feedback_data.enemy_hero_HP = referee_data->GameRobotHP.blue_1_robot_HP;
            // chassis_feedback_data.enemy_infantry_HP = referee_data->GameRobotHP.blue_3_robot_HP;
            // chassis_feedback_data.enemy_sentry_HP = referee_data->GameRobotHP.blue_7_robot_HP;
        }
    // 以下数据始终从裁判系统获取
    chassis_feedback_data.right_bullet_heat = referee_Data->PowerHeatData.shooter_17mm_barrel_heat;
    chassis_feedback_data.bullet_num = referee_Data->ProjectileAllowance.projectile_allowance_17mm;
    chassis_feedback_data.bullet_speed = referee_Data->ShootData.bullet_speed;
    chassis_feedback_data.enemy_color = COLOR_RED;
}

/* 机器人底盘控制核心任务 */
void ChassisTask()
{
    SubGetMessage(chassis_sub, &chassis_cmd_recv);
    SendJudgeData(referee_data);
    ChassisStateSet();
    // 根据控制模式进行正运动学解算,计算底盘输出
    MecanumCalculate();

    // 根据裁判系统的反馈数据和电容数据对输出限幅并设定闭环参考值
    LimitChassisOutput();
    // 推送反馈消息
    PubPushMessage(chassis_pub, (void *)&chassis_feedback_data);
    SendPowerData();
    SendChassisData();
    SuperCapSend(cap, (uint8_t*)&power_data);
}
