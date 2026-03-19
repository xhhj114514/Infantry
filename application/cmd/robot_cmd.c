// app
#include "can_comm.h"
#include "fast_math_functions.h"
#include "mi_motor.h"
#include "rm_referee.h"
#include "robot_def.h"
#include "robot_cmd.h"
// module
#include "remote_control.h"
#include "ins_task.h"
#include "master_process.h"
#include "message_center.h"
#include "general_def.h"
#include "dji_motor.h"

#include "gimbal.h"
#include "referee_UI.h"
#include "referee_task.h"

// bsp
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "robot_def.h"
#include "can_comm.h"

#include "chassis.h"
#include <string.h>


static  uint32_t PC_PRSC;


// 私有宏,自动将编码器转换成角度值
#define YAW_ALIGN_ANGLE (YAW_CHASSIS_ALIGN_ECD * ECD_ANGLE_COEF_DJI) // 对齐时的角度,0-360
#define PTICH_HORIZON_ANGLE (PITCH_HORIZON_ECD * ECD_ANGLE_COEF_DJI) // pitch水平时电机的角度,0-360

/* cmd应用包含的模块实例指针和交互信息存储*/
static Publisher_t *chassis_cmd_pub;   // 底盘控制消息发布者
static Subscriber_t *chassis_feed_sub; // 底盘反馈信息订阅者

static Chassis_Ctrl_Cmd_s chassis_cmd_send;      // 发送给底盘应用的信息,包括控制信息和UI绘制相关
static Chassis_Upload_Data_s chassis_fetch_data; // 从底盘应用接收的反馈信息信息,底盘功率枪口热量与底盘运动状态等

static RC_ctrl_t *rc_data;              // 遥控器数据,初始化时返回
static Minipc_Recv_s *minipc_recv_data; // 视觉接收数据指针,初始化时返回
static Minipc_Send_s minipc_send_data;  // 视觉发送数据

static Publisher_t *gimbal_cmd_pub;            // 云台控制消息发布者
static Subscriber_t *gimbal_feed_sub;          // 云台反馈信息订阅者
static Gimbal_Ctrl_Cmd_s gimbal_cmd_send;      // 传递给云台的控制信息
static Gimbal_Upload_Data_s gimbal_fetch_data; // 从云台获取的反馈信息

static Publisher_t *shoot_cmd_pub;           // 发射控制消息发布者
static Subscriber_t *shoot_feed_sub;         // 发射反馈信息订阅者
static Shoot_Ctrl_Cmd_s shoot_cmd_send;      // 传递给发射的控制信息
static Shoot_Upload_Data_s shoot_fetch_data; // 从发射获取的反馈信息

static Robot_Status_e robot_state; // 机器人整体工作状态
static DataLebel_t DataLebel;

static uint8_t gimbal_location_init=0;
static uint8_t power_flag;

static referee_info_t* referee_data; // 用于获取裁判系统的数据
static Referee_Interactive_info_t ui_data; // UI数据，将底盘中的数据传入此结构体的对应变量中，UI会自动检测是否变化，对应显示UI
static float cnt1,cnt2; 
static float chassis_rotate_buff;
static float chassis_speed_buff;

static cal_round_patrol_t round_patrol;
static cal_mid_round_patrol_t mid_round_patrol;
static cal_temporary_round_patrol_t tem_round_patrol;
static uint32_t YAW_CHASSIS_ALIGN_ECD;
static uint8_t YAW_ECD_GREATER_THAN_4096;


static BoardCommInstance* Referee_can_commrecv;
static Referee_Ctrl_Cmd_s Referee_can_CTRL; 

void RobotCMDInit()
{
    referee_data= UITaskInit(&huart6,&ui_data);
    
#ifdef Gimbal_Board 
    rc_data = RemoteControlInit(&huart3);   // 修改为对应串口,注意如果是自研板dbus协议串口需选用添加了反相器的那个
    minipc_recv_data = minipcInit(&huart1); // 视觉通信串口
    memset(minipc_recv_data, 0, sizeof(Minipc_Recv_s));
            //双板通信Sender
    // BoardComm_Init_Config_s comm_conf = {
    //     .can_config = {
    //         .can_handle = &hcan1,
    //         //云台的tx是底盘的rx，别搞错了！！！
    //         .tx_id = 0x200,
    //         .rx_id = 0x209,
    //     },
    //     .recv_data_len = sizeof(Referee_Ctrl_Cmd_s),
    //     .send_data_len = sizeof(Referee_Ctrl_Cmd_s),
    // };
    // Referee_can_commrecv = BoardCommInit(&comm_conf);
#endif

    gimbal_cmd_pub = PubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    gimbal_feed_sub = SubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    shoot_cmd_pub = PubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
    shoot_feed_sub = SubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));

    chassis_cmd_pub = PubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_feed_sub = SubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));

    gimbal_cmd_send.pitch = 0;

    YAW_CHASSIS_ALIGN_ECD = ALIGNECD;
    YAW_ECD_GREATER_THAN_4096 = ALIGNECD_GREATER_THAN_4096;
}


/**
 * @brief 根据gimbal app传回的当前电机角度计算和零位的误差
 *        单圈绝对角度的范围是0~360,说明文档中有图示
 *
 */
static void CalcOffsetAngle()
{
    gimbal_fetch_data.offset_diff =  gimbal_fetch_data.yaw_motor_single_round_angle-YAW_ALIGN_ANGLE;
    // 别名angle提高可读性,不然太长了不好看,虽然基本不会动这个函数
    static float angle;
    angle = gimbal_fetch_data.yaw_motor_single_round_angle; // 从云台获取的当前yaw电机单圈角度
    if(YAW_ECD_GREATER_THAN_4096 == 1)
    {
        if (angle > YAW_ALIGN_ANGLE && angle <= 180.0f + YAW_ALIGN_ANGLE)
            chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
        else if (angle > 180.0f + YAW_ALIGN_ANGLE)
            chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE - 360.0f;
        else
            chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
    }
    else if(YAW_ECD_GREATER_THAN_4096 == 0)
    {
        if (angle > YAW_ALIGN_ANGLE)
            chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
        else if (angle <= YAW_ALIGN_ANGLE && angle >= YAW_ALIGN_ANGLE - 180.0f)
            chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
        else
            chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE + 360.0f;
        }
    else {
        chassis_cmd_send.offset_angle=0;
    }
// #if YAW_ECD_GREATER_THAN_4096                               // 如果大于180度
//     if (angle > YAW_ALIGN_ANGLE && angle <= 180.0f + YAW_ALIGN_ANGLE)
//         chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
//     else if (angle > 180.0f + YAW_ALIGN_ANGLE)
//         chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE - 360.0f;
//     else
//         chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
// #else // 小于180度
//     if (angle > YAW_ALIGN_ANGLE)
//         chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
//     else if (angle <= YAW_ALIGN_ANGLE && angle >= YAW_ALIGN_ANGLE - 180.0f)
//         chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
//     else
//         chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE + 360.0f;
// #endif

}

static void GimbalPitchLimit()
{
    gimbal_cmd_send.gimbal_mode=GIMBAL_GYRO_MODE;
    // 云台软件限位
    if(gimbal_cmd_send.pitch<PITCH_MIN_ANGLE)
    gimbal_cmd_send.pitch=PITCH_MIN_ANGLE;
    else if (gimbal_cmd_send.pitch>PITCH_MAX_ANGLE)
    gimbal_cmd_send.pitch=PITCH_MAX_ANGLE;
    else
    gimbal_cmd_send.pitch=gimbal_cmd_send.pitch;
}

/**
 * @brief 判断视觉有没有发信息
 *
 */
static void VisionJudge()
{
    if (minipc_recv_data->Vision.shoot_flag == 1) // 代表收到信息
    {
        DataLebel.vision_flag = 1;
        DataLebel.fire_flag = minipc_recv_data->Vision.shoot_flag;
    }
    else DataLebel.fire_flag = 0;
}

static void BasicSet()
{
    VisionJudge();
    CalcOffsetAngle();
    GimbalPitchLimit();
    
    //发射基本模式设定
    shoot_cmd_send.shoot_mode = SHOOT_ON;
    shoot_cmd_send.friction_mode = FRICTION_ON;
    shoot_cmd_send.shoot_rate=8;
    chassis_cmd_send.power_limit=referee_data->GameRobotState.chassis_power_limit;

}


static void GimbalRC()
{
    gimbal_cmd_send.yaw -= 0.0035f * (float)rc_data[TEMP].rc.rocker_right_x;//0.0005f * (float)rc_data[TEMP].rc.rocker_right_x
    gimbal_cmd_send.pitch -= 0.00005f * (float)rc_data[TEMP].rc.rocker_right_y;
    gimbal_cmd_send.real_pitch = ((gimbal_fetch_data.gimbal_imu_data.Pitch)-gimbal_fetch_data.init_location)/57.39;
}

/**
* @brief minipc-OKGIMBALAUTOAIM
 */
static void INFANTRY_GimbalAC()
{
    gimbal_cmd_send.yaw = gimbal_fetch_data.gimbal_imu_data.YawTotalAngle - minipc_recv_data->Vision.yaw;   //往右获得的yaw是减
    gimbal_cmd_send.pitch = gimbal_fetch_data.pitch_angle - 0.2*minipc_recv_data->Vision.pitch*DEGREE_2_RAD;
}

static void ChassisRC()
{
    if(gimbal_fetch_data.offset_diff >= -45 && gimbal_fetch_data.offset_diff <= 30)//BASE
    {
        chassis_cmd_send.vx = 1*chassis_cmd_send.vx_dir;
        chassis_cmd_send.vy = 1*chassis_cmd_send.vy_dir;
        chassis_cmd_send.offset_angle -= 0;

    }
    else if(gimbal_fetch_data.offset_diff >=50 && gimbal_fetch_data.offset_diff <=110)//2048
    {
        chassis_cmd_send.vx = -1*chassis_cmd_send.vy_dir;
        chassis_cmd_send.vy = 1*chassis_cmd_send.vx_dir;
        chassis_cmd_send.offset_angle -= 81;

    }
    else if(gimbal_fetch_data.offset_diff >=130 && gimbal_fetch_data.offset_diff <= 190)//3072
    {
        chassis_cmd_send.vx = -1*chassis_cmd_send.vx_dir;
        chassis_cmd_send.vy = -1*chassis_cmd_send.vy_dir;
        chassis_cmd_send.offset_angle -= 170;
 
    }
    else if(gimbal_fetch_data.offset_diff >=225 || gimbal_fetch_data.offset_diff <= -70)//3072
    {
        chassis_cmd_send.vx = 1*chassis_cmd_send.vy_dir;
        chassis_cmd_send.vy = -1*chassis_cmd_send.vx_dir;
        chassis_cmd_send.offset_angle -= 260;

    }
    else
    { 
        chassis_cmd_send.vx = 1*chassis_cmd_send.vx_dir;
        chassis_cmd_send.vy = 1*chassis_cmd_send.vy_dir;

    }

    chassis_cmd_send.vx_dir = 2.0f * (float)rc_data[TEMP].rc.rocker_left_y ;// /660.0   * 4.0f * REDUCTION_RATIO_WHEEL * 360.0f / PERIMETER_WHEEL * 1000.0f; // _水平方向
    chassis_cmd_send.vy_dir =-2.0f * (float)rc_data[TEMP].rc.rocker_left_x ;// /660.0   * 4.0f * REDUCTION_RATIO_WHEEL * 360.0f / PERIMETER_WHEEL * 1000.0f; // 竖直方向


}

static void GetGimbalInitImu()
{
    if (mid_round_patrol.flag == 0)
    {
        mid_round_patrol.yaw_init = gimbal_fetch_data.gimbal_imu_data.Yaw;
        mid_round_patrol.yaw = mid_round_patrol.yaw_init;
        mid_round_patrol.flag = 1;
    }
}

static void RoundPatrol()
{
    if (round_patrol.flag == 0)
    {
        round_patrol.init_totol_round = gimbal_fetch_data.gimbal_imu_data.YawTotalAngle / 360.0f;
        round_patrol.flag = 1;
    }
    round_patrol.total_round = (gimbal_fetch_data.gimbal_imu_data.YawTotalAngle / 360.0f) - round_patrol.init_totol_round;
    gimbal_cmd_send.yaw += 0.5f;
}

void FoundEnermy()
{
    gimbal_cmd_send.autoaim_mode = AUTO_ON;
    if (abs(minipc_recv_data->Vision.yaw) > 5 && abs(minipc_recv_data->Vision.yaw) < 20)
    {
        gimbal_cmd_send.yaw -= (0.032f * minipc_recv_data->Vision.yaw) + 0.00001; // 往右获得的yaw是减 //0.0036
    }
    else
    {
        gimbal_cmd_send.yaw -= (0.032f * minipc_recv_data->Vision.yaw); // 往右获得的yaw是减 //0.00355
    }
    gimbal_cmd_send.pitch += 0.0011f * minipc_recv_data->Vision.pitch;//.00037
}


#define PERIOD_COUNTS   250    // 800個週期tick ≈ 8秒（假設100Hz調用）
#define AMPLITUDE_Q15   22937  // 0.8 * 32768 ≈ Q15定點
#define CENTER_Q15     -12780  // -0.39 * 32768

static int32_t counter = 0;
static int8_t direction = 1;

float update_pitch_triangle_fixed(void)
{
    counter += direction;

    if (counter >= PERIOD_COUNTS/2)  direction = -1;
    if (counter <= -PERIOD_COUNTS/2) direction = +1;


    int32_t amp_part = (int64_t)AMPLITUDE_Q15 * counter / (PERIOD_COUNTS/2);

    return (CENTER_Q15 + amp_part) / 32768.0f;

}

void OLD_SENTRY_GimbalAC()
{
    static MIMotorInstance *PP_Motor;
    PP_Motor = GetPitchMotor();
    static float PIT;
    static uint8_t stage = 0;
    GetGimbalInitImu();
    
    // 没发信息时巡逻
    if (DataLebel.vision_flag == 0)
    {
        DataLebel.t_pitch = (float32_t)DWT_GetTimeline_s();//+-0.6->  -0.1  -0.86
        PIT = 0.2f * sinf(10.0f * DataLebel.t_pitch)-0.35;
        GimbalPitchLimit();
        if (fabs(PP_Motor->measure.angle - PIT)>=0.1 && DataLebel.ACEntryPoint)
        {
            gimbal_cmd_send.pitch = 0.5f * PIT;
            if(fabs(PP_Motor->measure.angle - PIT)<=0.01)
            {
                DataLebel.ACEntryPoint = 0;
            }
            RoundPatrol();
        }
        else {
            gimbal_cmd_send.pitch = PIT;
            DataLebel.ACEntryPoint = 0;
            RoundPatrol();
        }
        gimbal_cmd_send.autoaim_mode = AUTO_OFF;
        // else
        // {
        //     MidRoundPatrol();
        // }
    }
    else
    {
        // if(DataLebel.flag==2)
        // {
        //     TemporaryPatrol();
        // }
        // else
        // {
        FoundEnermy();
        // }
    }
}

static void Sentry_ChassisAC()
{
       chassis_cmd_send.vx = minipc_recv_data->Vision.linevx  * 4.0f * REDUCTION_RATIO_WHEEL * 360.0f / PERIMETER_WHEEL * 1000.0f;
       chassis_cmd_send.vy = -minipc_recv_data->Vision.linevy  * 4.0f * REDUCTION_RATIO_WHEEL * 360.0f / PERIMETER_WHEEL * 1000.0f;
    if (minipc_recv_data->Vision.gimbal_mode == CHASSIS_ROTATE)
    {
        chassis_cmd_send.chassis_mode=CHASSIS_ROTATE;
        if(chassis_fetch_data.power_flag==1)
        {
            chassis_cmd_send.chassis_rotate_buff= 2;
        }
        else
        {
            chassis_cmd_send.chassis_rotate_buff= 1.0;
        }
    }
    else
        chassis_cmd_send.chassis_mode=CHASSIS_FOLLOW_GIMBAL_YAW;
}

static void ChassisRotateSet()
{
    // 根据控制模式设定旋转速度
    switch (chassis_cmd_send.chassis_mode)
    {
        //底盘跟随就不调了，懒
        case CHASSIS_FOLLOW_GIMBAL_YAW: // 底盘不旋转,但维持全向机动,一般用于调整云台姿态
            chassis_cmd_send.wz =  (-30.0*abs(chassis_cmd_send.offset_angle)*chassis_cmd_send.offset_angle);
        break;
        case CHASSIS_ROTATE: // 变速小陀螺
            // chassis_cmd_send.wz = 4000*chassis_cmd_send.chassis_rotate_buff;
            chassis_cmd_send.wz =(15000+2000*sin(DWT_GetTimeline_s()*7))*chassis_cmd_send.chassis_rotate_buff;
        break;
        default:
            chassis_cmd_send.wz = 0.0;
        break;
    }
}



// static void AutoAimSet()//看数据
// {
//     INFANTRY_GimbalAC();
//     // if(minipc_recv_data->Vision.gimbal_mode == CHASSIS_ROTATE)
//     // {
//     //     ChassisRotateSet();
//     // }
//     // if(DataLebel.aim_flag==1)
//     // {
//         // GimbalAC();
//         // if(DataLebel.fire_flag==1)
//         // {
//         //     shoot_cmd_send.loader_mode = LOAD_BURSTFIRE;
//         // }
//     // }
// }


static void ShootAC()
{
    if(DataLebel.fire_flag == 1)
    {
        shoot_cmd_send.loader_mode=LOAD_BURSTFIRE;
    }
    else
    {
        shoot_cmd_send.loader_mode=LOAD_STOP;
        DataLebel.reverse_flag=0;
    }
}

static void ShootRC()
{
    if(rc_data->rc.dial>200)
    {
        shoot_cmd_send.loader_mode=LOAD_BURSTFIRE;
    }
    else if (rc_data->rc.dial<-200)
    {
        shoot_cmd_send.loader_mode=LOAD_REVERSE;
        DataLebel.reverse_flag=1;
    }
    else
    {
        shoot_cmd_send.loader_mode=LOAD_STOP;
        DataLebel.reverse_flag=0;
    }
}

static void MidRoundPatrol()
{
    DJIMotorInstance *yaw_motor = GetYawMotor();
    // 如果巡逻模式刚刚被激活，重新初始化
    if (mid_round_patrol.flag == 0 || yaw_motor->Power_out == 1)
    {
        mid_round_patrol.yaw_init = gimbal_fetch_data.gimbal_imu_data.Yaw;
        mid_round_patrol.yaw = mid_round_patrol.yaw_init;
        mid_round_patrol.flag = 1;
    }
    
    // 使用当前实际角度计算，而不是历史角度
    float current_relative_angle = gimbal_fetch_data.gimbal_imu_data.Yaw - mid_round_patrol.yaw_init;
    
    // 更新目标角度
    current_relative_angle += 0.15f * mid_round_patrol.direction;
    
    // 边界检查
    if (current_relative_angle > 70.0f)
    {
        current_relative_angle = 70.0f;
        mid_round_patrol.direction = -1;
    }
    else if (current_relative_angle < -70.0f)
    {
        current_relative_angle = -70.0f;
        mid_round_patrol.direction = 1;
    }
    
    // 设置云台指令
    gimbal_cmd_send.yaw = current_relative_angle + round_patrol.total_round * 360.0f;
    
    // 更新内部状态（可选，用于显示等）
    mid_round_patrol.yaw_total_angle = current_relative_angle;
}

/**
 * @brief 控制输入为遥控器(调试时)的模式和控制量设置
 * @note  LEFT DOWN AUTOAIM;MID GIMBAL;UP CHASSISROTATE  左上反向小陀螺   左中正向  左下FOLLOW GIMBAL YAW
 */
static void RemoteControlSet()
{

    if(switch_is_up(rc_data[TEMP].rc.switch_left)) 
    {
        chassis_cmd_send.chassis_mode=CHASSIS_ROTATE;
        if(chassis_fetch_data.power_flag==1)
        {
            chassis_cmd_send.chassis_rotate_buff= -2;
        }
        else
        {
            chassis_cmd_send.chassis_rotate_buff= -1.0;
        }
        ChassisRC();
        ChassisRotateSet();
        GimbalRC();

    }
    else if(switch_is_mid(rc_data[TEMP].rc.switch_left))
    {
        chassis_cmd_send.chassis_mode=CHASSIS_ROTATE;
        if(chassis_fetch_data.power_flag==1)
        {
            chassis_cmd_send.chassis_rotate_buff= 2;
        }
        else
        {
            chassis_cmd_send.chassis_rotate_buff= 1.0;
        }
        ChassisRC();
        ChassisRotateSet();
        GimbalRC();
    }
    else
    {
        chassis_cmd_send.chassis_mode=CHASSIS_FOLLOW_GIMBAL_YAW;
        chassis_cmd_send.chassis_rotate_buff = 0.0;
        chassis_cmd_send.wz = 0.0;
        ChassisRC();
        ChassisRotateSet();
        GimbalRC();
        ShootRC();
    }
    DataLebel.ACEntryPoint = 1;
}
static void NoneAutoMouseControl()
{
    gimbal_cmd_send.yaw -= Referee_can_CTRL.mouse_x/ 32767.0f*50; //(float)rc_data[TEMP].mouse.x / 660 *3 ; 
    gimbal_cmd_send.pitch -= Referee_can_CTRL.mouse_y/32767.0f*0.8; //(float)rc_data[TEMP].mouse.y / 660/57 ;
    if(Referee_can_CTRL.mouse_left ==1)
    {
        if(DataLebel.reverse_flag==1)
        {
            shoot_cmd_send.loader_mode = LOAD_REVERSE;
        }
        else
        {
            shoot_cmd_send.loader_mode = LOAD_BURSTFIRE;
        }
    }
    else
    {
        shoot_cmd_send.loader_mode = LOAD_STOP;
    }            
}
static void MouseControl()
{
    DataLebel.aim_flag=0;
    if(Referee_can_CTRL.mouse_right==1)
    {
        if(DataLebel.aim_flag!=1)
        {
            gimbal_cmd_send.autoaim_mode=AUTO_ON;
        }
        else
        {
            gimbal_cmd_send.autoaim_mode=FIND_Enermy;
        }
    }
    else
    {
        gimbal_cmd_send.autoaim_mode=AUTO_OFF;
    }

    if(gimbal_cmd_send.autoaim_mode==AUTO_ON||gimbal_cmd_send.autoaim_mode==FIND_Enermy)
    {
        // AutoAimSet();
        if(DataLebel.aim_flag!=1)
        {
            NoneAutoMouseControl();
        }
    }
    else
    {
        NoneAutoMouseControl();
    }
}



static void KeyControl()
{
    // chassis_cmd_send.vx = (rc_data[TEMP].key[KEY_PRESS].w * 20000 - rc_data[TEMP].key[KEY_PRESS].s * 20000)*chassis_speed_buff; 
    // chassis_cmd_send.vy = (rc_data[TEMP].key[KEY_PRESS].d * 20000 - rc_data[TEMP].key[KEY_PRESS].a * 20000)*chassis_speed_buff;
    //TODO 搞个插值 高低速出来的不同
    if(gimbal_fetch_data.offset_diff >= -45 && gimbal_fetch_data.offset_diff <= 30)//BASE
    {
        chassis_cmd_send.vx = 1*chassis_cmd_send.vx_dir;
        chassis_cmd_send.vy = 1*chassis_cmd_send.vy_dir;
        chassis_cmd_send.offset_angle -= 0;

    }
    else if(gimbal_fetch_data.offset_diff >=50 && gimbal_fetch_data.offset_diff <=110)//2048
    {
        chassis_cmd_send.vx = -1*chassis_cmd_send.vy_dir;
        chassis_cmd_send.vy = 1*chassis_cmd_send.vx_dir;
        chassis_cmd_send.offset_angle -= 81;

    }
    else if(gimbal_fetch_data.offset_diff >=130 && gimbal_fetch_data.offset_diff <= 190)//3072
    {
        chassis_cmd_send.vx = -1*chassis_cmd_send.vx_dir;
        chassis_cmd_send.vy = -1*chassis_cmd_send.vy_dir;
        chassis_cmd_send.offset_angle -= 170;
 
    }
    else if(gimbal_fetch_data.offset_diff >=225 || gimbal_fetch_data.offset_diff <= -70)//3072
    {
        chassis_cmd_send.vx = 1*chassis_cmd_send.vy_dir;
        chassis_cmd_send.vy = -1*chassis_cmd_send.vx_dir;
        chassis_cmd_send.offset_angle -= 260;

    }
    else
    { 
        chassis_cmd_send.vx = 1*chassis_cmd_send.vx_dir;
        chassis_cmd_send.vy = 1*chassis_cmd_send.vy_dir;

    }
    chassis_cmd_send.vx_dir = (Referee_can_CTRL.w * 5.0 - Referee_can_CTRL.s * 5.0)*chassis_speed_buff* 4.0f * REDUCTION_RATIO_WHEEL * 360.0f / PERIMETER_WHEEL * 1000.0f;; 
    chassis_cmd_send.vy_dir = (Referee_can_CTRL.a * 5.0 - Referee_can_CTRL.d * 5.0)*chassis_speed_buff* 4.0f * REDUCTION_RATIO_WHEEL * 360.0f / PERIMETER_WHEEL * 1000.0f;;

    ChassisRotateSet();
    switch (referee_data->GameRobotState.robot_level)
    {
    case 1:
        chassis_rotate_buff = 1;
        chassis_speed_buff  = 1;
        break;
    case 2:
        chassis_rotate_buff = 1.2;
        chassis_speed_buff  = 1.03;
        break;
    case 3:
        chassis_rotate_buff = 1.3;
        chassis_speed_buff  = 1.05;
        break;
    case 4:
        chassis_rotate_buff = 1.4;
        chassis_speed_buff  = 1.1;
        break;
    case 5:
        chassis_rotate_buff = 1.5;
        chassis_speed_buff  = 1.15;
        break;
    case 6:
        chassis_rotate_buff = 1.6;
        chassis_speed_buff  = 1.2;
        break;
    case 7:
        chassis_rotate_buff = 1.7;
        chassis_speed_buff  = 1.25;
        break;
    case 8:
        chassis_rotate_buff = 1.8;
        chassis_speed_buff  = 1.3;
        break;
    case 9:
        chassis_rotate_buff = 1.9;
        chassis_speed_buff  = 1.35;
        break;
    case 10:
        chassis_rotate_buff = 2;
        chassis_speed_buff  = 1.4;
        break;
    default:
        chassis_rotate_buff = 1;
        chassis_speed_buff  = 1;
        break;
    }

    if(chassis_fetch_data.power_flag==1)
    {
        chassis_cmd_send.chassis_rotate_buff= 2;
    }
    else
    {
        chassis_cmd_send.chassis_rotate_buff= chassis_rotate_buff;
    }

    switch (Referee_can_CTRL.r_cnt % 2) 
    {
    case 0:
        chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
        break;
    default:
        chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;
    }

    if(Referee_can_CTRL.q)
    {
        DataLebel.reverse_flag=1;
        shoot_cmd_send.loader_mode = LOAD_REVERSE;
    }
    else
    {
        DataLebel.reverse_flag=0;
    }

    switch (Referee_can_CTRL.shift) // 待添加 按shift允许超功率 消耗缓冲能量
    {
    case 1:
        chassis_cmd_send.chassis_speed_buff= 2;
        break;
    default:
        break;
    }
}



/**
 * @brief 输入为键鼠时模式和控制量设置
 *
 */
static void MouseKeySet()
{
    MouseControl();
    KeyControl();//校对裁判数据，能继续用
}

/**
 * @brief RIGHT-STICK UP STATE
 * @note  左上   左中  左下
 *
 */
static void SentrySet()
{
    // if(referee_data->GameState.game_progress == 4)
    // {
        Sentry_ChassisAC();
        ChassisRotateSet();
        ShootAC();
        OLD_SENTRY_GimbalAC();
    // }
}

void Deathcheck()
{
    if(referee_data->GameRobotState.current_HP == 0)
    {
        gimbal_cmd_send.Death_reInit = 1;
    }
    else {
        gimbal_cmd_send.Death_reInit = 0;
    }
    
}

/**
 * @brief 停止
 */
static void AnythingStop()
{
    gimbal_cmd_send.gimbal_mode=GIMBAL_ZERO_FORCE;
    chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
    shoot_cmd_send.shoot_mode = SHOOT_OFF;
    shoot_cmd_send.friction_mode = FRICTION_OFF;
    shoot_cmd_send.loader_mode = LOAD_STOP;
    gimbal_cmd_send.pitch = 0.0;
    DataLebel.ACEntryPoint = 1;
    //重置与小电脑通信失败的标志位
    DataLebel.cmd_error_flag=0;
}

/**
 * @brief 控制量及模式设置
 *
 */
static void ControlDataDeal()
{
    if (switch_is_mid(rc_data[TEMP].rc.switch_right)) 
    {
        BasicSet();
        RemoteControlSet();//Waiting for referee&&PhotoTransefer
    }
    else if (switch_is_up(rc_data[TEMP].rc.switch_right)) 
    {
        BasicSet();
#ifdef InfantryMode 
       MouseKeySet();
#endif 

#ifdef SentryMode
        SentrySet(); //FULLY AUTO 
#endif
    }
    else if (switch_is_down(rc_data[TEMP].rc.switch_right)) 
    {
        AnythingStop();
    }
}

static void EnemyJudge()
{
    if(referee_data->GameRobotState.robot_id>7)
    {
        minipc_send_data.Vision.detect_color = COLOR_RED;
    }
    else
    {
        minipc_send_data.Vision.detect_color = COLOR_BLUE;
    }
}
static void SendToUIData()
{
    ui_data.autoaim_mode=gimbal_cmd_send.autoaim_mode;
    ui_data.chassis_mode=chassis_cmd_send.chassis_mode;
    ui_data.loader_mode=shoot_cmd_send.loader_mode;
    ui_data.shoot_mode=shoot_cmd_send.shoot_mode;
    ui_data.gimbal_mode=gimbal_cmd_send.gimbal_mode;
}
//TEST
// static uint16_t ch_0;
// static uint16_t ch_1;
// static uint16_t ch_2;
// static uint16_t ch_3;
// static uint8_t mode_sw;
// static uint8_t pause;
// static uint8_t fn_1;
// static uint8_t fn_2;
// static uint16_t wheel;
// static uint8_t trigger;
static uint8_t OCCU = 0;
static uint8_t match=0;
static uint16_t HP = 400;
static uint16_t TIME=300;

/* 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率) */
void RobotCMDTask()
{
    PC_PRSC = 0;
    SubGetMessage(chassis_feed_sub, (void *)&chassis_fetch_data);
    SubGetMessage(shoot_feed_sub, &shoot_fetch_data);
    SubGetMessage(gimbal_feed_sub, &gimbal_fetch_data);
    Deathcheck();
    // 根据gimbal的反馈值计算云台和底盘正方向的夹角,不需要传参,通过static私有变量完成
    CalcOffsetAngle();
    ControlDataDeal();
    
	// ch_0 = referee_data->VT03.ch_0;
	// ch_1 = referee_data->VT03.ch_1;
	// ch_2 = referee_data->VT03.ch_2;
	// ch_3 = referee_data->VT03.ch_3;
	// mode_sw = referee_data->VT03.mode_sw;
        /**************************************    SendData    **************************************/
    // 设置巡航和视觉需要用到的数据
    SendJudgeData(referee_data);
    NavSetMessage(
        chassis_fetch_data.real_vx,
        chassis_fetch_data.real_vy,
        gimbal_fetch_data.gimbal_imu_data.Yaw,
        ((referee_data->EventData.event_type >> 21) & 0x03),
        referee_data->GameRobotHP.self_sentry_HP,
        referee_data->GameRobotHP.self_3_fantry_HP,
        referee_data->GameRobotHP.self_Hero_HP ,
        0,
        0,
        0,
        referee_data->GameState.stage_remain_time,
        shoot_cmd_send.bullet_num,
        referee_data->GameState.game_progress,
        chassis_fetch_data.enemy_color,
        referee_data->ShootData.bullet_speed
    );
    VisionSetAltitude();
        if(PC_PRSC % 5 == 0)
        {
            SendMinipcData(); // 解算完成后发送视觉数据,但是当前的实现不太优雅,后续若添加硬件触发需要重新考虑结构的组织
            PC_PRSC = 0;
        }
        PC_PRSC++;    // 设置视觉发送数据,还需增加加速度和角速度数据
    // 推送消息,双板通信,视觉通信等
    PubPushMessage(chassis_cmd_pub, (void *)&chassis_cmd_send);
    PubPushMessage(shoot_cmd_pub, (void *)&shoot_cmd_send);
    PubPushMessage(gimbal_cmd_pub, (void *)&gimbal_cmd_send);
    SendToUIData();

#ifdef Gimbal_Board
    // Referee_can_CTRL = *(Referee_Ctrl_Cmd_s*)BoardCommGet(Referee_can_commrecv);
#endif

}
