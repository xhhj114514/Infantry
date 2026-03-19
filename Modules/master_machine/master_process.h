#ifndef MASTER_PROCESS_H
#define MASTER_PROCESS_H

#include "bsp_usart.h"
#include "seasky_protocol.h"

#define Minipc_Recv_sIZE 36u // 当前为固定值,36字节
#define Minipc_Send_sIZE 36+4u

#pragma pack(1)
typedef enum
{
	NO_FIRE = 0,
	AUTO_FIRE = 1,
	AUTO_AIM = 2
} Fire_Mode_e;

typedef enum
{
	NO_TARGET = 0,
	TARGET_CONVERGING = 1,
	READY_TO_FIRE = 2
} Target_State_e;

typedef enum
{
	NO_TARGET_NUM = 0,
	HERO1 = 1,
	ENGINEER2 = 2,
	INFANTRY3 = 3,
	INFANTRY4 = 4,
	INFANTRY5 = 5,
	OUTPOST = 6,
	SENTRY = 7,
	BASE = 8
} Target_Type_e;



typedef struct
{
	uint8_t header;  // 帧头，固定为0x5A
	struct
	{
		float linevx;
		float linevy;
		uint8_t gimbal_mode;
		float yaw;
		float pitch;
		int8_t shoot_flag;
		int32_t time;
	}Vision;
	// struct
	// {
                // header, 
                // linear_velocity_x,
                // linear_velocity_y, 
                // gimbal_mode,
                // yaw, 
                // pitch, 
                // can_fire, 
	// }NAV;

} __attribute__((packed)) Minipc_Recv_s;

typedef enum
{
	COLOR_BLUE = 1,
	COLOR_RED = 0,
} Enemy_Color_e;

typedef enum
{
	VISION_MODE_AIM = 0,
	VISION_MODE_SMALL_BUFF = 1,
	VISION_MODE_BIG_BUFF = 2,
} Vision_Work_Mode_e;

typedef struct
{
	uint8_t header;  // 帧头，固定为0xA5
	struct
	{
		uint8_t detect_color;
		float roll;
		float pitch;
		float yaw;
		float vx;
		float vy;
		float bspeed;
		uint16_t self_sentry_HP;
		uint16_t self_hero_HP;
		uint16_t self_infantry_HP;
		uint16_t remain_time;
		uint16_t remain_bullet;
		uint8_t match_progress;
		uint8_t occupation;
	}Vision;
	// struct
	// {
            // header = 0xA5          # 假设帧头是 0x5A (B)
            // detect_color = 1       # 红色 (B)
            // roll = 1.2             # (f)
            // pitch = -0.5           # (f)
            // yaw = 3.14             # (f)
            // vx = 0.5               # (f)
            // vy = 0.2               # (f)
            // sentry_hp = 600        # (H)
            // hero_hp = 1500         # (H)
            // infantry_hp = 200      # (H)
            // remain_time = 420      # (H)
            // remain_bullet = 150    # (H)
            // match_progress = 2     # (B)
            // occupation = 0         # (B)
	// }NAV;

} __attribute__((packed)) Minipc_Send_s;







#pragma pack()

/**
 * @brief 调用此函数初始化和视觉的串口通信
 *
 * @param handle 用于和视觉通信的串口handle(C板上一般为USART1,丝印为USART2,4pin)
 */
Minipc_Recv_s *minipcInit(UART_HandleTypeDef *_handle);

/**
 * @brief 发送视觉数据
 *
 */
void SendMinipcData();


void NavSetMessage(float vx, float vy, float yaw,uint8_t occupation,
					uint16_t self_sentry_HP,uint16_t self_infantry_HP,uint16_t self_hero_HP,
					uint16_t enermy_sentry_HP,uint16_t enermy_infantry_HP,uint16_t enermy_hero_HP,
                    uint16_t remain_time,uint16_t remain_bullet,uint8_t game_progress,uint8_t detect_color,float bspeed
					);




/*更新发送数据帧，并计算发送数据帧长度*/
void get_protocol_send_Vision_data(uint16_t send_id,        // 信号id
                            uint16_t flags_register, // 16位寄存器
                            Minipc_Send_s *tx_data,          // 待发送的float数据
                            uint8_t float_length,    // float的数据长度
                            uint8_t *tx_buf,         // 待发送的数据帧
                            uint16_t *tx_buf_len) ;   // 待发送的数据帧长度



void get_protocol_info_vision(uint8_t *rx_buf, 
                           uint16_t *flags_register, 
                           Minipc_Recv_s *recv_data);

						   void VisionSetAltitude();

#endif // !MASTER_PROCESS_H