#include "master_process.h"
#include "seasky_protocol.h"
#include "crc8.h"
#include "crc16.h"
#include "crc_ref.h"
#include "memory.h"
#include <string.h>
#include "robot_def.h"

static Minipc_Recv_s minipc_recv_data;
static Minipc_Send_s minipc_send_data;
/*获取CRC8校验码*/
uint8_t Get_CRC8_Check(uint8_t *pchMessage,uint16_t dwLength)
{
    return crc_8(pchMessage,dwLength);
}
/*检验CRC8数据段*/
static uint8_t CRC8_Check_Sum(uint8_t *pchMessage, uint16_t dwLength)
{
    uint8_t ucExpected = 0;
    if ((pchMessage == 0) || (dwLength <= 2))
        return 0;
    ucExpected = crc_8(pchMessage, dwLength - 1);
    return (ucExpected == pchMessage[dwLength - 1]);
}

/*获取CRC16校验码*/
uint16_t Get_CRC16_Check(uint8_t *pchMessage,uint32_t dwLength)
{
    return crc_16(pchMessage,dwLength);
}

/*检验CRC16数据段*/
static uint16_t CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength)
{
    uint16_t wExpected = 0;
    if ((pchMessage == 0) || (dwLength <= 2))
    {
        return 0;
    }
    wExpected = crc_16(pchMessage, dwLength - 2);
    return (((wExpected & 0xff) == pchMessage[dwLength - 2]) && (((wExpected >> 8) & 0xff) == pchMessage[dwLength - 1]));
}

/*检验数据帧头*/
static uint8_t protocol_heade_Check(protocol_rm_struct *pro, uint8_t *rx_buf)
{
    if (rx_buf[0] == PROTOCOL_CMD_ID)
    {
        pro->header.sof = rx_buf[0]; 
        //pro->header.data_length = (rx_buf[2] << 8) | rx_buf[1];
        //pro->header.crc_check = rx_buf[3];
        //pro->cmd_id = (rx_buf[5] << 8) | rx_buf[4];
        return 1;
    }
    return 0;
}

/*
    此函数根据待发送的数据更新数据帧格式以及内容，实现数据的打包操作
    后续调用通信接口的发送函数发送tx_buf中的对应数据
*/
void get_protocol_send_Vision_data(uint16_t send_id,        // 信号id
                            uint16_t flags_register, // 16位寄存器
                            Minipc_Send_s *tx_data,          // 待发送的float数据
                            uint8_t float_length,    // float的数据长度
                            uint8_t *tx_buf,         // 待发送的数据帧
                            uint16_t *tx_buf_len)    // 待发送的数据帧长度
{
    static uint16_t crc16;
    static uint16_t data_len;

    tx_buf[0] = SEND_ID;
	memcpy(&tx_buf[1], &tx_data->Vision.detect_color, 1);
    memcpy( &tx_buf[2],&tx_data->Vision.roll, 4);
    memcpy( &tx_buf[6],&tx_data->Vision.pitch, 4);
    memcpy( &tx_buf[10],&tx_data->Vision.yaw, 4);
    memcpy( &tx_buf[14],&tx_data->Vision.bspeed, 4);

#ifdef InfantryMode
    //VISION
    Append_CRC16_Check_Sum(&tx_buf[0],20);
    *tx_buf_len = 20;
#endif
#ifdef SentryMode
    memcpy( &tx_buf[14],&tx_data->Vision.vx, 4);
    memcpy( &tx_buf[18],&tx_data->Vision.vy, 4);

    memcpy( &tx_buf[22],&tx_data->Vision.self_sentry_HP, 2);
    memcpy( &tx_buf[24],&tx_data->Vision.self_hero_HP, 2);
    memcpy( &tx_buf[26],&tx_data->Vision.self_infantry_HP, 2);

    memcpy( &tx_buf[28],&tx_data->Vision.remain_time, 2);
    memcpy( &tx_buf[30],&tx_data->Vision.remain_bullet, 2);
    memcpy( &tx_buf[32],&tx_data->Vision.match_progress, 1);
    memcpy( &tx_buf[33],&tx_data->Vision.occupation, 1);
    memcpy( &tx_buf[34],&tx_data->Vision.bspeed, 4);
    //NAV USART
    Append_CRC16_Check_Sum(&tx_buf[0],36 +4);
    *tx_buf_len = 36+4;
#endif
}

/*
    此函数用于处理接收数据，
    返回数据内容的id
*/
void get_protocol_info_vision(uint8_t *rx_buf, 
                           uint16_t *flags_register, 
                        Minipc_Recv_s *recv_data)
{
    static protocol_rm_struct pro;
    static uint16_t date_length;

    // if (protocol_heade_Check(&pro, rx_buf)==1) 
    // {
    //     date_length = OFFSET_BYTE + pro.header.data_length;
    //     //if (CRC16_Check_Sum(rx_buf, date_length)) {
    //         *flags_register = (rx_buf[7] << 8) | rx_buf[6];

    //         // 将接收到的数据复制到Minipc_Recv_s结构体中
    //         recv_data->Vision.header = rx_buf[0];
    //         memcpy(&recv_data->Vision.yaw, &rx_buf[1], sizeof(float));
    //         memcpy(&recv_data->Vision.pitch, &rx_buf[5], sizeof(float));
    //         memcpy(&recv_data->Vision.deep, &rx_buf[9], sizeof(float));
    //         recv_data->Vision.checksum = (rx_buf[date_length - 2] << 8) | rx_buf[date_length - 1];
    // // }
//     if(CRC16_Check_Sum(rx_buf, Minipc_Recv_sIZE) && rx_buf[Minipc_Recv_sIZE]== NAV_PROTOCOL_END_ID && rx_buf[0]== NAV_PROTOCOL_START_ID)
//     {
//         recv_data->NAV.header = rx_buf[0];
//         recv_data->NAV.gimbal_mode = rx_buf[9];
//         recv_data->NAV.fire_judge = rx_buf[15];
//         memcpy(&recv_data->NAV.line_vx, &rx_buf[1], sizeof(float));
//         memcpy(&recv_data->NAV.line_vy, &rx_buf[5], sizeof(float));
//         memcpy(&recv_data->NAV.yaw, &rx_buf[9], sizeof(float));
//         memcpy(&recv_data->NAV.pitch, &rx_buf[13], sizeof(float));
//         recv_data->Vision.detect_color = rx_buf[1];
// // 检查帧头
// }
        if (rx_buf[0] != PROTOCOL_CMD_ID)
        {
            // return 0;
        }
        else {
#ifdef InfantryMode
            //VISION
            memcpy(&recv_data->header, &rx_buf[0], 1);
            memcpy(&recv_data->Vision.yaw, &rx_buf[1], 4);
            memcpy(&recv_data->Vision.pitch, &rx_buf[5], 4);
            memcpy(&recv_data->Vision.shoot_flag, &rx_buf[9], 1);
            memcpy(&recv_data->Vision.time, &rx_buf[10], 4);
#endif
#ifdef SentryMode
            //NAV UART
            memcpy(&recv_data->header, &rx_buf[0], 1);
            memcpy(&recv_data->Vision.linevx, &rx_buf[1], 4);
            memcpy(&recv_data->Vision.linevy, &rx_buf[5], 4);
            memcpy(&recv_data->Vision.gimbal_mode, &rx_buf[9], 4);
            memcpy(&recv_data->Vision.yaw, &rx_buf[13], 4);
            memcpy(&recv_data->Vision.pitch, &rx_buf[17], 4);
            memcpy(&recv_data->Vision.shoot_flag, &rx_buf[21], 4);
            // recv_data->Vision.pitch *= -1;
#endif
        }

        // return 1; // 解析成功
    // }

}
