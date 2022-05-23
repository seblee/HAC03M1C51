/******************************************************************************
          版权所有 (C), 2020，DFX，Write by Food(181 1266 9427)
    本程序是基于DWin的4寸480*480温控器C语言例子程序，去除了不需要的按键检测、RTC等
不需要的命令、函数。
   配置如下：
     1. UART2 作为log的输出，用于监控程序的状态
     2. UART4 作为ModBus的通讯串口，处理发送和接收命令，
     3. 其他为迪文的DGUS基本配置，
     功能如下：
     1. 实现Modbus RTU主站命令，03读取命令，06写寄存器命令，控制、显示modbus从站状态
     2. 实现UI的显示，控制，状态的更新，

     说明：程序可自由传播，但请仔细阅读后再应用，对引起的任何问题无任何的责任。使用
过程中，如果有疑问，可以加V(181 1266 9427)共同探讨。
******************************************************************************/
#include "modbus.h"
#include "crc16.h"
#include "sys.h"
#include "uart.h"
#include "ui.h"

void Modbus_RX_Reset(void);
// void Modbus_TX_Reset(void);
void Modbus_Write_Register06H(modbosCmd_t *CmdNow, u16 value);
void Modbus_Write_Register10H(modbosCmd_t *CmdNow);
void Modbus_Read_Register03H(modbosCmd_t *CmdNow);
void modbus_process_command(u8 *pstr, u16 strlen);

u8 modbus_rx_count = 0;                 //接收到的字符串的长度
u8 modbus_rx_flag  = 0;                 //接收到的字符串的标志，为1表示有收到数据
u8 modbus_rx_buf[UART_RX_BUF_MAX_LEN];  //接收到的字符串的内容

extern u32 data ModbusSysTick;               //每隔1ms+1
u32             uart_rx_check_tick     = 0;  //检查串口是否接收结束
u8              modbus_rx_count_before = 0;  //接收串口的数据

u32 modbus_tx_process_tick = 0;  // modbus发送命令的时间间隔

const modbosCmd_t modbusCmdlib[] = {
  //  en         id         fun     len  timeout      mod    modP     VP  slaveAddr feedback
    {BUS_EN, SLAVE_ID, BUS_FUN_10H, 0x03, 0xc8, MODE_PARA, 0x5015, 0x5016, 0x0000, 0x00ff}, // rtc
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x03, 0xc8, MODE_ALWA, 0x0000, 0xa020, 0x031c, 0x00ff}, //告警数
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PAGE, PAGE57, 0xa023, 0x0319, 0x00ff}, //位状态
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xa084, 0xa024, 0x0100, PAGE57}, // power switch
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PAGE, PAGE57, 0xa025, 0x0104, 0x00ff}, //诊断模式
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PARA, 0xa087, 0xa027, 0x0104, 0x00ff}, //诊断模式
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x03, 0xc8, MODE_PAGE, PAGE57, 0xa0a0, 0x011a, 0x00ff}, //温湿度
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x02, 0xc8, MODE_ALWA, 0x0000, 0xa0a3, 0x0310, 0x00ff}, //回风温湿度
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x02, 0xc8, MODE_PAGE, PAGE10, 0xaa00, 0x0326, 0x00ff},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x08, 0xc8, MODE_PAGE, PAGE10, 0xaa02, 0x0341, 0x00ff},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PAGE, PAGE10, 0xaa0a, 0x036f, 0x00ff},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PAGE, PAGE10, 0xaa0b, 0x0377, 0x00ff},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PAGE, PAGE10, 0xaa0c, 0x0365, 0x00ff},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PAGE, PAGE10, 0xaa0d, 0x036c, 0x00ff},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PAGE, PAGE10, 0xaa0e, 0x034f, 0x00ff},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PAGE, PAGE10, 0xaa0f, 0x0369, 0x00ff},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PAGE, PAGE10, 0xcf23, 0x0110, 0x00ff},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PAGE, PAGE10, 0xcf27, 0x0197, 0x00ff},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x1f, 0xc8, MODE_PAGE, PAGE11, 0xab20, 0x038c, 0x00ff},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xac80, 0xac20, 0x023a, PAGE12}, //清楚当前告警
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x06, 0xc8, MODE_ALWA, 0x000d, 0xaea0, 0x0320, 0x00ff}, //告警
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PAGE, PAGE15, 0xcf27, 0x0197, 0x00ff},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x07, 0xc8, MODE_PANP, 0xb300, 0xb320, 0x011a, PAGE19},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xb380, 0xb320, 0x011a, PAGE19},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xb382, 0xb322, 0x011c, PAGE19},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xb383, 0xb323, 0x011d, PAGE19},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xb384, 0xb324, 0x011e, PAGE19},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xb385, 0xb325, 0x011f, PAGE19},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xb386, 0xb326, 0x0120, PAGE19},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x0b, 0xc8, MODE_PANP, 0xb400, 0xb420, 0x0158, PAGE20},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PANP, 0xb400, 0xb42b, 0x013b, PAGE20},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PANP, 0xb400, 0xb42c, 0x019e, PAGE20},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xb480, 0xb420, 0x0158, PAGE20},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xb481, 0xb421, 0x0159, PAGE20},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xb483, 0xb423, 0x015b, PAGE20},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xb484, 0xb424, 0x015c, PAGE20},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xb485, 0xb425, 0x015d, PAGE20},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xb486, 0xb426, 0x015e, PAGE20},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xb487, 0xb427, 0x015f, PAGE20},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xb488, 0xb428, 0x0160, PAGE20},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xb48a, 0xb42a, 0x0162, PAGE20},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xb48b, 0xb42b, 0x013b, PAGE20},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xb48c, 0xb42c, 0x019e, PAGE20},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x06, 0xc8, MODE_PANP, 0xb700, 0xb720, 0x013b, PAGE23},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x05, 0xc8, MODE_PANP, 0xb700, 0xb726, 0x0143, PAGE23},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xb780, 0xb720, 0x013b, PAGE23},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xb784, 0xb724, 0x013f, PAGE23},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xb785, 0xb725, 0x0140, PAGE23},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xb786, 0xb726, 0x0143, PAGE23},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xb787, 0xb727, 0x0144, PAGE23},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xb788, 0xb728, 0x0145, PAGE23},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xb789, 0xb729, 0x0146, PAGE23},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xb78a, 0xb72a, 0x0147, PAGE23},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PANP, 0xb800, 0xb820, 0x016a, PAGE24},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x03, 0xc8, MODE_PANP, 0xb800, 0xb821, 0x016d, PAGE24},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PANP, 0xb800, 0xb824, 0x0172, PAGE24},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PANP, 0xb800, 0xb825, 0x0174, PAGE24},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PANP, 0xb800, 0xb826, 0x017b, PAGE24},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PANP, 0xb800, 0xb827, 0x022a, PAGE24},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xb880, 0xb820, 0x016a, PAGE24},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xb881, 0xb821, 0x016d, PAGE24},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xb882, 0xb822, 0x016e, PAGE24},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xb883, 0xb823, 0x016f, PAGE24},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xb884, 0xb824, 0x0172, PAGE24},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xb885, 0xb825, 0x0174, PAGE24},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xb886, 0xb826, 0x017b, PAGE24},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xb887, 0xb827, 0x022a, PAGE24},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PANP, 0xbb00, 0xbb20, 0x01b0, PAGE27},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x02, 0xc8, MODE_PANP, 0xbb00, 0xbb21, 0x01b2, PAGE27},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x02, 0xc8, MODE_PANP, 0xbb00, 0xbb23, 0x01b5, PAGE27},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x02, 0xc8, MODE_PANP, 0xbb00, 0xbb25, 0x01b8, PAGE27},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PANP, 0xbb00, 0xbb27, 0x01bb, PAGE27},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x02, 0xc8, MODE_PANP, 0xbb00, 0xbb28, 0x01d1, PAGE27},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x02, 0xc8, MODE_PANP, 0xbb00, 0xbb2a, 0x01fb, PAGE27},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x02, 0xc8, MODE_PANP, 0xbb00, 0xbb2c, 0x0276, PAGE27},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xbb80, 0xbb20, 0x01b0, PAGE27},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xbb81, 0xbb21, 0x01b2, PAGE27},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xbb82, 0xbb22, 0x01b3, PAGE27},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xbb83, 0xbb23, 0x01b5, PAGE27},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xbb84, 0xbb24, 0x01b6, PAGE27},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xbb85, 0xbb25, 0x01b8, PAGE27},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xbb86, 0xbb26, 0x01b9, PAGE27},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xbb87, 0xbb27, 0x01bb, PAGE27},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xbb88, 0xbb28, 0x01d1, PAGE27},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xbb89, 0xbb29, 0x01d2, PAGE27},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xbb8a, 0xbb2a, 0x01fb, PAGE27},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xbb8b, 0xbb2b, 0x01fc, PAGE27},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xbb8c, 0xbb2c, 0x0276, PAGE27},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xbb8d, 0xbb2d, 0x0277, PAGE27},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x02, 0xc8, MODE_PANP, 0xbc00, 0xbc20, 0x0232, PAGE28},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PANP, 0xbc00, 0xbc22, 0x0252, PAGE28},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PANP, 0xbc00, 0xbc23, 0x0255, PAGE28},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PANP, 0xbc00, 0xbc24, 0x0258, PAGE28},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PANP, 0xbc00, 0xbc25, 0x0261, PAGE28},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x02, 0xc8, MODE_PANP, 0xbc00, 0xbc26, 0x0271, PAGE28},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PANP, 0xbc00, 0xbc28, 0x0286, PAGE28},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xbc80, 0xbc20, 0x0232, PAGE28},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xbc81, 0xbc21, 0x0233, PAGE28},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xbc82, 0xbc22, 0x0252, PAGE28},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xbc83, 0xbc23, 0x0255, PAGE28},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xbc84, 0xbc24, 0x0258, PAGE28},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xbc85, 0xbc25, 0x0261, PAGE28},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xbc86, 0xbc26, 0x0271, PAGE28},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xbc87, 0xbc27, 0x0272, PAGE28},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xbc88, 0xbc28, 0x0286, PAGE28},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x02, 0xc8, MODE_PANP, 0xbf00, 0xbf20, 0x0102, PAGE31},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PANP, 0xbf00, 0xbf22, 0x0107, PAGE31},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PANP, 0xbf00, 0xbf23, 0x01a5, PAGE31},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xbf80, 0xbf20, 0x0102, PAGE31},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xbf81, 0xbf21, 0x0103, PAGE31},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xbf82, 0xbf22, 0x0107, PAGE31},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xbf83, 0xbf23, 0x01a5, PAGE31},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x08, 0xc8, MODE_PANP, 0xc200, 0xc220, 0x0182, PAGE34},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PANP, 0xc200, 0xc228, 0x018c, PAGE34},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PANP, 0xc200, 0xc229, 0x0198, PAGE34},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xc280, 0xc220, 0x0182, PAGE34},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xc281, 0xc221, 0x0183, PAGE34},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xc282, 0xc222, 0x0184, PAGE34},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xc284, 0xc224, 0x0186, PAGE34},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xc285, 0xc225, 0x0187, PAGE34},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xc286, 0xc226, 0x0188, PAGE34},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xc287, 0xc227, 0x0189, PAGE34},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xc288, 0xc228, 0x018c, PAGE34},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xc289, 0xc229, 0x0198, PAGE34},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xc580, 0xc520, 0x023b, PAGE15}, // reset factory
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PAGE, PAGE38, 0xcf27, 0x0197, 0x00ff},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PANP, 0xc700, 0xc720, 0x01a6, PAGE39},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PANP, 0xc700, 0xc721, 0x01a9, PAGE39},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PANP, 0xc700, 0xc725, 0x01ab, PAGE39},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PANP, 0xc700, 0xc726, 0x02d9, PAGE39},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PANP, 0xc700, 0xc727, 0x02d8, PAGE39},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x02, 0xc8, MODE_PANP, 0xc700, 0xc728, 0x02ce, PAGE39},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PAGE, PAGE39, 0xa025, 0x0104, 0x00ff}, //诊断模式
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xc780, 0xc720, 0x01a6, PAGE39},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xc781, 0xc721, 0x01a9, PAGE39},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xc782, 0xc722, 0x01a8, PAGE39},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PAGE, PAGE39, 0xc723, 0x0375, 0x00ff},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PAGE, PAGE39, 0xc724, 0x0377, 0x00ff},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xc785, 0xc725, 0x01ab, PAGE39},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xc786, 0xc726, 0x02d9, PAGE39},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xc787, 0xc727, 0x02d8, PAGE39},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xc788, 0xc728, 0x02ce, PAGE39},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xc789, 0xc729, 0x02cf, PAGE39},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PANP, 0xc800, 0xc820, 0x0127, PAGE40},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PANP, 0xc800, 0xc821, 0x0129, PAGE40},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x04, 0xc8, MODE_PANP, 0xc800, 0xc822, 0x012d, PAGE40},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PANP, 0xc800, 0xc826, 0x0190, PAGE40},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PANP, 0xc800, 0xc827, 0x0194, PAGE40},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xc880, 0xc820, 0x0127, PAGE40},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xc881, 0xc821, 0x0129, PAGE40},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xc882, 0xc822, 0x012d, PAGE40},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xc883, 0xc823, 0x012e, PAGE40},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xc884, 0xc824, 0x012f, PAGE40},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xc885, 0xc825, 0x0130, PAGE40},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xc886, 0xc826, 0x0190, PAGE40},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xc887, 0xc827, 0x0194, PAGE40},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x0e, 0xc8, MODE_PAGE, PAGE41, 0xc9b0, 0x0378, 0x00ff},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xc990, 0xc930, 0x023c, PAGE41},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x03, 0xc8, MODE_PANP, 0xcc00, 0xcca0, 0x0302, PAGE44},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x04, 0xc8, MODE_PANP, 0xcf00, 0xcf20, 0x010d, PAGE47},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PANP, 0xcf00, 0xcf24, 0x0141, PAGE47},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PANP, 0xcf00, 0xcf25, 0x015a, PAGE47},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PANP, 0xcf00, 0xcf26, 0x02d0, PAGE47},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PANP, 0xcf00, 0xcf27, 0x0197, PAGE47},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xcf80, 0xcf20, 0x010d, PAGE47},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xcf81, 0xcf21, 0x010e, PAGE47},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xcf82, 0xcf22, 0x010f, PAGE47},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xcf83, 0xcf23, 0x0110, PAGE47},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xcf84, 0xcf24, 0x0141, PAGE47},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xcf85, 0xcf25, 0x015a, PAGE47},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xcf86, 0xcf26, 0x02d0, PAGE47},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xcf87, 0xcf27, 0x0197, PAGE47},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xcf8a, 0xcf2a, 0x023b, PAGE47},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x0A, 0xc8, MODE_PAGE, PAGE52, 0xd420, 0x039c, 0x00ff},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x02, 0xc8, MODE_PAGE, PAGE52, 0xd42a, 0x03a8, 0x00ff},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x07, 0xc8, MODE_PANP, 0xd500, 0xd520, 0x011a, PAGE53},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xd580, 0xd520, 0x011a, PAGE53},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xd582, 0xd522, 0x011c, PAGE53},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xd583, 0xd523, 0x011d, PAGE53},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xd584, 0xd524, 0x011e, PAGE53},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xd585, 0xd525, 0x011f, PAGE53},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xd586, 0xd526, 0x0120, PAGE53},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PANP, 0xd600, 0xd620, 0x0108, PAGE54},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x02, 0xc8, MODE_PANP, 0xd600, 0xd621, 0x010a, PAGE54},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PANP, 0xd600, 0xd623, 0x01a7, PAGE54},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xd680, 0xd620, 0x0108, PAGE54},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xd681, 0xd621, 0x010a, PAGE54},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xd682, 0xd622, 0x010b, PAGE54},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xd683, 0xd623, 0x01a7, PAGE54},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PANP, 0xd800, 0xd820, 0x01f5, PAGE56},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PANP, 0xd800, 0xd821, 0x0207, PAGE56},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x0c, 0xc8, MODE_PANP, 0xd800, 0xd822, 0x01bc, PAGE56},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PANP, 0xd800, 0xd82e, 0x027d, PAGE56},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PANP, 0xd800, 0xd82f, 0x027f, PAGE56},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PANP, 0xd800, 0xd830, 0x020f, PAGE56},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xd880, 0xd820, 0x01f5, PAGE56},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xd881, 0xd821, 0x0207, PAGE56},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xd882, 0xd822, 0x01bc, PAGE56},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xd883, 0xd823, 0x01bd, PAGE56},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xd884, 0xd824, 0x01be, PAGE56},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xd885, 0xd825, 0x01bf, PAGE56},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xd886, 0xd826, 0x01c0, PAGE56},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xd887, 0xd827, 0x01c1, PAGE56},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xd888, 0xd828, 0x01c2, PAGE56},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xd889, 0xd829, 0x01c3, PAGE56},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xd88a, 0xd82a, 0x01c4, PAGE56},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xd88b, 0xd82b, 0x01c5, PAGE56},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xd88c, 0xd82c, 0x01c6, PAGE56},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xd88d, 0xd82d, 0x01c7, PAGE56},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xd88e, 0xd82e, 0x027d, PAGE56},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xd88f, 0xd82f, 0x027f, PAGE56},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xd890, 0xd830, 0x020f, PAGE56},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PAGE, PAGE58, 0xda20, 0x03cd, 0x00ff},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PAGE, PAGE58, 0xda21, 0x03cf, 0x00ff},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x02, 0xc8, MODE_PAGE, PAGE58, 0xda22, 0x03d1, 0x00ff},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PAGE, PAGE58, 0xda24, 0x03d5, 0x00ff},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PAGE, PAGE58, 0xda25, 0x03d7, 0x00ff},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PAGE, PAGE58, 0xda26, 0x03d9, 0x00ff},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PAGE, PAGE58, 0xda27, 0x03db, 0x00ff},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PAGE, PAGE58, 0xda28, 0x03dd, 0x00ff},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PANP, 0xdb00, 0xdb20, 0x019a, PAGE59},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PANP, 0xdb00, 0xdb21, 0x02d3, PAGE59},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x02, 0xc8, MODE_PANP, 0xdb00, 0xdb22, 0x02d5, PAGE59},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x02, 0xc8, MODE_PANP, 0xdb00, 0xdb24, 0x02d8, PAGE59},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x05, 0xc8, MODE_PANP, 0xdb00, 0xdb26, 0x02db, PAGE59},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PANP, 0xdb00, 0xdb2b, 0x0106, PAGE59},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PANP, 0xdb00, 0xdb2c, 0x02e1, PAGE59},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x02, 0xc8, MODE_PANP, 0xdb00, 0xdb2d, 0x02e4, PAGE59},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x02, 0xc8, MODE_PANP, 0xdb00, 0xdb2f, 0x0178, PAGE59},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xdb80, 0xdb20, 0x019a, PAGE59},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xdb81, 0xdb21, 0x02d3, PAGE59},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xdb82, 0xdb22, 0x02d5, PAGE59},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xdb83, 0xdb23, 0x02d6, PAGE59},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xdb84, 0xdb24, 0x02d8, PAGE59},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xdb85, 0xdb25, 0x02d9, PAGE59},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xdb86, 0xdb26, 0x02db, PAGE59},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xdb87, 0xdb27, 0x02dc, PAGE59},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xdb88, 0xdb28, 0x02dd, PAGE59},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xdb89, 0xdb29, 0x02de, PAGE59},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xdb8a, 0xdb2a, 0x02df, PAGE59},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xdb8b, 0xdb2b, 0x0106, PAGE59},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xdb8c, 0xdb2c, 0x02e1, PAGE59},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xdb8d, 0xdb2d, 0x02e4, PAGE59},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xdb8e, 0xdb2e, 0x02e5, PAGE59},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xdb8f, 0xdb2f, 0x0178, PAGE59},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xdb90, 0xdb30, 0x0179, PAGE59},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PANP, 0xdd00, 0xdd20, 0x019b, PAGE61},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x04, 0xc8, MODE_PANP, 0xdd00, 0xdd21, 0x01e7, PAGE61},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x02, 0xc8, MODE_PANP, 0xdd00, 0xdd25, 0x01ed, PAGE61},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xdd80, 0xdd20, 0x019b, PAGE61},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xdd81, 0xdd21, 0x01e7, PAGE61},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xdd82, 0xdd22, 0x01e8, PAGE61},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xdd83, 0xdd23, 0x01e9, PAGE61},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xdd84, 0xdd24, 0x01ea, PAGE61},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xdd85, 0xdd25, 0x01ed, PAGE61},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xdd86, 0xdd26, 0x01ee, PAGE61},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PAGE, PAGE62, 0xde20, 0x0375, 0x00ff},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x03, 0xc8, MODE_PANP, 0xdf00, 0xdf20, 0x0237, PAGE63},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x06, 0xc8, MODE_PANP, 0xdf00, 0xdf23, 0x029e, PAGE63},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x03, 0xc8, MODE_PANP, 0xdf00, 0xdf29, 0x0201, PAGE63},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xdf80, 0xdf20, 0x0237, PAGE63},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xdf81, 0xdf21, 0x0238, PAGE63},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xdf82, 0xdf22, 0x0239, PAGE63},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xdf83, 0xdf23, 0x029e, PAGE63},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xdf84, 0xdf24, 0x029f, PAGE63},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xdf85, 0xdf25, 0x02a0, PAGE63},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xdf86, 0xdf26, 0x02a1, PAGE63},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xdf87, 0xdf27, 0x02a2, PAGE63},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xdf88, 0xdf28, 0x02a3, PAGE63},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xdf89, 0xdf29, 0x0201, PAGE63},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xdf8a, 0xdf2a, 0x0202, PAGE63},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xdf8b, 0xdf2b, 0x0203, PAGE63},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PAGE, PAGE64, 0xe022, 0x03be, 0x00ff},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PAGE, PAGE64, 0xe023, 0x03c9, 0x00ff},
    {BUS_EN, SLAVE_ID, BUS_FUN_10H, 0x02, 0xc8, MODE_PANP, 0xe080, 0xe020, 0x0000, PAGE64},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_ALWA, 0x0000, 0xe000, 0x03cc, 0x00ff}, // lock state
    {BUS_EN, SLAVE_ID, BUS_FUN_10H, 0x02, 0xc8, MODE_PANP, 0xe080, 0xe020, 0x0000, PAGE65},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PANP, 0xc700, 0xc720, 0x01a6, PAGE67},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PANP, 0xc700, 0xc721, 0x01a9, PAGE67},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PANP, 0xc700, 0xc725, 0x01ab, PAGE67},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PANP, 0xc700, 0xc726, 0x02d9, PAGE67},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PANP, 0xc700, 0xc727, 0x02d8, PAGE67},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x02, 0xc8, MODE_PANP, 0xc700, 0xc728, 0x02ce, PAGE67},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PAGE, PAGE67, 0xa025, 0x0104, 0x00ff}, //诊断模式
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xc780, 0xc720, 0x01a6, PAGE67},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xc781, 0xc721, 0x01a9, PAGE67},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xc782, 0xc722, 0x01a8, PAGE67},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PAGE, PAGE67, 0xc723, 0x0375, 0x00ff},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PAGE, PAGE67, 0xc724, 0x0377, 0x00ff},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xc785, 0xc725, 0x01ab, PAGE67},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xc786, 0xc726, 0x02d9, PAGE67},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xc787, 0xc727, 0x02d8, PAGE67},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xc788, 0xc728, 0x02ce, PAGE67},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xc789, 0xc729, 0x02cf, PAGE67},
};
modbosCmd_t modbusCmdNow = {0};
u16         CmdIndex     = 0;

const dataCheckCmd_t dataCheckLib[] = {
  // en     page  data    back   flag
    {BUS_EN, PAGE19, 0xb320, 0xb350, 0xb380}, //
    {BUS_EN, PAGE19, 0xb322, 0xb352, 0xb382}, //
    {BUS_EN, PAGE19, 0xb323, 0xb353, 0xb383}, //
    {BUS_EN, PAGE19, 0xb324, 0xb354, 0xb384}, //
    {BUS_EN, PAGE19, 0xb325, 0xb355, 0xb385}, //
    {BUS_EN, PAGE19, 0xb326, 0xb356, 0xb386}, //
    {BUS_EN, PAGE20, 0xb420, 0xb450, 0xb480}, //
    {BUS_EN, PAGE20, 0xb421, 0xb451, 0xb481}, //
    {BUS_EN, PAGE20, 0xb423, 0xb453, 0xb483}, //
    {BUS_EN, PAGE20, 0xb424, 0xb454, 0xb484}, //
    {BUS_EN, PAGE20, 0xb425, 0xb455, 0xb485}, //
    {BUS_EN, PAGE20, 0xb426, 0xb456, 0xb486}, //
    {BUS_EN, PAGE20, 0xb427, 0xb457, 0xb487}, //
    {BUS_EN, PAGE20, 0xb428, 0xb458, 0xb488}, //
    {BUS_EN, PAGE20, 0xb42a, 0xb45a, 0xb48a}, //
    {BUS_EN, PAGE20, 0xb42b, 0xb45b, 0xb48b}, //
    {BUS_EN, PAGE20, 0xb42c, 0xb45c, 0xb48c}, //
    {BUS_EN, PAGE23, 0xb720, 0xb750, 0xb780}, //
    {BUS_EN, PAGE23, 0xb724, 0xb754, 0xb784}, //
    {BUS_EN, PAGE23, 0xb725, 0xb755, 0xb785}, //
    {BUS_EN, PAGE23, 0xb726, 0xb756, 0xb786}, //
    {BUS_EN, PAGE23, 0xb727, 0xb757, 0xb787}, //
    {BUS_EN, PAGE23, 0xb728, 0xb758, 0xb788}, //
    {BUS_EN, PAGE23, 0xb729, 0xb759, 0xb789}, //
    {BUS_EN, PAGE23, 0xb72a, 0xb75a, 0xb78a}, //
    {BUS_EN, PAGE24, 0xb820, 0xb850, 0xb880}, //
    {BUS_EN, PAGE24, 0xb821, 0xb851, 0xb881}, //
    {BUS_EN, PAGE24, 0xb822, 0xb852, 0xb882}, //
    {BUS_EN, PAGE24, 0xb823, 0xb853, 0xb883}, //
    {BUS_EN, PAGE24, 0xb824, 0xb854, 0xb884}, //
    {BUS_EN, PAGE24, 0xb825, 0xb855, 0xb885}, //
    {BUS_EN, PAGE24, 0xb826, 0xb856, 0xb886}, //
    {BUS_EN, PAGE24, 0xb827, 0xb857, 0xb887}, //
    {BUS_EN, PAGE27, 0xbb20, 0xbb50, 0xbb80}, //
    {BUS_EN, PAGE27, 0xbb21, 0xbb51, 0xbb81}, //
    {BUS_EN, PAGE27, 0xbb22, 0xbb52, 0xbb82}, //
    {BUS_EN, PAGE27, 0xbb23, 0xbb53, 0xbb83}, //
    {BUS_EN, PAGE27, 0xbb24, 0xbb54, 0xbb84}, //
    {BUS_EN, PAGE27, 0xbb25, 0xbb55, 0xbb85}, //
    {BUS_EN, PAGE27, 0xbb26, 0xbb56, 0xbb86}, //
    {BUS_EN, PAGE27, 0xbb27, 0xbb57, 0xbb87}, //
    {BUS_EN, PAGE27, 0xbb28, 0xbb58, 0xbb88}, //
    {BUS_EN, PAGE27, 0xbb29, 0xbb59, 0xbb89}, //
    {BUS_EN, PAGE27, 0xbb2a, 0xbb5a, 0xbb8a}, //
    {BUS_EN, PAGE27, 0xbb2b, 0xbb5b, 0xbb8b}, //
    {BUS_EN, PAGE27, 0xbb2c, 0xbb5c, 0xbb8c}, //
    {BUS_EN, PAGE27, 0xbb2d, 0xbb5d, 0xbb8d}, //
    {BUS_EN, PAGE28, 0xbc20, 0xbc50, 0xbc80}, //
    {BUS_EN, PAGE28, 0xbc21, 0xbc51, 0xbc81}, //
    {BUS_EN, PAGE28, 0xbc22, 0xbc52, 0xbc82}, //
    {BUS_EN, PAGE28, 0xbc23, 0xbc53, 0xbc83}, //
    {BUS_EN, PAGE28, 0xbc24, 0xbc54, 0xbc84}, //
    {BUS_EN, PAGE28, 0xbc25, 0xbc55, 0xbc85}, //
    {BUS_EN, PAGE28, 0xbc26, 0xbc56, 0xbc86}, //
    {BUS_EN, PAGE28, 0xbc27, 0xbc57, 0xbc87}, //
    {BUS_EN, PAGE28, 0xbc28, 0xbc58, 0xbc88}, //
    {BUS_EN, PAGE31, 0xbf20, 0xbf50, 0xbf80}, //
    {BUS_EN, PAGE31, 0xbf21, 0xbf51, 0xbf81}, //
    {BUS_EN, PAGE31, 0xbf22, 0xbf52, 0xbf82}, //
    {BUS_EN, PAGE31, 0xbf23, 0xbf53, 0xbf83}, //
    {BUS_EN, PAGE34, 0xc220, 0xc250, 0xc280}, //
    {BUS_EN, PAGE34, 0xc221, 0xc251, 0xc281}, //
    {BUS_EN, PAGE34, 0xc222, 0xc252, 0xc282}, //
    {BUS_EN, PAGE34, 0xc224, 0xc254, 0xc284}, //
    {BUS_EN, PAGE34, 0xc225, 0xc255, 0xc285}, //
    {BUS_EN, PAGE34, 0xc226, 0xc256, 0xc286}, //
    {BUS_EN, PAGE34, 0xc227, 0xc257, 0xc287}, //
    {BUS_EN, PAGE34, 0xc228, 0xc258, 0xc288}, //
    {BUS_EN, PAGE34, 0xc229, 0xc259, 0xc289}, //
    {BUS_EN, PAGE39, 0xc720, 0xc750, 0xc780}, //
    {BUS_EN, PAGE39, 0xc721, 0xc751, 0xc781}, //
    {BUS_EN, PAGE39, 0xc725, 0xc755, 0xc785}, //
    {BUS_EN, PAGE39, 0xc726, 0xc756, 0xc786}, //
    {BUS_EN, PAGE39, 0xc727, 0xc757, 0xc787}, //
    {BUS_EN, PAGE39, 0xc728, 0xc758, 0xc788}, //
    {BUS_EN, PAGE39, 0xc729, 0xc759, 0xc789}, //
    {BUS_EN, PAGE40, 0xc820, 0xc850, 0xc880}, //
    {BUS_EN, PAGE40, 0xc821, 0xc851, 0xc881}, //
    {BUS_EN, PAGE40, 0xc822, 0xc852, 0xc882}, //
    {BUS_EN, PAGE40, 0xc823, 0xc853, 0xc883}, //
    {BUS_EN, PAGE40, 0xc824, 0xc854, 0xc884}, //
    {BUS_EN, PAGE40, 0xc825, 0xc855, 0xc885}, //
    {BUS_EN, PAGE40, 0xc826, 0xc856, 0xc886}, //
    {BUS_EN, PAGE40, 0xc827, 0xc857, 0xc887}, //
    {BUS_EN, PAGE47, 0xcf20, 0xcf50, 0xcf80}, //
    {BUS_EN, PAGE47, 0xcf21, 0xcf51, 0xcf81}, //
    {BUS_EN, PAGE47, 0xcf22, 0xcf52, 0xcf82}, //
    {BUS_EN, PAGE47, 0xcf23, 0xcf53, 0xcf83}, //
    {BUS_EN, PAGE47, 0xcf24, 0xcf54, 0xcf84}, //
    {BUS_EN, PAGE47, 0xcf25, 0xcf55, 0xcf85}, //
    {BUS_EN, PAGE47, 0xcf26, 0xcf56, 0xcf86}, //
    {BUS_EN, PAGE47, 0xcf27, 0xcf57, 0xcf87}, //
    {BUS_EN, PAGE53, 0xd520, 0xd550, 0xd580}, //
    {BUS_EN, PAGE53, 0xd522, 0xd552, 0xd582}, //
    {BUS_EN, PAGE53, 0xd523, 0xd553, 0xd583}, //
    {BUS_EN, PAGE53, 0xd524, 0xd554, 0xd584}, //
    {BUS_EN, PAGE53, 0xd525, 0xd555, 0xd585}, //
    {BUS_EN, PAGE53, 0xd526, 0xd556, 0xd586}, //
    {BUS_EN, PAGE54, 0xd620, 0xd650, 0xd680}, //
    {BUS_EN, PAGE54, 0xd621, 0xd651, 0xd681}, //
    {BUS_EN, PAGE54, 0xd622, 0xd652, 0xd682}, //
    {BUS_EN, PAGE54, 0xd623, 0xd653, 0xd683}, //
    {BUS_EN, PAGE56, 0xd820, 0xd850, 0xd880}, //
    {BUS_EN, PAGE56, 0xd821, 0xd851, 0xd881}, //
    {BUS_EN, PAGE56, 0xd822, 0xd852, 0xd882}, //
    {BUS_EN, PAGE56, 0xd823, 0xd853, 0xd883}, //
    {BUS_EN, PAGE56, 0xd824, 0xd854, 0xd884}, //
    {BUS_EN, PAGE56, 0xd825, 0xd855, 0xd885}, //
    {BUS_EN, PAGE56, 0xd826, 0xd856, 0xd886}, //
    {BUS_EN, PAGE56, 0xd827, 0xd857, 0xd887}, //
    {BUS_EN, PAGE56, 0xd828, 0xd858, 0xd888}, //
    {BUS_EN, PAGE56, 0xd829, 0xd859, 0xd889}, //
    {BUS_EN, PAGE56, 0xd82a, 0xd85a, 0xd88a}, //
    {BUS_EN, PAGE56, 0xd82b, 0xd85b, 0xd88b}, //
    {BUS_EN, PAGE56, 0xd82c, 0xd85c, 0xd88c}, //
    {BUS_EN, PAGE56, 0xd82d, 0xd85d, 0xd88d}, //
    {BUS_EN, PAGE56, 0xd82e, 0xd85e, 0xd88e}, //
    {BUS_EN, PAGE56, 0xd82f, 0xd85f, 0xd88f}, //
    {BUS_EN, PAGE56, 0xd830, 0xd860, 0xd890}, //
    {BUS_EN, PAGE59, 0xdb20, 0xdb50, 0xdb80}, //
    {BUS_EN, PAGE59, 0xdb21, 0xdb51, 0xdb81}, //
    {BUS_EN, PAGE59, 0xdb22, 0xdb52, 0xdb82}, //
    {BUS_EN, PAGE59, 0xdb23, 0xdb53, 0xdb83}, //
    {BUS_EN, PAGE59, 0xdb24, 0xdb54, 0xdb84}, //
    {BUS_EN, PAGE59, 0xdb25, 0xdb55, 0xdb85}, //
    {BUS_EN, PAGE59, 0xdb26, 0xdb56, 0xdb86}, //
    {BUS_EN, PAGE59, 0xdb27, 0xdb57, 0xdb87}, //
    {BUS_EN, PAGE59, 0xdb28, 0xdb58, 0xdb88}, //
    {BUS_EN, PAGE59, 0xdb29, 0xdb59, 0xdb89}, //
    {BUS_EN, PAGE59, 0xdb2a, 0xdb5a, 0xdb8a}, //
    {BUS_EN, PAGE59, 0xdb2b, 0xdb5b, 0xdb8b}, //
    {BUS_EN, PAGE59, 0xdb2c, 0xdb5c, 0xdb8c}, //
    {BUS_EN, PAGE59, 0xdb2d, 0xdb5d, 0xdb8d}, //
    {BUS_EN, PAGE59, 0xdb2e, 0xdb5e, 0xdb8e}, //
    {BUS_EN, PAGE59, 0xdb2f, 0xdb5f, 0xdb8f}, //
    {BUS_EN, PAGE59, 0xdb30, 0xdb60, 0xdb90}, //
    {BUS_EN, PAGE61, 0xdd20, 0xdd50, 0xdd80}, //
    {BUS_EN, PAGE61, 0xdd21, 0xdd51, 0xdd81}, //
    {BUS_EN, PAGE61, 0xdd22, 0xdd52, 0xdd82}, //
    {BUS_EN, PAGE61, 0xdd23, 0xdd53, 0xdd83}, //
    {BUS_EN, PAGE61, 0xdd24, 0xdd54, 0xdd84}, //
    {BUS_EN, PAGE61, 0xdd25, 0xdd55, 0xdd85}, //
    {BUS_EN, PAGE61, 0xdd26, 0xdd56, 0xdd86}, //
    {BUS_EN, PAGE63, 0xdf20, 0xdf50, 0xdf80}, //
    {BUS_EN, PAGE63, 0xdf21, 0xdf51, 0xdf81}, //
    {BUS_EN, PAGE63, 0xdf22, 0xdf52, 0xdf82}, //
    {BUS_EN, PAGE63, 0xdf23, 0xdf53, 0xdf83}, //
    {BUS_EN, PAGE63, 0xdf24, 0xdf54, 0xdf84}, //
    {BUS_EN, PAGE63, 0xdf25, 0xdf55, 0xdf85}, //
    {BUS_EN, PAGE63, 0xdf26, 0xdf56, 0xdf86}, //
    {BUS_EN, PAGE63, 0xdf27, 0xdf57, 0xdf87}, //
    {BUS_EN, PAGE63, 0xdf28, 0xdf58, 0xdf88}, //
    {BUS_EN, PAGE63, 0xdf29, 0xdf59, 0xdf89}, //
    {BUS_EN, PAGE63, 0xdf2a, 0xdf5a, 0xdf8a}, //
    {BUS_EN, PAGE63, 0xdf2b, 0xdf5b, 0xdf8b}, //
    {BUS_EN, PAGE67, 0xc720, 0xc750, 0xc780}, //
    {BUS_EN, PAGE67, 0xc721, 0xc751, 0xc781}, //
    {BUS_EN, PAGE67, 0xc725, 0xc755, 0xc785}, //
    {BUS_EN, PAGE67, 0xc726, 0xc756, 0xc786}, //
    {BUS_EN, PAGE67, 0xc727, 0xc757, 0xc787}, //
    {BUS_EN, PAGE67, 0xc728, 0xc758, 0xc788}, //
    {BUS_EN, PAGE67, 0xc729, 0xc759, 0xc789}, //
};

_TKS_FLAGA_type modbusFlag = {0};
/******************************************************************************
          版权所有 (C), 2020，DFX，Write by Food(181 1266 9427)
 ******************************************************************************
modbus 命令解析处理程序，实现：
1. 03H的回送命令解析
2. 06H的回送命令解析，如果回送命令正确，则停止UI的触发发送命令
******************************************************************************/

void modbus_process_command(u8 *pstr, u16 strlen)
{
    u16 num;
    u16 crc_data;
    u16 len;

    if (strlen < 5) {
        return;
    }
    num = 0;
    do {
        if ((*(pstr + num)) == SLAVE_ID) {
            switch (*(pstr + num + 1))  //判读下一个字节是modbus的哪个命令
            {
                case BUS_FUN_03H:
                    len = *(pstr + num + 2);
                    if ((len + num + 5) > strlen)  //长度超过最大长度
                    {
                        num = strlen;  //非modbus命令
                        break;
                    }
                    crc_data = crc16table(pstr + num, 3 + len);
                    // printf("num:%d,len:%d,crc data:%02X,%02X,", num, len, (u16)((crc_data >> 8) & 0xFF),(u16)(crc_data & 0xFF));
                    if ((*(pstr + num + len + 3) != ((crc_data >> 8) & 0xFF)) || (*(pstr + num + len + 4) != (crc_data & 0xFF))) {  // CRC
                        break;
                    }
                    WriteDGUS(modbusCmdNow.VPAddr, (pstr + num + 3), len);
                    memset(&modbusCmdNow, 0, sizeof(modbosCmd_t));
                    num       = len + 5;
                    cmdRxFlag = 1;
                    break;
                case BUS_FUN_06H:
                    if ((num + 8) > strlen) {
                        num = strlen;  //非modbus命令
                        break;
                    }
                    crc_data = crc16table(pstr + num, 6);
                    if ((*(pstr + num + 6) != ((crc_data >> 8) & 0xFF)) || (*(pstr + num + 7) != (crc_data & 0xFF))) {  // CRC
                        break;
                    }
                    num += 8;
                    memset(&modbusCmdNow, 0, sizeof(modbosCmd_t));
                    cmdRxFlag = 1;
                    break;
                case BUS_FUN_10H:
                    if ((num + 8) > strlen) {
                        num = strlen;  //非modbus命令
                        break;
                    }
                    crc_data = crc16table(pstr + num, 6);
                    if ((*(pstr + num + 6) != ((crc_data >> 8) & 0xFF)) || (*(pstr + num + 7) != (crc_data & 0xFF))) {  // CRC
                        break;
                    }
                    num += 8;
                    memset(&modbusCmdNow, 0, sizeof(modbosCmd_t));
                    cmdRxFlag = 1;
                    break;
                default:
                    break;
            }
        }
        num++;
    } while (num < (strlen - 5));  // addre,command,data,crch,crcl,至少需要有5个字节
}
/******************************************************************************
          版权所有 (C), 2020，DFX，Write by Food(181 1266 9427)
 ******************************************************************************
modbus 发送和接收任务处理程序，实现：
1. 监控串口接收，当判断接收结束后，调用处理函数，
2. 监控UI的触发命令，当有检测到发送命令时，发送modbus写命令
3. 每隔1秒钟触发一次查询modbus寄存器状态的命令
******************************************************************************/
void Modbus_Process_Task(void)
{
    modbosCmd_t *cmdTemp_t = NULL;
    if (modbus_rx_flag == 1)  //接收数据
    {
        if (modbus_rx_count > modbus_rx_count_before) {
            modbus_rx_count_before = modbus_rx_count;
            uart_rx_check_tick     = 0;
        } else if (modbus_rx_count == modbus_rx_count_before) {
            if (uart_rx_check_tick > 0) {
                if ((ModbusSysTick - uart_rx_check_tick) > RX_CHECK_TICK_TIME) {
                    modbus_process_command(modbus_rx_buf, modbus_rx_count);
                    Modbus_RX_Reset();
                }
            } else {
                uart_rx_check_tick = ModbusSysTick;
            }
        }
    }

    if (cmdTxFlag) {
        if ((cmdRxFlag) || ((ModbusSysTick - modbus_tx_process_tick) >= modbusCmdNow.timeout)) {
            if (cmdRxFlag) {
                CmdIndex++;
                goto processCMDLib;
            } else {
                cmdTxFlag = 0;
                return;
            }
        }
        return;
    }

    if ((ModbusSysTick - modbus_tx_process_tick) < MODBUS_SEND_TIME_PERIOD) {  //间隔固定时间后再处理UI的设置命令，
        return;
    }
processCMDLib:
    if (CmdIndex == 0)
        checkChange();
    modbus_tx_process_tick = 0;
    ModbusSysTick          = 0;
    cmdRxFlag              = 0;
    cmdTxFlag              = 0;
    Modbus_RX_Reset();
    getCmd(&CmdIndex);
    if (CmdIndex < CMD_NUMBER) {
        memcpy(&modbusCmdNow, &modbusCmdlib[CmdIndex], sizeof(modbosCmd_t));
        Modbus_RX_Reset();
        if (modbusCmdNow.funCode == BUS_FUN_03H) {
            Modbus_Read_Register03H(&modbusCmdNow);
            cmdTxFlag = 1;
        } else if (modbusCmdNow.funCode == BUS_FUN_06H) {
            u16 value;
            ReadDGUS(modbusCmdNow.VPAddr, (u8 *)(&value), 2);
            Modbus_Write_Register06H(&modbusCmdNow, value);
            cmdTxFlag = 1;
        } else if (modbusCmdNow.funCode == BUS_FUN_10H) {
            Modbus_Write_Register10H(&modbusCmdNow);
            cmdTxFlag = 1;
        }
        ModbusSysTick = 0;
    } else {
        CmdIndex = 0;
    }
}
// modbus 03H 读取寄存器
void Modbus_Read_Register03H(modbosCmd_t *CmdNow)
{
    u16 crc_data;
    u8  len = 0;
    u8  modbus_tx_buf[20];

    modbus_tx_buf[len++] = CmdNow->slaveID;
    modbus_tx_buf[len++] = BUS_FUN_03H;                      // command
    modbus_tx_buf[len++] = (CmdNow->slaveAddr >> 8) & 0xFF;  // register
    modbus_tx_buf[len++] = CmdNow->slaveAddr & 0xFF;
    modbus_tx_buf[len++] = (CmdNow->length >> 8) & 0xFF;  // register number
    modbus_tx_buf[len++] = CmdNow->length & 0xFF;
    crc_data             = crc16table(modbus_tx_buf, len);
    modbus_tx_buf[len++] = (crc_data >> 8) & 0xFF;
    modbus_tx_buf[len++] = crc_data & 0xFF;
#ifdef MDO_UART2
    Uart2SendStr(modbus_tx_buf, len);
#endif
#ifdef MDO_UART5
    Uart5SendStr(modbus_tx_buf, len);
#endif
}

// modbus 06H 发送
void Modbus_Write_Register06H(modbosCmd_t *CmdNow, u16 value)
{
    u16 crc_data;
    u8  len = 0;
    u8  modbus_tx_buf[20];

    modbus_tx_buf[len++] = CmdNow->slaveID;
    modbus_tx_buf[len++] = BUS_FUN_06H;                      // command
    modbus_tx_buf[len++] = (CmdNow->slaveAddr >> 8) & 0xFF;  // register
    modbus_tx_buf[len++] = CmdNow->slaveAddr & 0xFF;
    modbus_tx_buf[len++] = (value >> 8) & 0xFF;  // register value
    modbus_tx_buf[len++] = value & 0xFF;
    crc_data             = crc16table(modbus_tx_buf, len);
    modbus_tx_buf[len++] = (crc_data >> 8) & 0xFF;
    modbus_tx_buf[len++] = crc_data & 0xFF;
#ifdef MDO_UART2
    Uart2SendStr(modbus_tx_buf, len);
#endif
#ifdef MDO_UART5
    Uart5SendStr(modbus_tx_buf, len);
#endif
}  // modbus 06H 发送
void Modbus_Write_Register10H(modbosCmd_t *CmdNow)
{
    u16 crc_data;
    u8  len = 0;
    u8  modbus_tx_buf[64];

    modbus_tx_buf[len++] = CmdNow->slaveID;
    modbus_tx_buf[len++] = BUS_FUN_10H;                      // command
    modbus_tx_buf[len++] = (CmdNow->slaveAddr >> 8) & 0xFF;  // register
    modbus_tx_buf[len++] = CmdNow->slaveAddr & 0xFF;
    modbus_tx_buf[len++] = (CmdNow->length >> 8) & 0xFF;  // register number
    modbus_tx_buf[len++] = CmdNow->length & 0xFF;
    modbus_tx_buf[len++] = CmdNow->length * 2;
    ReadDGUS(modbusCmdNow.VPAddr, (u8 *)(&modbus_tx_buf[len]), CmdNow->length * 2);
    len += CmdNow->length * 2;
    crc_data             = crc16table(modbus_tx_buf, len);
    modbus_tx_buf[len++] = (crc_data >> 8) & 0xFF;
    modbus_tx_buf[len++] = crc_data & 0xFF;
#ifdef MDO_UART2
    Uart2SendStr(modbus_tx_buf, len);
#endif
#ifdef MDO_UART5
    Uart5SendStr(modbus_tx_buf, len);
#endif
}
//清除modbus RX的相关参数
void Modbus_RX_Reset(void)
{
    modbus_rx_count = 0;
    modbus_rx_flag  = 0;
    memset(modbus_rx_buf, '\0', UART_RX_BUF_MAX_LEN);
    modbus_rx_count_before = 0;
    uart_rx_check_tick     = 0;
}
//初始化modbus 相关参数
void Modbus_UART_Init(void)
{
    //	Modbus_TX_Reset();
    Modbus_RX_Reset();
    modbus_tx_process_tick = 0;  //初始化 0
}

void getCmd(u16 *index)
{
    u16 i;
    for (i = *index; i < CMD_NUMBER; i++) {
        if ((modbusCmdlib[i].modbusEn != BUS_EN) || (modbusCmdlib[i].length == 0)) {
            continue;
        }
        if (modbusCmdlib[i].mode == MODE_ALWA) {
            goto getCmdExit;
        } else if (modbusCmdlib[i].mode == MODE_PAGE) {
            if (picNow == modbusCmdlib[i].modePara) {
                goto getCmdExit;
            }
            continue;
        } else if (modbusCmdlib[i].mode == MODE_PARA) {
            u16 paraTemp;
            ReadDGUS(modbusCmdlib[i].modePara, (u8 *)(&paraTemp), 2);
            if ((paraTemp & 0xff) == 0x5a) {
                if (i < CMD_NUMBER - 1) {
                    if ((modbusCmdlib[i + 1].mode == MODE_PARA) && (modbusCmdlib[i].modePara == modbusCmdlib[i + 1].modePara)) {
                        goto getCmdExit;
                    }
                }
                paraTemp = 0;
                WriteDGUS(modbusCmdlib[i].modePara, (u8 *)(&paraTemp), 2);
                goto getCmdExit;
            }
            continue;
        } else if (modbusCmdlib[i].mode == MODE_PANP) {
            u16 paraTemp;
            if (modbusCmdlib[i].feedback != picNow) {
                continue;
            }
            ReadDGUS(modbusCmdlib[i].modePara, (u8 *)(&paraTemp), 2);
            if ((paraTemp & 0xff) == 0x5a) {
                if (i < CMD_NUMBER - 1) {
                    if ((modbusCmdlib[i + 1].mode == MODE_PANP) && (modbusCmdlib[i].modePara == modbusCmdlib[i + 1].modePara)) {
                        goto getCmdExit;
                    }
                }
                paraTemp = 0;
                WriteDGUS(modbusCmdlib[i].modePara, (u8 *)(&paraTemp), 2);
                goto getCmdExit;
            }
            continue;
        }
    }
getCmdExit:
    *index = i;
}

void checkChange(void)
{
    u16 cache[20] = {0};
    u16 i;
    for (i = 0; i < CHECK_NUMBER; i++) {
        if (dataCheckLib[i].page != picNow)
            continue;
        ReadDGUS(dataCheckLib[i].dataAddr, (u8 *)&cache[0], 2);
        ReadDGUS(dataCheckLib[i].backAddr, (u8 *)&cache[1], 2);
        if (cache[0] != cache[1]) {
            WriteDGUS(dataCheckLib[i].backAddr, (u8 *)&cache[0], 2);
            cache[2] = 0x5a;
            WriteDGUS(dataCheckLib[i].flagAddr, (u8 *)&cache[2], 2);
        }
    }
}

void forcedOutputHandle(void)
{
    u16 cache = 0;
    ReadDGUS(0xc7a0, (u8 *)&cache, 2);
    WriteDGUS(0xc722, (u8 *)&cache, 2);
    cache = 0x005a;
    WriteDGUS(0xc782, (u8 *)&cache, 2);
}
