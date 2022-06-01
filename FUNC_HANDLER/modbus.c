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
  //  en         id         fun      len  timeout   mod      modP     VP  slaveAddr feedback
    {BUS_EN, SLAVE_ID, BUS_FUN_10H, 0x03, 0xc8, MODE_PARA, 0x5015, 0x5016, 0x0000, 0x00ff}, // rtc
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x03, 0xc8, MODE_ALWA, 0x0000, 0xd920, 0x031c, 0x00ff}, //告警数
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PAGE, PAGE57, 0xd923, 0x0319, 0x00ff}, //位状态
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xd984, 0xd924, 0x0100, PAGE57}, // power switch
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PAGE, PAGE57, 0xd925, 0x0104, 0x00ff}, //诊断模式
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PARA, 0xd987, 0xd927, 0x0104, 0x00ff}, //诊断切换
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x03, 0xc8, MODE_PAGE, PAGE57, 0xd9a0, 0x011a, 0x00ff}, //温湿度设定值
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x02, 0xc8, MODE_ALWA, 0x0000, 0xd9a3, 0x0310, 0x00ff}, //回风温湿度
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x02, 0xc8, MODE_PAGE, PAGE10, 0xaa20, 0x0326, 0x00ff},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PAGE, PAGE10, 0xaa22, 0x0369, 0x00ff},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x02, 0xc8, MODE_PAGE, PAGE10, 0xaa23, 0x036b, 0x00ff},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PAGE, PAGE10, 0xaa25, 0x0377, 0x00ff},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x1f, 0xc8, MODE_PAGE, PAGE11, 0xab20, 0x0375, 0x00ff},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xac80, 0xac20, 0x023a, PAGE12}, //清楚当前告警
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x06, 0xc8, MODE_ALWA, 0x000d, 0xaea0, 0x0320, 0x00ff}, //告警
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xaf80, 0xaf20, 0x023b, PAGE15}, // reset factory
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PANP, 0xb300, 0xb320, 0x011a, PAGE19},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x02, 0xc8, MODE_PANP, 0xb300, 0xb321, 0x013b, PAGE19},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xb380, 0xb320, 0x011a, PAGE19},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xb381, 0xb321, 0x013b, PAGE19},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xb382, 0xb322, 0x013c, PAGE19},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x05, 0xc8, MODE_PANP, 0xb400, 0xb420, 0x015b, PAGE20},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xb480, 0xb420, 0x015b, PAGE20},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xb481, 0xb421, 0x015c, PAGE20},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xb482, 0xb422, 0x015d, PAGE20},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xb483, 0xb423, 0x015e, PAGE20},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xb484, 0xb424, 0x015f, PAGE20},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PANP, 0xb600, 0xb620, 0x02e4, PAGE22},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PANP, 0xb600, 0xb621, 0x02ea, PAGE22},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xb680, 0xb620, 0x02e4, PAGE22},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xb681, 0xb621, 0x02ea, PAGE22},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PANP, 0xb800, 0xb820, 0x010d, PAGE24},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PANP, 0xb800, 0xb821, 0x010f, PAGE24},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PANP, 0xb800, 0xb822, 0x015a, PAGE24},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xb880, 0xb820, 0x010d, PAGE24},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xb881, 0xb821, 0x010f, PAGE24},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xb882, 0xb822, 0x015a, PAGE24},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PANP, 0xb900, 0xb920, 0x0108, PAGE25},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x02, 0xc8, MODE_PANP, 0xb900, 0xb921, 0x010a, PAGE25},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PANP, 0xb900, 0xb923, 0x01a7, PAGE25},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xb980, 0xb920, 0x0108, PAGE25},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xb981, 0xb921, 0x010a, PAGE25},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xb982, 0xb922, 0x010b, PAGE25},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xb983, 0xb923, 0x01a7, PAGE25},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PANP, 0xbb00, 0xbb20, 0x0210, PAGE27},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PANP, 0xbb00, 0xbb21, 0x0213, PAGE27},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PANP, 0xbb00, 0xbb22, 0x0216, PAGE27},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x02, 0xc8, MODE_PANP, 0xbb00, 0xbb23, 0x0219, PAGE27},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x03, 0xc8, MODE_PANP, 0xbb00, 0xbb24, 0x027d, PAGE27},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x02, 0xc8, MODE_PANP, 0xbb00, 0xbb25, 0x0246, PAGE27},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PANP, 0xbb00, 0xbb27, 0x0277, PAGE27},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xbb80, 0xbb20, 0x0210, PAGE27},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xbb81, 0xbb21, 0x0213, PAGE27},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xbb82, 0xbb22, 0x0216, PAGE27},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xbb83, 0xbb23, 0x0219, PAGE27},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xbb84, 0xbb24, 0x027d, PAGE27},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xbb85, 0xbb25, 0x0246, PAGE27},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xbb86, 0xbb26, 0x0247, PAGE27},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xbb87, 0xbb27, 0x0277, PAGE27},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x02, 0xc8, MODE_PANP, 0xbf00, 0xbf20, 0x0102, PAGE31},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PANP, 0xbf00, 0xbf22, 0x0107, PAGE31},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PANP, 0xbf00, 0xbf23, 0x01a5, PAGE31},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xbf80, 0xbf20, 0x0102, PAGE31},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xbf81, 0xbf21, 0x0103, PAGE31},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xbf82, 0xbf22, 0x0107, PAGE31},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xbf83, 0xbf23, 0x01a5, PAGE31},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PANP, 0xc300, 0xc320, 0x02eb, PAGE35},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PANP, 0xc300, 0xc321, 0x01a9, PAGE35},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PAGE, PAGE35, 0xd925, 0x0104, 0x00ff}, //诊断模式
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xc380, 0xc320, 0x02eb, PAGE35},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xc381, 0xc321, 0x01a9, PAGE35},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xc382, 0xc322, 0x01a8, PAGE35},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PAGE, PAGE35, 0xc323, 0x0375, 0x00ff},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PAGE, PAGE35, 0xc324, 0x0377, 0x00ff},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x02, 0xc8, MODE_PANP, 0xc400, 0xc420, 0x012d, PAGE36},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PANP, 0xc400, 0xc422, 0x0129, PAGE36},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x02, 0xc8, MODE_PANP, 0xc400, 0xc423, 0x012b, PAGE36},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xc480, 0xc420, 0x012d, PAGE36},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xc481, 0xc421, 0x012e, PAGE36},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xc482, 0xc422, 0x0129, PAGE36},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xc483, 0xc423, 0x012b, PAGE36},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xc484, 0xc424, 0x012c, PAGE36},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x03, 0xc8, MODE_PANP, 0xc600, 0xc6a0, 0x0302, PAGE38},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x01, 0xc8, MODE_PANP, 0xd500, 0xd520, 0x011a, PAGE53},
    {BUS_EN, SLAVE_ID, BUS_FUN_03H, 0x02, 0xc8, MODE_PANP, 0xd500, 0xd521, 0x013b, PAGE53},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xd580, 0xd520, 0x011a, PAGE53},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xd581, 0xd521, 0x013b, PAGE53},
    {BUS_EN, SLAVE_ID, BUS_FUN_06H, 0x01, 0xc8, MODE_PANP, 0xd582, 0xd522, 0x013c, PAGE53},
};
modbosCmd_t modbusCmdNow = {0};
u16         CmdIndex     = 0;

const dataCheckCmd_t dataCheckLib[] = {
  // en     page  data    back   flag
    {BUS_EN, PAGE19, 0xb320, 0xb350, 0xb380}, //
    {BUS_EN, PAGE19, 0xb321, 0xb351, 0xb381}, //
    {BUS_EN, PAGE19, 0xb322, 0xb352, 0xb382}, //
    {BUS_EN, PAGE20, 0xb420, 0xb450, 0xb480}, //
    {BUS_EN, PAGE20, 0xb421, 0xb451, 0xb481}, //
    {BUS_EN, PAGE20, 0xb422, 0xb452, 0xb482}, //
    {BUS_EN, PAGE20, 0xb423, 0xb453, 0xb483}, //
    {BUS_EN, PAGE20, 0xb424, 0xb454, 0xb484}, //
    {BUS_EN, PAGE22, 0xb620, 0xb650, 0xb680}, //
    {BUS_EN, PAGE22, 0xb621, 0xb651, 0xb681}, //
    {BUS_EN, PAGE24, 0xb820, 0xb850, 0xb880}, //
    {BUS_EN, PAGE24, 0xb821, 0xb851, 0xb881}, //
    {BUS_EN, PAGE24, 0xb822, 0xb852, 0xb882}, //
    {BUS_EN, PAGE25, 0xb920, 0xb950, 0xb980}, //
    {BUS_EN, PAGE25, 0xb921, 0xb951, 0xb981}, //
    {BUS_EN, PAGE25, 0xb922, 0xb952, 0xb982}, //
    {BUS_EN, PAGE25, 0xb923, 0xb953, 0xb983}, //
    {BUS_EN, PAGE27, 0xbb20, 0xbb50, 0xbb80}, //
    {BUS_EN, PAGE27, 0xbb21, 0xbb51, 0xbb81}, //
    {BUS_EN, PAGE27, 0xbb22, 0xbb52, 0xbb82}, //
    {BUS_EN, PAGE27, 0xbb23, 0xbb53, 0xbb83}, //
    {BUS_EN, PAGE27, 0xbb24, 0xbb54, 0xbb84}, //
    {BUS_EN, PAGE27, 0xbb25, 0xbb55, 0xbb85}, //
    {BUS_EN, PAGE27, 0xbb26, 0xbb56, 0xbb86}, //
    {BUS_EN, PAGE27, 0xbb27, 0xbb57, 0xbb87}, //
    {BUS_EN, PAGE31, 0xbf20, 0xbf50, 0xbf80}, //
    {BUS_EN, PAGE31, 0xbf21, 0xbf51, 0xbf81}, //
    {BUS_EN, PAGE31, 0xbf22, 0xbf52, 0xbf82}, //
    {BUS_EN, PAGE31, 0xbf23, 0xbf53, 0xbf83}, //
    {BUS_EN, PAGE36, 0xc420, 0xc450, 0xc480}, //
    {BUS_EN, PAGE36, 0xc421, 0xc451, 0xc481}, //
    {BUS_EN, PAGE36, 0xc422, 0xc452, 0xc482}, //
    {BUS_EN, PAGE36, 0xc423, 0xc453, 0xc483}, //
    {BUS_EN, PAGE36, 0xc424, 0xc454, 0xc484}, //
    {BUS_EN, PAGE53, 0xd520, 0xd550, 0xd580}, //
    {BUS_EN, PAGE53, 0xd521, 0xd551, 0xd581}, //
    {BUS_EN, PAGE53, 0xd522, 0xd552, 0xd582}, //
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
    u16 cache[3] = 0;
    ReadDGUS(0xc3a0, (u8 *)&cache[0], 2);
    WriteDGUS(0xc322, (u8 *)&cache[0], 2);
    cache[0] = 0x005a;
    cache[1] = 0x005a;
    cache[2] = 0x005a;
    WriteDGUS(0xc380, (u8 *)&cache[0], 6);
}
