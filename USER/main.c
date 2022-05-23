/**
 * @file main.c
 * @brief
 * @author  xiaowine (xiaowine@sina.cn)
 * @version 01.00
 * @date    2020-10-12
 *
 * @copyright Copyright (c) {2020}  xiaowine
 *
 * @par 修改日志:
 * <table>
 * <tr><th>Date       <th>Version <th>Author  <th>Description
 * <tr><td>2020-10-12 <td>1.0     <td>wangh     <td>内容
 * </table>
 * ******************************************************************
 * *                   .::::
 * *                 .::::::::
 * *                ::::::::::
 * *             ..:::::::::::
 * *          '::::::::::::
 * *            .:::::::::
 * *       '::::::::::::::..        女神助攻,流量冲天
 * *            ..::::::::::::.     永不宕机,代码无bug
 * *          ``:::::::::::::::
 * *           ::::``:::::::::'        .:::
 * *          ::::'   ':::::'       .::::::::
 * *        .::::'      ::::     .:::::::'::::
 * *       .:::'       :::::  .:::::::::' ':::::
 * *      .::'        :::::.:::::::::'      ':::::
 * *     .::'         ::::::::::::::'         ``::::
 * * ...:::           ::::::::::::'              ``::
 * *```` ':.          ':::::::::'                  ::::.
 * *                   '.:::::'                    ':'````.
 * ******************************************************************
 */

/*****************************************************************************
系统库*/

/*****************************************************************************
自定义头文件*/
#include "T5LOS8051.H"
#include "sys.h"
// #include "handle.h"
//#include "password.h"
#include "ChineseCharacter.h"
#include "alarm.h"
#include "control.h"
#include "curve.h"
#include "modbus.h"
#include "rtc.h"
#include "string.h"
#include "timer.h"
#include "uart.h"
#include "ui.h"
/*****************************************************************************
主函数*/
void main(void)
{
    InitSys();
    init_rtc();  //硬件RTC初始化
    Modbus_UART_Init();
    curveInit();
    alarmInit();
    passwordInit();
    while (1) {
        if (timer1msFlag) {
            MS1msFlag = 1;
            if (timer100msFlag)
                MS100msFlag = 1;
            if (timer500msFlag)
                MS500msFlag = 1;

            timer1msFlag   = 0;
            timer100msFlag = 0;
            timer500msFlag = 0;
        }
        WDT_RST();               //喂狗
        if (SysTick_RTC >= 500)  //原来是500，调试改为50
        {
            rdtime();  //更新硬件RTC时间
            SysTick_RTC = 0;
            // Uart2SendStr(test, strlen(test));
            RTC_Set_CMD();
        }
        // HandleProc();
        Modbus_Process_Task();  // Modbus串口处理流程
        ui();
        curveProcess();
        alarmTask();
        touchHandler();
        passwordTask();
        if (MS1msFlag) {
            MS1msFlag   = 0;
            MS100msFlag = 0;
            MS500msFlag = 0;
        }
    }
}
