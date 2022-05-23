/**
 * @file rtc.h
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
#ifndef _RTC_H_
#define _RTC_H_

#include "sys.h"

typedef struct _dev_time {
    u8 year;
    u8 month;
    u8 day;
    u8 week;
    u8 hour;
    u8 min;
    u8 sec;
    u8 res;
} rtc_time;

struct tm {
    int tm_sec;   /* 秒 – 取值区间为[0,59] */
    int tm_min;   /* 分 - 取值区间为[0,59] */
    int tm_hour;  /* 时 - 取值区间为[0,23] */
    int tm_mday;  /* 一个月中的日期 - 取值区间为[1,31] */
    int tm_mon;   /* 月份（从一月开始，0代表一月） - 取值区间为[0,11] */
    int tm_year;  /* 年份，其值等于实际年份减去1900 */
    int tm_wday;  /* 星期 – 取值区间为[0,6]，其中0代表星期天，1代表星期一，以此类推 */
    int tm_yday;  /* 从每年的1月1日开始的天数 – 取值区间为[0,365]，其中0代表1月1日，1代表1月2日，以此类推 */
    int tm_isdst; /* 夏令时标识符，实行夏令时的时候，tm_isdst为正。不实行夏令时的进候，tm_isdst为0；不了解情况时，tm_isdst()为负。*/
};

//#define NULL ((void*)0)
#define DHW_NPAGE 0x0014  //当前页面
#define RTC_Set   0x009C
#define RTC       0x0010

sbit RTC_SDA = P3 ^ 3;
sbit RTC_SCL = P3 ^ 2;

extern u16 data SysTick_RTC;
extern uint8_t  RTCNow[7];
extern uint8_t  RTCHex[7];

void          delayus(unsigned char t);
void          SDA_IN(void);
void          SDA_OUT(void);
void          i2cstart(void);
void          i2cstop(void);
void          mack(void);
void          mnak(void);
void          cack(void);
void          i2cbw(unsigned char dat);
unsigned char i2cbr(void);
void          init_rtc(void);
void          RTC_Set_Time(u8 *prtc_set);
void          rdtime(void);

void    RTC_Set_CMD();
uint8_t RTC_Get_Week(uint8_t, uint8_t, uint8_t);
u32     time_to_stamp(const struct tm *ltm, int utc_diff);

#endif
