/**
 * @file control.h
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
#ifndef __CONTROL_H_
#define __CONTROL_H_
#include "sys.h"

#define TOUCH_EVENT_FLAG 0x5014

#define POWER_SWITCH_EVENT         0Xa000
#define ALARM_CONFIRM_EVENT        0Xa026
#define PASSWORD_CONFIRM_EVENT     0Xa400
#define PASSWORD_CANCLE_EVENT      0Xa401
#define PASSWORD_PAGEJUMP_START    0xa4a0
#define PASSWORD_PAGEJUMP_00_EVENT 0xA4A0  // PAGE00
#define PASSWORD_PAGEJUMP_01_EVENT 0xA4A1  // PAGE01
#define PASSWORD_PAGEJUMP_02_EVENT 0xA4A2  // PAGE02
#define PASSWORD_PAGEJUMP_03_EVENT 0xA4A3  // PAGE03
#define PASSWORD_PAGEJUMP_04_EVENT 0xA4A4  // PAGE04
#define PASSWORD_PAGEJUMP_05_EVENT 0xA4A5  // PAGE05
#define PASSWORD_PAGEJUMP_06_EVENT 0xA4A6  // PAGE06
#define PASSWORD_PAGEJUMP_07_EVENT 0xA4A7  // PAGE07
#define PASSWORD_PAGEJUMP_08_EVENT 0xA4A8  // PAGE08
#define PASSWORD_PAGEJUMP_09_EVENT 0xA4A9  // PAGE09
#define PASSWORD_PAGEJUMP_0A_EVENT 0xA4AA  // PAGE10
#define PASSWORD_PAGEJUMP_0B_EVENT 0xA4AB  // PAGE11
#define PASSWORD_PAGEJUMP_0C_EVENT 0xA4AC  // PAGE12
#define PASSWORD_PAGEJUMP_0D_EVENT 0xA4AD  // PAGE13
#define PASSWORD_PAGEJUMP_0E_EVENT 0xA4AE  // PAGE14
#define PASSWORD_PAGEJUMP_0F_EVENT 0xA4AF  // PAGE15
#define PASSWORD_PAGEJUMP_10_EVENT 0xA4B0  // PAGE16
#define PASSWORD_PAGEJUMP_11_EVENT 0xA4B1  // PAGE17
#define PASSWORD_PAGEJUMP_12_EVENT 0xA4B2  // PAGE18
#define PASSWORD_PAGEJUMP_13_EVENT 0xA4B3  // PAGE19
#define PASSWORD_PAGEJUMP_14_EVENT 0xA4B4  // PAGE20
#define PASSWORD_PAGEJUMP_15_EVENT 0xA4B5  // PAGE21
#define PASSWORD_PAGEJUMP_16_EVENT 0xA4B6  // PAGE22
#define PASSWORD_PAGEJUMP_17_EVENT 0xA4B7  // PAGE23
#define PASSWORD_PAGEJUMP_18_EVENT 0xA4B8  // PAGE24
#define PASSWORD_PAGEJUMP_19_EVENT 0xA4B9  // PAGE25
#define PASSWORD_PAGEJUMP_1A_EVENT 0xA4BA  // PAGE26
#define PASSWORD_PAGEJUMP_1B_EVENT 0xA4BB  // PAGE27
#define PASSWORD_PAGEJUMP_1C_EVENT 0xA4BC  // PAGE28
#define PASSWORD_PAGEJUMP_1D_EVENT 0xA4BD  // PAGE29
#define PASSWORD_PAGEJUMP_1E_EVENT 0xA4BE  // PAGE30
#define PASSWORD_PAGEJUMP_1F_EVENT 0xA4BF  // PAGE31
#define PASSWORD_PAGEJUMP_20_EVENT 0xA4C0  // PAGE32
#define PASSWORD_PAGEJUMP_21_EVENT 0xA4C1  // PAGE33
#define PASSWORD_PAGEJUMP_22_EVENT 0xA4C2  // PAGE34
#define PASSWORD_PAGEJUMP_23_EVENT 0xA4C3  // PAGE35
#define PASSWORD_PAGEJUMP_24_EVENT 0xA4C4  // PAGE36
#define PASSWORD_PAGEJUMP_25_EVENT 0xA4C5  // PAGE37
#define PASSWORD_PAGEJUMP_26_EVENT 0xA4C6  // PAGE38
#define PASSWORD_PAGEJUMP_27_EVENT 0xA4C7  // PAGE39
#define PASSWORD_PAGEJUMP_28_EVENT 0xA4C8  // PAGE40
#define PASSWORD_PAGEJUMP_29_EVENT 0xA4C9  // PAGE41
#define PASSWORD_PAGEJUMP_2A_EVENT 0xA4CA  // PAGE42
#define PASSWORD_PAGEJUMP_2B_EVENT 0xA4CB  // PAGE43
#define PASSWORD_PAGEJUMP_2C_EVENT 0xA4CC  // PAGE44
#define PASSWORD_PAGEJUMP_2D_EVENT 0xA4CD  // PAGE45
#define PASSWORD_PAGEJUMP_2E_EVENT 0xA4CE  // PAGE46
#define PASSWORD_PAGEJUMP_2F_EVENT 0xA4CF  // PAGE47
#define PASSWORD_PAGEJUMP_30_EVENT 0xA4D0  // PAGE48
#define PASSWORD_PAGEJUMP_31_EVENT 0xA4D1  // PAGE49
#define PASSWORD_PAGEJUMP_32_EVENT 0xA4D2  // PAGE50
#define PASSWORD_PAGEJUMP_33_EVENT 0xA4D3  // PAGE51
#define PASSWORD_PAGEJUMP_34_EVENT 0xA4D4  // PAGE52
#define PASSWORD_PAGEJUMP_35_EVENT 0xA4D5  // PAGE53
#define PASSWORD_PAGEJUMP_36_EVENT 0xA4D6  // PAGE54
#define PASSWORD_PAGEJUMP_37_EVENT 0xA4D7  // PAGE55
#define PASSWORD_PAGEJUMP_38_EVENT 0xA4D8  // PAGE56
#define PASSWORD_PAGEJUMP_39_EVENT 0xA4D9  // PAGE57
#define PASSWORD_PAGEJUMP_3A_EVENT 0xA4DA  // PAGE58
#define PASSWORD_PAGEJUMP_3B_EVENT 0xA4DB  // PAGE59
#define PASSWORD_PAGEJUMP_3C_EVENT 0xA4DC  // PAGE60
#define PASSWORD_PAGEJUMP_3D_EVENT 0xA4DD  // PAGE61
#define PASSWORD_PAGEJUMP_3E_EVENT 0xA4DE  // PAGE62
#define PASSWORD_PAGEJUMP_3F_EVENT 0xA4DF  // PAGE63
#define PASSWORD_PAGEJUMP_40_EVENT 0xA4E0  // PAGE64
#define PASSWORD_PAGEJUMP_41_EVENT 0xA4E1  // PAGE65
#define PASSWORD_PAGEJUMP_42_EVENT 0xA4E2  // PAGE66
#define PASSWORD_PAGEJUMP_43_EVENT 0xA4E3  // PAGE67
#define PASSWORD_PAGEJUMP_44_EVENT 0xA4E4  // PAGE68
#define PASSWORD_PAGEJUMP_45_EVENT 0xA4E5  // PAGE69

#define PASSWORD_FUN_00_EVENT 0xA4F0
#define PASSWORD_FUN_01_EVENT 0xA4F1
#define PASSWORD_FUN_02_EVENT 0xA4F2
#define PASSWORD_FUN_03_EVENT 0xA4F3
#define PASSWORD_FUN_04_EVENT 0xA4F4

#define PASSWORD_CHANGE_CONFIRM_EVENT 0xA600
#define PASSWORD_CHANGE_CANCLE_EVENT  0xA601

#define ALARM_CLEAR_EVENT      0Xac00
#define CUR_ALARM_CLEAR_EVENT  0Xac01
#define RESET_EVENT            0Xc500
#define IN_MAINTAIN_MOD_EVENT  0Xc600
#define OUTPUT_EVENT           0Xc700
#define OUT_MAINTAIN_MOD_EVENT 0Xc701
#define CLEAR_RUNTIME_EVENT_0E 0Xc90e
#define CLEAR_RUNTIME_EVENT_0C 0Xc90c
#define CLEAR_RUNTIME_EVENT_01 0Xc901
#define CLEAR_RUNTIME_EVENT_06 0Xc906
#define CLEAR_RUNTIME_EVENT_11 0Xc911
#define REST_ORIGINAL_PARA     0xcf3c
#define SAVE_FACTORY_PARA      0xcf69

#define SAVE_FACTORY_CFG_EVENT 0xD600

#define UNLOCK_EVENT 0xe000

#define LEVEL_NUM            7
#define PASSWORD_FLASH_START 0x3000
#define PASSWORD_FLASH_LENTH (LEVEL_NUM - 1) * 2

typedef enum password_mode {
    PWM_PAGEJUMP = 0x0000,
    PWM_FUN      = 0x0001,
} _password_mode_t;

enum {
    FUN00 = 0X00,
    FUN01 = 0X01,
    FUN02 = 0X02,
    FUN03 = 0X03,
    FUN04 = 0X04,
};

extern u8 passwordGotLevel;

void touchHandler(void);
void resetEventHandle(void);
void clearRunTimeHandle(u16 eventId);
void powerSwitchEventHandle(void);
void inMaintainModEventHandle(void);
void outMaintainModEventHandle(void);
void factoryParaOpt(u16 eventId);
void saveFactoryCFG(void);
void setDIPolarityEventHandle(void);
void setDIMaskEventHandle(void);

void unlockEventHandle(void);

void passwordConfirmEventHandle(void);
void passwordCancleEventHandle(void);
void passwordPageJumpEventHandle(u16 event);
void passwordFunEventHandle(u16 event);
void passwordOperation(void);
void passwordFunOPThandle(u16 fun, u16 page);
u8   getPasswordLevel(u16 event);
u8   checkPassword(u8 level, u8 *input);
void passwordInit(void);
void passwordTask(void);
void savePassword(void);
void passwordChangeConfirmEventHandle(void);
void pageHandle(u16 page);

#endif
