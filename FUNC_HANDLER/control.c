/**
 * @file control.c
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

/* Private includes ----------------------------------------------------------*/
#include "control.h"
#include "T5L_lib.h"
#include "alarm.h"
#include "curve.h"
#include "dgus.h"
#include "modbus.h"
#include "timer.h"
#include "ui.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
u8        password[LEVEL_NUM][4]     = {0};
const u32 defaultPassword[LEVEL_NUM] = {0, 1, 2, 160608, 666888, 200908, 519525};
u8        passwordGotLevel           = 0xff;

const u8 pageLevel[][2] = {
    {PAGE00, 0}, //  PASSWORD_PAGEJUMP_00_EVENT
    {PAGE01, 0}, //  PASSWORD_PAGEJUMP_01_EVENT
    {PAGE02, 0}, //  PASSWORD_PAGEJUMP_02_EVENT
    {PAGE03, 0}, //  PASSWORD_PAGEJUMP_03_EVENT
    {PAGE04, 0}, //  PASSWORD_PAGEJUMP_04_EVENT
    {PAGE05, 0}, //  PASSWORD_PAGEJUMP_05_EVENT
    {PAGE06, 0}, //  PASSWORD_PAGEJUMP_06_EVENT
    {PAGE07, 0}, //  PASSWORD_PAGEJUMP_07_EVENT
    {PAGE08, 0}, //  PASSWORD_PAGEJUMP_08_EVENT
    {PAGE09, 0}, //  PASSWORD_PAGEJUMP_09_EVENT
    {PAGE10, 0}, //  PASSWORD_PAGEJUMP_0A_EVENT
    {PAGE11, 0}, //  PASSWORD_PAGEJUMP_0B_EVENT
    {PAGE12, 0}, //  PASSWORD_PAGEJUMP_0C_EVENT
    {PAGE13, 0}, //  PASSWORD_PAGEJUMP_0D_EVENT
    {PAGE14, 0}, //  PASSWORD_PAGEJUMP_0E_EVENT
    {PAGE15, 1}, //  set page
    {PAGE16, 0}, //  PASSWORD_PAGEJUMP_10_EVENT
    {PAGE17, 0}, //  PASSWORD_PAGEJUMP_11_EVENT
    {PAGE18, 2}, //  setSys
    {PAGE19, 0}, //  PASSWORD_PAGEJUMP_13_EVENT
    {PAGE20, 0}, //  PASSWORD_PAGEJUMP_14_EVENT
    {PAGE21, 0}, //  PASSWORD_PAGEJUMP_15_EVENT
    {PAGE22, 0}, //  PASSWORD_PAGEJUMP_16_EVENT
    {PAGE23, 0}, //  PASSWORD_PAGEJUMP_17_EVENT
    {PAGE24, 0}, //  PASSWORD_PAGEJUMP_18_EVENT
    {PAGE25, 0}, //  PASSWORD_PAGEJUMP_19_EVENT
    {PAGE26, 0}, //  PASSWORD_PAGEJUMP_1A_EVENT
    {PAGE27, 2}, //  alarmSet
    {PAGE28, 0}, //  PASSWORD_PAGEJUMP_1C_EVENT
    {PAGE29, 0}, //  PASSWORD_PAGEJUMP_1D_EVENT
    {PAGE30, 0}, //  PASSWORD_PAGEJUMP_1E_EVENT
    {PAGE31, 0}, //  PASSWORD_PAGEJUMP_1F_EVENT
    {PAGE32, 0}, //  PASSWORD_PAGEJUMP_20_EVENT
    {PAGE33, 0}, //  PASSWORD_PAGEJUMP_21_EVENT
    {PAGE34, 0}, //  PASSWORD_PAGEJUMP_22_EVENT
    {PAGE35, 0}, //  PASSWORD_PAGEJUMP_23_EVENT
    {PAGE36, 0}, //  PASSWORD_PAGEJUMP_24_EVENT
    {PAGE37, 0}, //  PASSWORD_PAGEJUMP_25_EVENT
    {PAGE38, 2}, //  MAINTAIM PAGE
    {PAGE39, 0}, //  PASSWORD_PAGEJUMP_27_EVENT
    {PAGE40, 0}, //  PASSWORD_PAGEJUMP_28_EVENT
    {PAGE41, 0}, //  PASSWORD_PAGEJUMP_29_EVENT
    {PAGE42, 0}, //  PASSWORD_PAGEJUMP_2A_EVENT
    {PAGE43, 0}, //  PASSWORD_PAGEJUMP_2B_EVENT
    {PAGE44, 0}, //  PASSWORD_PAGEJUMP_2C_EVENT
    {PAGE45, 0}, //  PASSWORD_PAGEJUMP_2D_EVENT
    {PAGE46, 0}, //  PASSWORD_PAGEJUMP_2E_EVENT
    {PAGE47, 3}, //  sysConfig
    {PAGE48, 0}, //  PASSWORD_PAGEJUMP_30_EVENT
    {PAGE49, 0}, //  PASSWORD_PAGEJUMP_31_EVENT
    {PAGE50, 0}, //  PASSWORD_PAGEJUMP_32_EVENT
    {PAGE51, 0}, //  PASSWORD_PAGEJUMP_33_EVENT
    {PAGE52, 0}, //  PASSWORD_PAGEJUMP_34_EVENT
    {PAGE53, 0}, //  PASSWORD_PAGEJUMP_35_EVENT
    {PAGE54, 0}, //  PASSWORD_PAGEJUMP_36_EVENT
    {PAGE55, 0}, //  PASSWORD_PAGEJUMP_37_EVENT
    {PAGE56, 0}, //  PASSWORD_PAGEJUMP_38_EVENT
    {PAGE57, 0}, //  PASSWORD_PAGEJUMP_39_EVENT
    {PAGE58, 0}, //  PASSWORD_PAGEJUMP_3A_EVENT
    {PAGE59, 0}, //  PASSWORD_PAGEJUMP_3B_EVENT
    {PAGE60, 0}, //  PASSWORD_PAGEJUMP_3C_EVENT
    {PAGE61, 0}, //  PASSWORD_PAGEJUMP_3D_EVENT
    {PAGE62, 0}, //  PASSWORD_PAGEJUMP_3E_EVENT
    {PAGE63, 0}, //  PASSWORD_PAGEJUMP_3F_EVENT
    {PAGE64, 5}, //  unLock
    {PAGE65, 0}, //  PASSWORD_PAGEJUMP_41_EVENT
    {PAGE66, 0}, //  PASSWORD_PAGEJUMP_42_EVENT
    {PAGE67, 0}, //  PASSWORD_PAGEJUMP_43_EVENT
    {PAGE68, 0}, //  PASSWORD_PAGEJUMP_44_EVENT
    {PAGE69, 0}, //  PASSWORD_PAGEJUMP_45_EVENT
};
const u8 funLevel[][2] = {
    {FUN00, 1}, // clear current alarm
    {FUN01, 3}, // clear alarm history
    {FUN02, 3}, // clear curve
    {FUN03, 6}, // reset password
    {FUN04, 2}, // sysSet
};

u16                     jumpPage    = 0;
u16                     currentPage = 0;
static _password_mode_t mode;
static u16              funOpt       = 0;
static u8               currentLevel = 0;
/* Private function prototypes -----------------------------------------------*/
static void sysSetPageSwitch(void);
/* Private user code ---------------------------------------------------------*/

void touchHandler(void)
{
    u16 touchEventFlag = 0;
    if (!MS1msFlag)
        return;
    ReadDGUS(TOUCH_EVENT_FLAG, (u8 *)&touchEventFlag, 2);
    if (touchEventFlag) {
        switch (touchEventFlag) {
            case POWER_SWITCH_EVENT:
                powerSwitchEventHandle();
                break;
            case ALARM_CONFIRM_EVENT:
                alarmConfirmEventHandle();
                break;
            case PASSWORD_CONFIRM_EVENT:
                passwordConfirmEventHandle();
                break;
            case PASSWORD_CANCLE_EVENT:
                passwordCancleEventHandle();
                break;
            case PASSWORD_PAGEJUMP_00_EVENT:  // PAGE00
            case PASSWORD_PAGEJUMP_01_EVENT:  // PAGE01
            case PASSWORD_PAGEJUMP_02_EVENT:  // PAGE02
            case PASSWORD_PAGEJUMP_03_EVENT:  // PAGE03
            case PASSWORD_PAGEJUMP_04_EVENT:  // PAGE04
            case PASSWORD_PAGEJUMP_05_EVENT:  // PAGE05
            case PASSWORD_PAGEJUMP_06_EVENT:  // PAGE06
            case PASSWORD_PAGEJUMP_07_EVENT:  // PAGE07
            case PASSWORD_PAGEJUMP_08_EVENT:  // PAGE08
            case PASSWORD_PAGEJUMP_09_EVENT:  // PAGE09
            case PASSWORD_PAGEJUMP_0A_EVENT:  // PAGE10
            case PASSWORD_PAGEJUMP_0B_EVENT:  // PAGE11
            case PASSWORD_PAGEJUMP_0C_EVENT:  // PAGE12
            case PASSWORD_PAGEJUMP_0D_EVENT:  // PAGE13
            case PASSWORD_PAGEJUMP_0E_EVENT:  // PAGE14
            case PASSWORD_PAGEJUMP_0F_EVENT:  // PAGE15
            case PASSWORD_PAGEJUMP_10_EVENT:  // PAGE16
            case PASSWORD_PAGEJUMP_11_EVENT:  // PAGE17
            case PASSWORD_PAGEJUMP_12_EVENT:  // PAGE18
            case PASSWORD_PAGEJUMP_13_EVENT:  // PAGE19
            case PASSWORD_PAGEJUMP_14_EVENT:  // PAGE20
            case PASSWORD_PAGEJUMP_15_EVENT:  // PAGE21
            case PASSWORD_PAGEJUMP_16_EVENT:  // PAGE22
            case PASSWORD_PAGEJUMP_17_EVENT:  // PAGE23
            case PASSWORD_PAGEJUMP_18_EVENT:  // PAGE24
            case PASSWORD_PAGEJUMP_19_EVENT:  // PAGE25
            case PASSWORD_PAGEJUMP_1A_EVENT:  // PAGE26
            case PASSWORD_PAGEJUMP_1B_EVENT:  // PAGE27
            case PASSWORD_PAGEJUMP_1C_EVENT:  // PAGE28
            case PASSWORD_PAGEJUMP_1D_EVENT:  // PAGE29
            case PASSWORD_PAGEJUMP_1E_EVENT:  // PAGE30
            case PASSWORD_PAGEJUMP_1F_EVENT:  // PAGE31
            case PASSWORD_PAGEJUMP_20_EVENT:  // PAGE32
            case PASSWORD_PAGEJUMP_21_EVENT:  // PAGE33
            case PASSWORD_PAGEJUMP_22_EVENT:  // PAGE34
            case PASSWORD_PAGEJUMP_23_EVENT:  // PAGE35
            case PASSWORD_PAGEJUMP_24_EVENT:  // PAGE36
            case PASSWORD_PAGEJUMP_25_EVENT:  // PAGE37
            case PASSWORD_PAGEJUMP_26_EVENT:  // PAGE38
            case PASSWORD_PAGEJUMP_27_EVENT:  // PAGE39
            case PASSWORD_PAGEJUMP_28_EVENT:  // PAGE40
            case PASSWORD_PAGEJUMP_29_EVENT:  // PAGE41
            case PASSWORD_PAGEJUMP_2A_EVENT:  // PAGE42
            case PASSWORD_PAGEJUMP_2B_EVENT:  // PAGE43
            case PASSWORD_PAGEJUMP_2C_EVENT:  // PAGE44
            case PASSWORD_PAGEJUMP_2D_EVENT:  // PAGE45
            case PASSWORD_PAGEJUMP_2E_EVENT:  // PAGE46
            case PASSWORD_PAGEJUMP_2F_EVENT:  // PAGE47
            case PASSWORD_PAGEJUMP_30_EVENT:  // PAGE48
            case PASSWORD_PAGEJUMP_31_EVENT:  // PAGE49
            case PASSWORD_PAGEJUMP_32_EVENT:  // PAGE50
            case PASSWORD_PAGEJUMP_33_EVENT:  // PAGE51
            case PASSWORD_PAGEJUMP_34_EVENT:  // PAGE52
            case PASSWORD_PAGEJUMP_35_EVENT:  // PAGE53
            case PASSWORD_PAGEJUMP_36_EVENT:  // PAGE54
            case PASSWORD_PAGEJUMP_37_EVENT:  // PAGE55
            case PASSWORD_PAGEJUMP_38_EVENT:  // PAGE56
            case PASSWORD_PAGEJUMP_39_EVENT:  // PAGE57
            case PASSWORD_PAGEJUMP_3A_EVENT:  // PAGE58
            case PASSWORD_PAGEJUMP_3B_EVENT:  // PAGE59
            case PASSWORD_PAGEJUMP_3C_EVENT:  // PAGE60
            case PASSWORD_PAGEJUMP_3D_EVENT:  // PAGE61
            case PASSWORD_PAGEJUMP_3E_EVENT:  // PAGE62
            case PASSWORD_PAGEJUMP_3F_EVENT:  // PAGE63
            case PASSWORD_PAGEJUMP_40_EVENT:  // PAGE64
            case PASSWORD_PAGEJUMP_41_EVENT:  // PAGE65
            case PASSWORD_PAGEJUMP_42_EVENT:  // PAGE66
            case PASSWORD_PAGEJUMP_43_EVENT:  // PAGE67
            case PASSWORD_PAGEJUMP_44_EVENT:  // PAGE68
            case PASSWORD_PAGEJUMP_45_EVENT:  // PAGE69
                passwordPageJumpEventHandle(touchEventFlag);
                break;
            case PASSWORD_FUN_00_EVENT:
            case PASSWORD_FUN_01_EVENT:
            case PASSWORD_FUN_02_EVENT:
            case PASSWORD_FUN_03_EVENT:
            case PASSWORD_FUN_04_EVENT:
                passwordFunEventHandle(touchEventFlag);
                break;
            case PASSWORD_CHANGE_CONFIRM_EVENT:
                passwordChangeConfirmEventHandle();
                break;
            case PASSWORD_CHANGE_CANCLE_EVENT:
                break;
            case ALARM_CLEAR_EVENT:
                alarmClearHandle();
                break;
            case CUR_ALARM_CLEAR_EVENT:
                curAlarmClearHandle();
                break;
            case RESET_EVENT:
                resetEventHandle();
                break;
            case IN_MAINTAIN_MOD_EVENT:
                inMaintainModEventHandle();
                break;
            case OUTPUT_EVENT:
                forcedOutputHandle();
                break;
            case OUT_MAINTAIN_MOD_EVENT:
                outMaintainModEventHandle();
                break;
            case CLEAR_RUNTIME_EVENT_0E:
            case CLEAR_RUNTIME_EVENT_0C:
            case CLEAR_RUNTIME_EVENT_01:
            case CLEAR_RUNTIME_EVENT_06:
            case CLEAR_RUNTIME_EVENT_11:
                clearRunTimeHandle(touchEventFlag);
                break;
            case REST_ORIGINAL_PARA:
            case SAVE_FACTORY_PARA:
                factoryParaOpt(touchEventFlag);
                break;
            case SAVE_FACTORY_CFG_EVENT:
                saveFactoryCFG();
                break;
            case UNLOCK_EVENT:
                unlockEventHandle();
                break;
            default:
                break;
        }
        touchEventFlag = 0;
        WriteDGUS(TOUCH_EVENT_FLAG, (u8 *)&touchEventFlag, 2);
    }
}

void resetEventHandle(void)
{
    u16 cache;
    cache = 0x005a;
    WriteDGUS(0xc520, (u8 *)&cache, 2);
    WriteDGUS(0xc580, (u8 *)&cache, 2);
}

void clearRunTimeHandle(u16 eventId)
{
    u16 cache = eventId & 0xff;
    WriteDGUS(0xc930, (u8 *)&cache, 2);
    cache = 0x005a;
    WriteDGUS(0xc990, (u8 *)&cache, 2);
}
void powerSwitchEventHandle(void)
{
    u16 cache;
    ReadDGUS(0xa023, (u8 *)&cache, 2);
    if (cache & 1) {
        cache = 0;
    } else {
        cache = 1;
    }
    WriteDGUS(0xa024, (u8 *)&cache, 2);
    cache = 0x005a;
    WriteDGUS(0xa084, (u8 *)&cache, 2);
}

void inMaintainModEventHandle(void)
{
    u16 cache = 0;
    ReadDGUS(0xcf27, (u8 *)&cache, 2);
    if (cache > 2) {
        JumpPage(67);
    } else {
        JumpPage(39);
    }
    cache = 0x005a;
    WriteDGUS(0xc700, (u8 *)&cache, 2);
}
void outMaintainModEventHandle(void)
{
    u16 cache = 0;
    WriteDGUS(0xa027, (u8 *)&cache, 2);
    cache = 0x005a;
    WriteDGUS(0xa087, (u8 *)&cache, 2);
}

void factoryParaOpt(u16 eventId)
{
    u16 cache = eventId & 0xff;
    WriteDGUS(0xcf2a, (u8 *)&cache, 2);
    cache = 0x005a;
    WriteDGUS(0xcf8a, (u8 *)&cache, 2);
}

void saveFactoryCFG(void)
{
    u16 cache = 0x69;
    WriteDGUS(0xd62b, (u8 *)&cache, 2);
    cache = 0x005a;
    WriteDGUS(0xd68b, (u8 *)&cache, 2);
}

void unlockEventHandle(void)
{
    u16 cache = 0x23;
    WriteDGUS(0xe020, (u8 *)&cache, 2);
    cache = 0x005a;
    WriteDGUS(0xe080, (u8 *)&cache, 2);
}

void passwordConfirmEventHandle(void)
{
    u8 cache[4] = {0};
    ReadDGUS(0xa420, cache, 4);
    if (checkPassword(currentLevel, cache)) {
        passwordOperation();
    } else {
        JumpPage(5);
    }
    memset(cache, 0, 4);
    WriteDGUS(0xa420, cache, 4);
}

void passwordCancleEventHandle(void)
{
    JumpPage(currentPage);
}

void passwordPageJumpEventHandle(u16 event)
{
    jumpPage     = event - PASSWORD_PAGEJUMP_START;
    currentPage  = picNow;
    currentLevel = getPasswordLevel(event);
    mode         = PWM_PAGEJUMP;
    if (currentLevel <= passwordGotLevel) {
        passwordOperation();
    } else {
        JumpPage(4);
    }
}

void passwordFunEventHandle(u16 event)
{
    funOpt       = event - PASSWORD_FUN_00_EVENT;
    currentLevel = getPasswordLevel(event);
    currentPage  = picNow;
    mode         = PWM_FUN;
    if (currentLevel <= passwordGotLevel) {
        passwordOperation();
    } else {
        JumpPage(4);
    }
}

void passwordOperation(void)
{
    switch (mode) {
        case PWM_PAGEJUMP:
            JumpPage(jumpPage);
            pageHandle(jumpPage);
            break;
        case PWM_FUN:
            passwordFunOPThandle(funOpt, currentPage);
            break;
        default:
            break;
    }
}
void passwordFunOPThandle(u16 fun, u16 page)
{
    if (fun == FUN00) {
        JumpPage(page);
        curAlarmClearHandle();
    } else if (fun == FUN01) {
        JumpPage(page);
        alarmClearHandle();
    } else if (fun == FUN02) {
        JumpPage(page);
        curveClearHandle();
    } else if (fun == FUN03) {
        u8 i;
        for (i = 0; i < LEVEL_NUM; i++) {
            *((u32 *)password[i]) = defaultPassword[i];
        }
        JumpPage(page);
        savePassword();
    } else if (fun == FUN04) {
        sysSetPageSwitch();
    }
}

void pageHandle(u16 page)
{
    u16 cache = 0x005a;
    if (page == PAGE27) {
        WriteDGUS(0xa000 + (page << 8), (u8 *)&cache, 2);
    }
    if (page == PAGE47) {
        WriteDGUS(0xa000 + (page << 8), (u8 *)&cache, 2);
    }
}

u8 getPasswordLevel(u16 event)
{
    if (event <= PASSWORD_PAGEJUMP_45_EVENT) {
        return pageLevel[event - PASSWORD_PAGEJUMP_START][1];
    }
    if (event <= PASSWORD_FUN_04_EVENT) {
        return funLevel[event - PASSWORD_FUN_00_EVENT][1];
    }
    return LEVEL_NUM - 1;
}

u8 checkPassword(u8 level, u8 *input)
{
    u8 i;
    if (level == 0)
        return 1;
    for (i = level; i < LEVEL_NUM; i++) {
        if (memcmp(input, &password[i][0], 4) == 0) {
            passwordGotLevel = i;
            return 1;
        }
    }
    return 0;
}

void passwordInit(void)
{
    u8 i;
    T5L_Flash(READRFLASH, VP_TEMP, PASSWORD_FLASH_START, PASSWORD_FLASH_LENTH);
    ReadDGUS(VP_TEMP, (u8 *)password, PASSWORD_FLASH_LENTH * 2);

    for (i = 0; i < LEVEL_NUM; i++) {
        if ((*((u32 *)password[i]) == 0) || (*((u32 *)password[i]) > 999999)) {
            *((u32 *)password[i]) = defaultPassword[i];
        }
    }
    WriteDGUS(0xa6a0, (u8 *)password, 20);
}
void passwordTask(void)
{
    if (picNow == PAGE57) {
        passwordGotLevel = 0;
    }
}

void savePassword(void)
{
    WriteDGUS(VP_TEMP, (u8 *)password, PASSWORD_FLASH_LENTH * 2);
    T5L_Flash(WRITERFLASH, VP_TEMP, PASSWORD_FLASH_START, PASSWORD_FLASH_LENTH);
}

void passwordChangeConfirmEventHandle(void)
{
    u16 cache[10];
    u8  i;
    ReadDGUS(0xa620, (u8 *)cache, 16);
    if (*((u32 *)&cache[2]) != *((u32 *)password[cache[0] + 1])) {
        JumpPage(7);
        return;
    }
    if (*((u32 *)&cache[4]) != *((u32 *)&cache[6])) {
        JumpPage(8);
        return;
    }
    *((u32 *)password[cache[0] + 1]) = *((u32 *)&cache[4]);
    savePassword();
    for (i = 0; i < 10; i++) {
        cache[i] = 0;
    }
    WriteDGUS(0xa622, (u8 *)cache, 12);
}

void passwordChangeCancleEventHandle(void) {}

/**
 * @brief
 */
static void sysSetPageSwitch(void)
{
    u16 cache = 0;
    ReadDGUS(0xcf27, (u8 *)&cache, 2);
    if (cache > 2) {
        JumpPage(66);
    } else {
        JumpPage(18);
    }
}
