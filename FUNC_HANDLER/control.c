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
    {PAGE00, PASSWORDLEVEL0}, //  PASSWORD_PAGEJUMP_00_EVENT
    {PAGE01, PASSWORDLEVEL0}, //  PASSWORD_PAGEJUMP_01_EVENT
    {PAGE02, PASSWORDLEVEL0}, //  PASSWORD_PAGEJUMP_02_EVENT
    {PAGE03, PASSWORDLEVEL0}, //  PASSWORD_PAGEJUMP_03_EVENT
    {PAGE04, PASSWORDLEVEL0}, //  PASSWORD_PAGEJUMP_04_EVENT
    {PAGE05, PASSWORDLEVEL0}, //  PASSWORD_PAGEJUMP_05_EVENT
    {PAGE06, PASSWORDLEVEL0}, //  PASSWORD_PAGEJUMP_06_EVENT
    {PAGE07, PASSWORDLEVEL0}, //  PASSWORD_PAGEJUMP_07_EVENT
    {PAGE08, PASSWORDLEVEL0}, //  PASSWORD_PAGEJUMP_08_EVENT
    {PAGE09, PASSWORDLEVEL0}, //  PASSWORD_PAGEJUMP_09_EVENT
    {PAGE10, PASSWORDLEVEL0}, //  PASSWORD_PAGEJUMP_0A_EVENT
    {PAGE11, PASSWORDLEVEL0}, //  PASSWORD_PAGEJUMP_0B_EVENT
    {PAGE12, PASSWORDLEVEL0}, //  PASSWORD_PAGEJUMP_0C_EVENT
    {PAGE13, PASSWORDLEVEL0}, //  PASSWORD_PAGEJUMP_0D_EVENT
    {PAGE14, PASSWORDLEVEL0}, //  PASSWORD_PAGEJUMP_0E_EVENT
    {PAGE15, PASSWORDLEVEL1}, //  set page
    {PAGE16, PASSWORDLEVEL0}, //  PASSWORD_PAGEJUMP_10_EVENT
    {PAGE17, PASSWORDLEVEL0}, //  PASSWORD_PAGEJUMP_11_EVENT
    {PAGE18, PASSWORDLEVEL2}, //  setSys
    {PAGE19, PASSWORDLEVEL0}, //  PASSWORD_PAGEJUMP_13_EVENT
    {PAGE20, PASSWORDLEVEL0}, //  PASSWORD_PAGEJUMP_14_EVENT
    {PAGE21, PASSWORDLEVEL0}, //  PASSWORD_PAGEJUMP_15_EVENT
    {PAGE22, PASSWORDLEVEL0}, //  PASSWORD_PAGEJUMP_16_EVENT
    {PAGE23, PASSWORDLEVEL0}, //  PASSWORD_PAGEJUMP_17_EVENT
    {PAGE24, PASSWORDLEVEL3}, //  sysConfig
    {PAGE25, PASSWORDLEVEL0}, //  PASSWORD_PAGEJUMP_19_EVENT
    {PAGE26, PASSWORDLEVEL0}, //  PASSWORD_PAGEJUMP_1A_EVENT
    {PAGE27, PASSWORDLEVEL2}, //  alarmSet
    {PAGE28, PASSWORDLEVEL0}, //  PASSWORD_PAGEJUMP_1C_EVENT
    {PAGE29, PASSWORDLEVEL0}, //  PASSWORD_PAGEJUMP_1D_EVENT
    {PAGE30, PASSWORDLEVEL0}, //  PASSWORD_PAGEJUMP_1E_EVENT
    {PAGE31, PASSWORDLEVEL0}, //  PASSWORD_PAGEJUMP_1F_EVENT
    {PAGE32, PASSWORDLEVEL0}, //  PASSWORD_PAGEJUMP_20_EVENT
    {PAGE33, PASSWORDLEVEL0}, //  PASSWORD_PAGEJUMP_21_EVENT
    {PAGE34, PASSWORDLEVEL2}, //  MAINTAIM PAGE
    {PAGE35, PASSWORDLEVEL0}, //  PASSWORD_PAGEJUMP_23_EVENT
    {PAGE36, PASSWORDLEVEL0}, //  PASSWORD_PAGEJUMP_24_EVENT
    {PAGE37, PASSWORDLEVEL0}, //  PASSWORD_PAGEJUMP_25_EVENT
    {PAGE38, PASSWORDLEVEL0}, //  PASSWORD_PAGEJUMP_27_EVENT
    {PAGE39, PASSWORDLEVEL0}, //  PASSWORD_PAGEJUMP_27_EVENT
    {PAGE40, PASSWORDLEVEL0}, //  PASSWORD_PAGEJUMP_28_EVENT
    {PAGE41, PASSWORDLEVEL0}, //  PASSWORD_PAGEJUMP_29_EVENT
    {PAGE42, PASSWORDLEVEL0}, //  PASSWORD_PAGEJUMP_2A_EVENT
    {PAGE43, PASSWORDLEVEL0}, //  PASSWORD_PAGEJUMP_2B_EVENT
    {PAGE44, PASSWORDLEVEL0}, //  PASSWORD_PAGEJUMP_2C_EVENT
    {PAGE45, PASSWORDLEVEL0}, //  PASSWORD_PAGEJUMP_2D_EVENT
    {PAGE46, PASSWORDLEVEL0}, //  PASSWORD_PAGEJUMP_2E_EVENT
    {PAGE47, PASSWORDLEVEL0}, //  PASSWORD_PAGEJUMP_2F_EVENT
    {PAGE48, PASSWORDLEVEL0}, //  PASSWORD_PAGEJUMP_30_EVENT
    {PAGE49, PASSWORDLEVEL0}, //  PASSWORD_PAGEJUMP_31_EVENT
    {PAGE50, PASSWORDLEVEL0}, //  PASSWORD_PAGEJUMP_32_EVENT
    {PAGE51, PASSWORDLEVEL0}, //  PASSWORD_PAGEJUMP_33_EVENT
    {PAGE52, PASSWORDLEVEL0}, //  PASSWORD_PAGEJUMP_34_EVENT
    {PAGE53, PASSWORDLEVEL0}, //  PASSWORD_PAGEJUMP_35_EVENT
    {PAGE54, PASSWORDLEVEL0}, //  PASSWORD_PAGEJUMP_36_EVENT
    {PAGE55, PASSWORDLEVEL0}, //  PASSWORD_PAGEJUMP_37_EVENT
    {PAGE56, PASSWORDLEVEL0}, //  PASSWORD_PAGEJUMP_38_EVENT
    {PAGE57, PASSWORDLEVEL0}, //  PASSWORD_PAGEJUMP_39_EVENT
    {PAGE58, PASSWORDLEVEL0}, //  PASSWORD_PAGEJUMP_3A_EVENT
    {PAGE59, PASSWORDLEVEL0}, //  PASSWORD_PAGEJUMP_3B_EVENT
    {PAGE60, PASSWORDLEVEL0}, //  PASSWORD_PAGEJUMP_3C_EVENT
    {PAGE61, PASSWORDLEVEL0}, //  PASSWORD_PAGEJUMP_3D_EVENT
    {PAGE62, PASSWORDLEVEL0}, //  PASSWORD_PAGEJUMP_3E_EVENT
    {PAGE63, PASSWORDLEVEL0}, //  PASSWORD_PAGEJUMP_3F_EVENT
    {PAGE64, PASSWORDLEVEL5}, //  unLock
    {PAGE65, PASSWORDLEVEL0}, //  PASSWORD_PAGEJUMP_41_EVENT
    {PAGE66, PASSWORDLEVEL0}, //  PASSWORD_PAGEJUMP_42_EVENT
    {PAGE67, PASSWORDLEVEL0}, //  PASSWORD_PAGEJUMP_43_EVENT
    {PAGE68, PASSWORDLEVEL0}, //  PASSWORD_PAGEJUMP_44_EVENT
    {PAGE69, PASSWORDLEVEL0}, //  PASSWORD_PAGEJUMP_45_EVENT
};
const u8 funLevel[][2] = {
    {FUN00, PASSWORDLEVEL1  }, // clear current alarm
    {FUN01, PASSWORDLEVEL3  }, // clear alarm history
    {FUN02, PASSWORDLEVEL3  }, // clear curve
    {FUN03, PASSWORDLEVELMAX}, // reset password
    {FUN04, PASSWORDLEVEL2  }, // sysSet
};

u16                     jumpPage    = 0;
u16                     currentPage = 0;
static _password_mode_t mode;
static u16              funOpt       = 0;
static u8               currentLevel = 0;
/* Private function prototypes -----------------------------------------------*/
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
    WriteDGUS(0xaf20, (u8 *)&cache, 2);
    WriteDGUS(0xaf80, (u8 *)&cache, 2);
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
    ReadDGUS(0xd923, (u8 *)&cache, 2);
    if (cache & 1) {
        cache = 0;
    } else {
        cache = 1;
    }
    WriteDGUS(0xd924, (u8 *)&cache, 2);
    cache = 0x005a;
    WriteDGUS(0xd984, (u8 *)&cache, 2);
}

void inMaintainModEventHandle(void)
{
    u16 cache = 0;
    JumpPage(PAGE35);
    cache = 0x005a;
    WriteDGUS(0xc300, (u8 *)&cache, 2);
}
void outMaintainModEventHandle(void)
{
    u16 cache = 0;
    WriteDGUS(0xd927, (u8 *)&cache, 2);
    cache = 0x005a;
    WriteDGUS(0xd987, (u8 *)&cache, 2);
}

void factoryParaOpt(u16 eventId)
{
    u16 cache = eventId & 0xff;
    WriteDGUS(0xb82a, (u8 *)&cache, 2);
    cache = 0x005a;
    WriteDGUS(0xb88a, (u8 *)&cache, 2);
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
    }
}

void pageHandle(u16 page)
{
    u16 cache = 0x005a;
    if (page == PAGE24) {
        WriteDGUS(0xa000 + (page << 8), (u8 *)&cache, 2);
    } else if (page == PAGE27) {
        WriteDGUS(0xa000 + (page << 8), (u8 *)&cache, 2);
    }
}

u8 getPasswordLevel(u16 event)
{
    if (event <= PASSWORD_PAGEJUMP_45_EVENT) {
        return pageLevel[event - PASSWORD_PAGEJUMP_START][1];
    }
    if (event <= PASSWORD_FUN_03_EVENT) {
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
