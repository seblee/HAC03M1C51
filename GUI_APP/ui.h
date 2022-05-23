/**
 * @file ui.h
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

#ifndef __UI_H
#define __UI_H

#include "dgus.h"
#include "sys.h"

typedef struct dgus_hex {
    u16  VP;
    u16  X;
    u16  Y;
    u16  Color;
    u8   Byte_Num;
    u8   Lib_ID;
    u8   Font_x;
    char String_Code[15];
} dgus_hex_t;
typedef struct dgus_string {
    u16 VP;
    u16 X;
    u16 Y;
    u16 Color;
    u16 Xs;
    u16 Ys;
    u16 Xe;
    u16 Ye;
    u16 Text_length;
    u8  Font0_ID;
    u8  Font1_ID;
    u8  Font_X_Dots;
    u8  Font_Y_Dots;
    u8  Encode_Mode;
    u8  HOR_Dis;
    u8  VER_Dis;
    u8  NC;
} dgus_string_t;

enum {
    PAGE00 = 0x00,
    PAGE01 = 0x01,
    PAGE02 = 0x02,
    PAGE03 = 0x03,
    PAGE04 = 0x04,
    PAGE05 = 0x05,
    PAGE06 = 0x06,
    PAGE07 = 0x07,
    PAGE08 = 0x08,
    PAGE09 = 0x09,
    PAGE10 = 0x0A,
    PAGE11 = 0x0B,
    PAGE12 = 0x0C,
    PAGE13 = 0x0D,
    PAGE14 = 0x0E,
    PAGE15 = 0x0F,
    PAGE16 = 0x10,
    PAGE17 = 0x11,
    PAGE18 = 0x12,
    PAGE19 = 0x13,
    PAGE20 = 0x14,
    PAGE21 = 0x15,
    PAGE22 = 0x16,
    PAGE23 = 0x17,
    PAGE24 = 0x18,
    PAGE25 = 0x19,
    PAGE26 = 0x1A,
    PAGE27 = 0x1B,
    PAGE28 = 0x1C,
    PAGE29 = 0x1D,
    PAGE30 = 0x1E,
    PAGE31 = 0x1F,
    PAGE32 = 0x20,
    PAGE33 = 0x21,
    PAGE34 = 0x22,
    PAGE35 = 0x23,
    PAGE36 = 0x24,
    PAGE37 = 0x25,
    PAGE38 = 0x26,
    PAGE39 = 0x27,
    PAGE40 = 0x28,
    PAGE41 = 0x29,
    PAGE42 = 0x2A,
    PAGE43 = 0x2B,
    PAGE44 = 0x2C,
    PAGE45 = 0x2D,
    PAGE46 = 0x2E,
    PAGE47 = 0x2F,
    PAGE48 = 0x30,
    PAGE49 = 0x31,
    PAGE50 = 0x32,
    PAGE51 = 0x33,
    PAGE52 = 0x34,
    PAGE53 = 0x35,
    PAGE54 = 0x36,
    PAGE55 = 0x37,
    PAGE56 = 0x38,
    PAGE57 = 0x39,
    PAGE58 = 0x3A,
    PAGE59 = 0x3B,
    PAGE60 = 0x3C,
    PAGE61 = 0x3D,
    PAGE62 = 0x3E,
    PAGE63 = 0x3F,
    PAGE64 = 0x40,
    PAGE65 = 0x41,
    PAGE66 = 0x42,
    PAGE67 = 0x43,
    PAGE68 = 0x44,
    PAGE69 = 0x45,
    PAGE70 = 0x46,
    PAGE71 = 0x47,
    PAGE72 = 0x48,
    PAGE73 = 0x49,
    PAGE74 = 0x4A,
    PAGE75 = 0x4B,
    PAGE76 = 0x4C,
    PAGE77 = 0x4D,
    PAGE78 = 0x4E,
    PAGE79 = 0x4F,
    PAGE80 = 0x50,
    PAGE81 = 0x51,
    PAGE82 = 0x52,
    PAGE83 = 0x53,
    PAGE84 = 0x54,
    PAGE85 = 0x55,
    PAGE86 = 0x56,
    PAGE87 = 0x57,
    PAGE88 = 0x58,
    PAGE89 = 0x59,
    PAGE90 = 0x5A,
    PAGE91 = 0x5B,
    PAGE92 = 0x5C,
    PAGE93 = 0x5D,
    PAGE94 = 0x5E,
    PAGE95 = 0x5F,
    PAGE96 = 0x60,
    PAGE97 = 0x61,
    PAGE98 = 0x62,
    PAGE99 = 0x63,
};
#define STANGBYTIME 300 * 2

extern u16 picNow;

void ui(void);
void JumpPage(uint16_t pageId);
void caculateGroupCtrlPic(void);

#endif
