#ifndef NSW_PROCON_MASK_H
#define NSW_PROCON_MASK_H


//common
//dpad(directional pad, 十字ボタン)
#define UP_MASK 0x01
#define DOWN_MASK 0x02
#define RIGHT_MASK 0x04
#define LEFT_MASK 0x08


//nsw pro-con
#define NSW_A_MASK 0x02
#define NSW_B_MASK 0x01
#define NSW_X_MASK 0x08
#define NSW_Y_MASK 0x04

#define NSW_L_MASK 0x10
#define NSW_R_MASK 0x20
#define NSW_ZL_MASK 0x40
#define NSW_ZR_MASK 0x80

#define NSW_L_STICK_MASK 0x0100
#define NSW_R_STICK_MASK 0x0200


//TODO:値があってるか確認する
//dual shock 3 (sixaxis)
#define DS3_Circle_MASK 0x02
#define DS3_Cross_MASK 0x01
#define DS3_Rectangle_MASK 0x08
#define DS3_Triangle_MASK 0x04

#define DS3_L1_MASK 0x10
#define DS3_R1_MASK 0x20
#define DS3_L2_MASK 0x40
#define DS3_R2_MASK 0x80

#define DS3_L3_MASK 0x0100
#define DS3_R3_MASK 0x0200

#endif //NSW_PROCON_MASK_H