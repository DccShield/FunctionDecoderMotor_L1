//--------------------------------------------------------------------------------
// DCC Smile Function Decoder Do Nothing
// DCC信号を受信するだけで何もしないデコーダ
// [DccCV.H]
// Copyright (c) 2020 Ayanosuke(Maison de DCC) / Desktop Station
// https://desktopstation.net/bb/index.php
//
// This software is released under the MIT License.
// http://opensource.org/licenses/mit-license.php
//--------------------------------------------------------------------------------

//#ifndef _DccCV_h_
//#define _DccCV_h_

#pragma once

#define DECODER_ADDRESS 3

#define ON 1
#define OFF 0
#define UP 1
#define DOWN 0

#define CV_VSTART    2
#define CV_ACCRATIO   3
#define CV_DECCRATIO  4
#define CV_F0_FORWARD 33
#define CV_F0_BACK 34
#define CV_F1 35
#define CV_F2 36
#define CV_F3 37
#define CV_F4 38
#define CV_F5 39
#define CV_F6 40
#define CV_F7 41
#define CV_F8 42
#define CV_F9 43
#define CV_F10 44
#define CV_F11 45
#define CV_F12 46
#define CV_49_F0_FORWARD_LIGHT 49
#define CV_DIMMING_SPEED 50
#define CV_DIMMING_LIGHT_QUANTITY 51
#define CV_ROOM_DIMMING 52

#define CV_zeroDeg 60       // 0deg時のPWMの値
#define CV_ninetyDeg 61     // 90deg時のPWMの値
#define CV_onDeg 62         // on時の角度
#define CV_offDeg 63        // off時の角度
#define CV_initDeg 64       // 起動時の角度
#define CV_onSpeed 65       // off->onに移行する時間
#define CV_offSpeed 66      // on->offに移行する時間
#define CV_sdir 67          // 最後のdir保持用
#define CV_function 68   // サーボモータファンクッション番号
#define CV_dummy 69         // dummy
#define MAN_ID_NUMBER 166   // Manufacture ID //
#define MAN_VER_NUMBER  01  // Release Ver CV07 //

//Task Schedule
unsigned long gPreviousL5 = 0;

//Function State
uint8_t gState_F0 = 0;
uint8_t gState_F1 = 0;
uint8_t gState_F2 = 0;
uint8_t gState_F3 = 0;
uint8_t gState_F4 = 0;
uint8_t gState_F5 = 0;
uint8_t gState_F6 = 0;
uint8_t gState_F7 = 0;
uint8_t gState_F8 = 0;
uint8_t gState_F9 = 0;
uint8_t gState_F10 = 0;
uint8_t gState_F11 = 0; 
uint8_t gState_F12 = 0;
uint8_t gState_F13 = 0;
uint8_t gState_F14 = 0; 
uint8_t gState_F15 = 0;
uint8_t gState_F16 = 0;
uint8_t gState_Function = 0;

//CV related
uint8_t gCV1_SAddr = 3; 
uint8_t gCVx_LAddr = 3;
uint8_t gCV29_Conf = 0;
uint8_t gCV49_fx = 20;
  
// #endif
