//--------------------------------------------------------------------------------
// DCC Smile Function Decoder Motor
// [FunctionDecoderMotor_L1.ino]
// スマイルデコーダ シリーズ の、Locomotive decoder sketch R4をベースにコメントを追加
// https://desktopstation.net/wiki/doku.php/ds_smile_decoder_r4
//
// Copyright (c) 2020 Ayanosuke(Maison de DCC) / Desktop Station
//
// http://maison-dcc.sblo.jp/ http://dcc.client.jp/ http://ayabu.blog.shinobi.jp/
// https://twitter.com/masashi_214
//
// DCC電子工作連合のメンバーです
// https://desktopstation.net/tmi/ https://desktopstation.net/bb/index.php
//
// This software is released under the MIT License.
// http://opensource.org/licenses/mit-license.php
//--------------------------------------------------------------------------------

// DCC Decoder for DS-DCC decode
// By yaasan
// Based on Nicolas's sketch http://blog.nicolas.cx
// Inspired by Geoff Bunza and his 17 Function DCC Decoder & updated library
//
// Debug serial output available on the serial port at baud 115200, aka Tools -> Serial Monitor
//

#include "NmraDcc.h"
#include "motor_ctrl.h"
#include <avr/eeprom.h>	 //required by notifyCVRead() function if enabled below

//#define ONBRAKEPWM
#define DEBUG

//各種設定、宣言

#define DECODER_ADDRESS 3
#define DCC_ACK_PIN 9	//if defined enables the ACK pin functionality. Comment out to disable.


#define MOTOR_LIM_MAX 255

#define CV_VSTART		2
#define CV_ACCRATIO		3
#define CV_DECCRATIO	4

//使用クラスの宣言
NmraDcc	 Dcc;
DCC_MSG	 Packet;

//Task Schedule
unsigned long gPreviousL5 = 0;

//モータ制御関連の変数
uint32_t gPwmLPF_buf = 0;
uint32_t gPwmLPF2_buf = 0;
uint8_t gPwmDir = 128;
uint16_t gSpeedCmd = 0;
uint16_t gPrevSpeed = 0;

uint8_t gCV1_SAddr = 3;
uint8_t gCVx_LAddr = 3;
uint8_t gCV2_Vstart = 32;
uint8_t gCV3_AccRatio = 32;
uint8_t gCV4_DecRatio = 32;

//Internal variables and other.
#if defined(DCC_ACK_PIN)
const int DccAckPin = DCC_ACK_PIN ;
#endif

/* Function assigned pins */
const int FunctionPin0 = 3;
const int FunctionPin1 = 4;
const int FunctionPin2 = 5;
const int FunctionPin3 = 6;
const int FunctionPin4 = 7;

struct CVPair{
  uint16_t	CV;
  uint8_t	Value;
};
CVPair FactoryDefaultCVs [] = {
  {CV_MULTIFUNCTION_PRIMARY_ADDRESS, DECODER_ADDRESS},
  {CV_ACCESSORY_DECODER_ADDRESS_MSB, 0},		//The LSB is set CV 1 in the libraries .h file, which is the regular address location, so by setting the MSB to 0 we tell the library to use the same address as the primary address. 0 DECODER_ADDRESS
  {CV_MULTIFUNCTION_EXTENDED_ADDRESS_MSB, 0},	 //XX in the XXYY address
  {CV_MULTIFUNCTION_EXTENDED_ADDRESS_LSB, 0},	 //YY in the XXYY address
  {CV_29_CONFIG, 128 },	 //Make sure this is 0 or else it will be random based on what is in the eeprom which could caue headaches
  {CV_VSTART, 16},
  {CV_ACCRATIO, 64},
  {CV_DECCRATIO, 64},

};

void(* resetFunc) (void) = 0;  //declare reset function at address 0


uint8_t FactoryDefaultCVIndex = sizeof(FactoryDefaultCVs) / sizeof(CVPair);


void notifyCVResetFactoryDefault()
{
	//When anything is writen to CV8 reset to defaults.

	resetCVToDefault();
#ifdef DEBUG
	Serial.println("Resetting...");
#endif
	delay(1000);  //typical CV programming sends the same command multiple times - specially since we dont ACK. so ignore them by delaying

	resetFunc();
};

//------------------------------------------------------------------
// CVをデフォルトにリセット(Initialize cv value)
// Serial.println("CVs being reset to factory defaults");
//------------------------------------------------------------------
void resetCVToDefault()
{
  //CVをデフォルトにリセット
#ifdef DEBUG
	Serial.println("CVs being reset to factory defaults");
#endif
	for (int j=0; j < FactoryDefaultCVIndex; j++ ){
		Dcc.setCV( FactoryDefaultCVs[j].CV, FactoryDefaultCVs[j].Value);
	}
};

//------------------------------------------------------------------
// CV値が変化した時の処理（特に何もしない）
//------------------------------------------------------------------
extern void	   notifyCVChange( uint16_t CV, uint8_t Value){
   //CVが変更されたときのメッセージ

#ifdef DEBUG
   Serial.print("CV ");
   Serial.print(CV);
   Serial.print(" Changed to ");
   Serial.println(Value, DEC);
#endif
};

//------------------------------------------------------------------
// CV Ackの処理
// そこそこ電流を流さないといけない
// 呼び出されるたびにFWD/REVを切り替える
//------------------------------------------------------------------
void notifyCVAck(void)
{
  MOTOR_Ack();
}


//------------------------------------------------------------------
// Arduino固有の関数 setup() :初期設定
//------------------------------------------------------------------
void setup()
{
#ifdef DEBUG
	//シリアル通信開始
	Serial.begin(115200);
#endif

	//D9,D10 PWM キャリア周期:31kHz
	TCCR1B &= B11111000;
	TCCR1B |= B00000001;

	//PWM出力ピン D9,D10を出力にセット
	pinMode(9, OUTPUT);
	pinMode(10, OUTPUT);

	//ファンクションの割り当てピン初期化
	pinMode(FunctionPin0, OUTPUT);
	digitalWrite(FunctionPin0, 0);

	pinMode(FunctionPin1, OUTPUT);
	digitalWrite(FunctionPin1, 0);

	pinMode(FunctionPin2, OUTPUT);
	digitalWrite(FunctionPin2, 0);

	pinMode(FunctionPin3, OUTPUT);
	digitalWrite(FunctionPin3, 0);

	pinMode(FunctionPin4, OUTPUT);
	digitalWrite(FunctionPin4, 0);


	//DCCの応答用負荷ピン

	#if defined(DCCACKPIN)
	//Setup ACK Pin
	//pinMode(DccAckPin,OUTPUT);
	//digitalWrite(DccAckPin, 0);
	#endif

   #if !defined(DECODER_DONT_DEFAULT_CV_ON_POWERUP)
	if ( Dcc.getCV(CV_MULTIFUNCTION_PRIMARY_ADDRESS) == 0xFF ){	 //if eeprom has 0xFF then assume it needs to be programmed
	  Serial.println("CV Defaulting due to blank eeprom");
	  notifyCVResetFactoryDefault();

   } else{
	 Serial.println("CV Not Defaulting");
   }
  #else
	 Serial.println("CV Defaulting Always On Powerup");
	 notifyCVResetFactoryDefault();
  #endif


  // Setup which External Interrupt, the Pin it's associated with that we're using, disable pullup.
  Dcc.pin(0, 2, 0);

  // Call the main DCC Init function to enable the DCC Receiver
  Dcc.init( MAN_ID_DIY, 100,   FLAGS_MY_ADDRESS_ONLY , 0 );

  //Reset task
  gPreviousL5 = millis();

  //Init CVs
  gCV1_SAddr = Dcc.getCV( CV_MULTIFUNCTION_PRIMARY_ADDRESS ) ;
  gCV2_Vstart = Dcc.getCV( CV_VSTART ) ;
  gCV3_AccRatio = Dcc.getCV( CV_ACCRATIO ) ;
  gCV4_DecRatio = Dcc.getCV( CV_DECCRATIO ) ;
  gCVx_LAddr = (Dcc.getCV( CV_MULTIFUNCTION_EXTENDED_ADDRESS_MSB ) << 8) + Dcc.getCV( CV_MULTIFUNCTION_EXTENDED_ADDRESS_LSB );

#ifdef DEBUG
  Serial.print("CV1(ShortAddr): ");
  Serial.println(gCV1_SAddr);
  Serial.print("CV17/18(LongAddr): ");
  Serial.println(gCVx_LAddr);
  Serial.print("CV2(Vstart): ");
  Serial.println(gCV2_Vstart);
  Serial.print("CV3(AccRatio): ");
  Serial.println(gCV3_AccRatio);
  Serial.print("CV2(DecRatio): ");
  Serial.println(gCV4_DecRatio);
  Serial.println("Ready");
#endif
}

//---------------------------------------------------------------------
// Arduino main loop
//---------------------------------------------------------------------
void loop(){

	// You MUST call the NmraDcc.process() method frequently from the Arduino loop() function for correct library operation
	Dcc.process();

	if( (millis() - gPreviousL5) >= 100){
		//Motor drive control
		MOTOR_Main(gSpeedCmd, gPwmDir);

		//Reset task
		gPreviousL5 = millis();
	}
}





//---------------------------------------------------------------------
//DCC速度信号の受信によるイベント
//extern void notifyDccSpeed( uint16_t Addr, uint8_t Speed, uint8_t ForwardDir, uint8_t MaxSpeed )
//---------------------------------------------------------------------
extern void notifyDccSpeed( uint16_t Addr, DCC_ADDR_TYPE AddrType, uint8_t Speed, DCC_DIRECTION Dir, DCC_SPEED_STEPS SpeedSteps )
{

	uint16_t aSpeedRef = 0;

	//速度値の正規化(255を100%とする処理)
	if( Speed >= 2){
    aSpeedRef = (Speed * 255) / SpeedSteps;
	}	else {
		aSpeedRef = 0;
	}

	//リミッタ
	if(aSpeedRef > 255) {
		aSpeedRef = 255;
	}


#if 0	
    Serial.print("Speed - ADR: ");
    Serial.print(Addr);
    Serial.print(", AddrType: ");
    Serial.print(AddrType);
    Serial.print(", SPD: ");
    Serial.print(Speed);
    Serial.print(", DIR: ");
    Serial.print(Dir);
    Serial.print(", SpeedSteps: ");
    Serial.print(SpeedSteps);
    Serial.print(", aSpeedRef: ");
    Serial.println(aSpeedRef);
#endif
	
	
	
	
	gSpeedCmd = aSpeedRef;
	gPwmDir = Dir;

}

//---------------------------------------------------------------------------
//ファンクション信号受信のイベント
//FN_0_4とFN_5_8は常時イベント発生（DCS50KはF8まで）
//FN_9_12以降はFUNCTIONボタンが押されたときにイベント発生
//前値と比較して変化あったら処理するような作り。
//---------------------------------------------------------------------------
//extern void notifyDccFunc( uint16_t Addr, FN_GROUP FuncGrp, uint8_t FuncState)
extern void notifyDccFunc(uint16_t Addr, DCC_ADDR_TYPE AddrType, FN_GROUP FuncGrp, uint8_t FuncState)
{
	switch(FuncGrp)
	{
		case FN_0_4:	//Function Group 1 F0 F4 F3 F2 F1
			exec_function( 0, FunctionPin0, (FuncState & FN_BIT_00)>>4 );
			exec_function( 1, FunctionPin1, (FuncState & FN_BIT_01));
			exec_function( 2, FunctionPin2, (FuncState & FN_BIT_02)>>1);
			exec_function( 3, FunctionPin3, (FuncState & FN_BIT_03)>>2 );
			exec_function( 4, FunctionPin4, (FuncState & FN_BIT_04)>>3 );
			break;

		case FN_5_8:	//Function Group 1 S FFFF == 1 F8 F7 F6 F5	&  == 0	 F12 F11 F10 F9 F8
			//exec_function( 5, FunctionPin5, (FuncState & FN_BIT_05));
			//exec_function( 6, FunctionPin6, (FuncState & FN_BIT_06)>>1 );
			//exec_function( 7, FunctionPin7, (FuncState & FN_BIT_07)>>2 );
			//exec_function( 8, FunctionPin8, (FuncState & FN_BIT_08)>>3 );
			break;

		case FN_9_12:
			//exec_function( 9, FunctionPin9,	(FuncState & FN_BIT_09));
			//exec_function( 10, FunctionPin10, (FuncState & FN_BIT_10)>>1 );
			//exec_function( 11, FunctionPin11, (FuncState & FN_BIT_11)>>2 );
			//exec_function( 12, FunctionPin12, (FuncState & FN_BIT_12)>>3 );
			break;

		case FN_13_20:	 //Function Group 2 FuncState == F20-F13 Function Control
			//exec_function( 13, FunctionPin13, (FuncState & FN_BIT_13));
			//exec_function( 14, FunctionPin14, (FuncState & FN_BIT_14)>>1 );
			//exec_function( 15, FunctionPin15, (FuncState & FN_BIT_15)>>2 );
			//exec_function( 16, FunctionPin16, (FuncState & FN_BIT_16)>>3 );
			break;

		case FN_21_28:
			break;

	}
}

void exec_function (int function, int pin, int FuncState)
{
	digitalWrite (pin, FuncState);
}
