#include <Arduino.h>
#include "motor_ctrl.h"
//#define SmDecN18
#define DEBUG             // シリアルにデバックメッセージを出力するときは // を外す
//#define ONBRAKEPWM      // ONブレーキタイプのモータデコーダを使用するときは // を外す　MP6513用

//---------------------------------------------------------------------
// MOTOR_Iniit()
// Motor control Task (10Hz)
//---------------------------------------------------------------------
void MOTOR_Init()
{
#ifdef SmDecN18
  //D3,D11 PWM キャリア周期:31kHz
  TCCR2B &= B11111000;
  TCCR2B |= B00000001;
  //PWM出力ピン D3,D11を出力にセット
  pinMode(MOTOR_PWM_B, OUTPUT);
  pinMode(MOTOR_PWM_A, OUTPUT);
#else
  //D9,D10 PWM キャリア周期:31kHz
  TCCR1B &= B11111000;
  TCCR1B |= B00000001;
  //PWM出力ピン D9,D10を出力にセット
  pinMode(MOTOR_PWM_B, OUTPUT);
  pinMode(MOTOR_PWM_A, OUTPUT);
#endif
}

//---------------------------------------------------------------------
// Motor Main Task (10Hz)
//---------------------------------------------------------------------
void MOTOR_Main(int inSpeedCmd, int inDirection)
{  
  uint16_t aPWMRef = 0;

  aPWMRef = inSpeedCmd;
#ifdef DEBUG
  Serial.print("inSpeedCmd:");
  Serial.print(inSpeedCmd);
  Serial.print(",inDirection:");
  Serial.println(inDirection);
#endif

  //PWM出力
  if( aPWMRef == 0){
    #ifdef ONBRAKEPWM
      analogWrite(MOTOR_PWM_A, 255);
      analogWrite(MOTOR_PWM_B, 255);
    #else
      analogWrite(MOTOR_PWM_A, 0);
      analogWrite(MOTOR_PWM_B, 0);
    #endif
  } else {
    //進行方向でPWMのABを切り替える
    if( inDirection > 0){
      #ifdef ONBRAKEPWM
        analogWrite(MOTOR_PWM_B, 255);            //Change  by MP6513.
        analogWrite(MOTOR_PWM_A, 255 - aPWMRef);  //Change  by MP6513.
      #else
        analogWrite(MOTOR_PWM_B, 0);
        analogWrite(MOTOR_PWM_A, aPWMRef);
      #endif
    }
    else
    {
      #ifdef ONBRAKEPWM
        analogWrite(MOTOR_PWM_A, 255);            //Change  by MP6513.
        analogWrite(MOTOR_PWM_B, 255 - aPWMRef);  //Change  by MP6513.
      #else
        analogWrite(MOTOR_PWM_A, 0);
        analogWrite(MOTOR_PWM_B, aPWMRef);
      #endif
    }
  }
}


//---------------------------------------------------------------------
// MOTOR_AckV()
// デコーダーの応答用関数
// PWM_A と PWM_B を交互に出力するように変更
//---------------------------------------------------------------------
void MOTOR_Ack(void)
{
  static char aFwdRev = 0;
  
  #ifdef DEBUG
  Serial.println("notifyCVAck");
  #endif
  
  if(aFwdRev == 0) {
  #if defined ONBRAKEPWM
    analogWrite(MOTOR_PWM_B, 255);
    analogWrite(MOTOR_PWM_A, 30); 
    delay( 6 );  
    analogWrite(MOTOR_PWM_B, 255);
    analogWrite(MOTOR_PWM_A, 255);
  #else
    analogWrite(MOTOR_PWM_B, 0);
    analogWrite(MOTOR_PWM_A, 50);
    delay( 6 );  
    analogWrite(MOTOR_PWM_A, 0);
  #endif
    aFwdRev = 1;
  } else {
  #ifdef ONBRAKEPWM
    analogWrite(MOTOR_PWM_B, 30);
    analogWrite(MOTOR_PWM_A, 255);
    delay( 6 );  
    analogWrite(MOTOR_PWM_B, 255);
    analogWrite(MOTOR_PWM_A, 255);
  #else
    analogWrite(MOTOR_PWM_B, 50);
    analogWrite(MOTOR_PWM_A, 0);
    delay( 6 );  
    analogWrite(MOTOR_PWM_B, 0);
  #endif
    aFwdRev = 0;  
  }
}
