#include <PS2X_lib.h>  //for MOEBIUS
#include "FaBoPWM_PCA9685.h"

//#include "servo.hpp"

FaBoPWM faboPWM;
int pos = 0;
int MAX_VALUE = 2000;   // 电机速度限制 motor speed 
int MIN_VALUE = 300;

//PS2手柄引脚；PS2 handle Pin
#define PS2_DAT        13
#define PS2_CMD        11
#define PS2_SEL        10
#define PS2_CLK        12

//MOTOR CONTROL Pin
#define DIRA1 0
#define DIRA2 1
#define DIRB1 2
#define DIRB2 3
#define DIRC1 4
#define DIRC2 5
#define DIRD1 6
#define DIRD2 7

char speed;
// #define pressures   true
#define pressures   false
// #define rumble      true
#define rumble      false

PS2X ps2x; // create PS2 Controller Class

//right now, the library does NOT support hot pluggable controllers, meaning
//you must always either restart your Arduino after you connect the controller,
//or call config_gamepad(pins) again after connecting the controller.

int error = 0;
byte type = 0;
byte vibrate = 0;

void (* resetFunc) (void) = 0;

//电机控制，前进、后退、停止   motor control advance\back\stop
#define MOTORA_FORWARD(pwm)    do{faboPWM.set_channel_value(DIRA1,pwm);faboPWM.set_channel_value(DIRA2, 0);}while(0)
#define MOTORA_STOP(x)         do{faboPWM.set_channel_value(DIRA1,0);faboPWM.set_channel_value(DIRA2, 0);}while(0)
#define MOTORA_BACKOFF(pwm)    do{faboPWM.set_channel_value(DIRA1,0);faboPWM.set_channel_value(DIRA2, pwm);}while(0)

#define MOTORB_FORWARD(pwm)    do{faboPWM.set_channel_value(DIRB1,pwm);faboPWM.set_channel_value(DIRB2, 0);}while(0)
#define MOTORB_STOP(x)         do{faboPWM.set_channel_value(DIRB1,0);faboPWM.set_channel_value(DIRB2, 0);}while(0)
#define MOTORB_BACKOFF(pwm)    do{faboPWM.set_channel_value(DIRB1,0);faboPWM.set_channel_value(DIRB2, pwm);}while(0)

#define MOTORC_FORWARD(pwm)    do{faboPWM.set_channel_value(DIRC1,pwm);faboPWM.set_channel_value(DIRC2, 0);}while(0)
#define MOTORC_STOP(x)         do{faboPWM.set_channel_value(DIRC1,0);faboPWM.set_channel_value(DIRC2, 0);}while(0)
#define MOTORC_BACKOFF(pwm)    do{faboPWM.set_channel_value(DIRC1,0);faboPWM.set_channel_value(DIRC2, pwm);}while(0)

#define MOTORD_FORWARD(pwm)    do{faboPWM.set_channel_value(DIRD1,pwm);faboPWM.set_channel_value(DIRD2, 0);}while(0)
#define MOTORD_STOP(x)         do{faboPWM.set_channel_value(DIRD1,0);faboPWM.set_channel_value(DIRD2, 0);}while(0)
#define MOTORD_BACKOFF(pwm)    do{faboPWM.set_channel_value(DIRD1,0);faboPWM.set_channel_value(DIRD2, pwm);}while(0)

#define SERIAL  Serial

//#define SERIAL  Serial3

#define LOG_DEBUG

#ifdef LOG_DEBUG
#define M_LOG SERIAL.print
#else
#define M_LOG 
#endif

//PWM参数
#define MAX_PWM   2000
#define MIN_PWM   300

int Motor_PWM = 1900;
 
//控制电机运动    宏定义
//    ↑A-----B↑   
//     |  ↑  |
//     |  |  |
//    ↑C-----D↑
void ADVANCE(uint8_t pwm_A,uint8_t pwm_B,uint8_t pwm_C,uint8_t pwm_D)
{
  MOTORA_BACKOFF(Motor_PWM);MOTORB_FORWARD(Motor_PWM);    
  MOTORC_BACKOFF(Motor_PWM);MOTORD_FORWARD(Motor_PWM);    
}

//    ↓A-----B↓ 
//     |  |  |
//     |  ↓  |
//    ↓C-----D↓
void BACK()
{
  MOTORA_FORWARD(Motor_PWM);MOTORB_BACKOFF(Motor_PWM);
  MOTORC_FORWARD(Motor_PWM);MOTORD_BACKOFF(Motor_PWM);
}
//    =A-----B↑   
//     |   ↖ |
//     | ↖   |
//    ↑C-----D=
void LEFT_1()
{
  MOTORA_STOP(Motor_PWM);MOTORB_FORWARD(Motor_PWM);
  MOTORC_BACKOFF(Motor_PWM);MOTORD_STOP(Motor_PWM);
}

//    ↓A-----B↑   
//     |  ←  |
//     |  ←  |
//    ↑C-----D↓
void LEFT_2()
{
  MOTORA_FORWARD(Motor_PWM);MOTORB_FORWARD(Motor_PWM);
  MOTORC_BACKOFF(Motor_PWM);MOTORD_BACKOFF(Motor_PWM);
}
//    ↓A-----B=   
//     | ↙   |
//     |   ↙ |
//    =C-----D↓
void LEFT_3()
{
  MOTORA_FORWARD(Motor_PWM);MOTORB_STOP(Motor_PWM);
  MOTORC_STOP(Motor_PWM);MOTORD_BACKOFF(Motor_PWM);
}
//    ↑A-----B=   
//     | ↗   |
//     |   ↗ |
//    =C-----D↑
void RIGHT_1()
{
  MOTORA_BACKOFF(Motor_PWM);MOTORB_STOP(Motor_PWM);
  MOTORC_STOP(Motor_PWM);MOTORD_FORWARD(Motor_PWM);
}
//    ↑A-----B↓   
//     |  →  |
//     |  →  |
//    ↓C-----D↑
void RIGHT_2()
{
  MOTORA_BACKOFF(Motor_PWM);MOTORB_BACKOFF(Motor_PWM);
  MOTORC_FORWARD(Motor_PWM);MOTORD_FORWARD(Motor_PWM);
}
//    =A-----B↓   
//     |   ↘ |
//     | ↘   |
//    ↓C-----D=
void RIGHT_3()
{
  MOTORA_STOP(Motor_PWM);MOTORB_BACKOFF(Motor_PWM);
  MOTORC_FORWARD(Motor_PWM);MOTORD_STOP(Motor_PWM);
}

//    ↑A-----B↓   
//     | ↗ ↘ |
//     | ↖ ↙ |
//    ↑C-----D↓
void rotate_1()  //tate_1(uint8_t pwm_A,uint8_t pwm_B,uint8_t pwm_C,uint8_t pwm_D) 
{
  MOTORA_BACKOFF(Motor_PWM);MOTORB_BACKOFF(Motor_PWM);
  MOTORC_BACKOFF(Motor_PWM);MOTORD_BACKOFF(Motor_PWM);
}

//    ↓A-----B↑   
//     | ↙ ↖ |
//     | ↘ ↗ |
//    ↓C-----D↑
void rotate_2()  // rotate_2(uint8_t pwm_A,uint8_t pwm_B,uint8_t pwm_C,uint8_t pwm_D)
{
  MOTORA_FORWARD(Motor_PWM);MOTORB_FORWARD(Motor_PWM);
  MOTORC_FORWARD(Motor_PWM);MOTORD_FORWARD(Motor_PWM);
}
//    =A-----B=  
//     |  =  |
//     |  =  |
//    =C-----D=
void STOP()
{
  MOTORA_STOP(Motor_PWM);MOTORB_STOP(Motor_PWM);
  MOTORC_STOP(Motor_PWM);MOTORD_STOP(Motor_PWM);
}

//串口输入控制
void Serial_Control()
{
  char Serial_Data=0;
  if(Serial.available())
  {
    Serial_Data = Serial.read();
  }
  switch(Serial_Data)
  {
     case 'A':  ADVANCE(500,500,500,500);  M_LOG("Run!\r\n");        break;
     case 'B':  RIGHT_1();  M_LOG("Right up!\r\n");     break;
     case 'C':  rotate_2();                            break;      
     case 'D':  RIGHT_3();  M_LOG("Right down!\r\n");   break;
     case 'E':  BACK();     M_LOG("Run!\r\n");          break;
     case 'F':  LEFT_3();   M_LOG("Left down!\r\n");    break;
     case 'G':  rotate_1();                              break;         
     case 'H':  LEFT_1();   M_LOG("Left up!\r\n");     break;
     case 'Z':  STOP();     M_LOG("Stop!\r\n");        break;
     case 'z':  STOP();     M_LOG("Stop!\r\n");        break;
     case 'd':  LEFT_2();   M_LOG("Left!\r\n");        break;
     case 'b':  RIGHT_2();  M_LOG("Right!\r\n");        break;
     case 'L':  Motor_PWM = 1500;                      break;
     case 'M':  Motor_PWM = 500;                       break;
   }
}

void IO_init()
{
  STOP();
}

void setup()
{
   IO_init();
   SERIAL.begin(9600);
  if(faboPWM.begin()) 
  {
    Serial.println("Find PCA9685");
    faboPWM.init(300);
  }
  faboPWM.set_hz(50);
  SERIAL.print("Start"); 

   delay(300) ; //added delay to give wireless ps2 module some time to startup, before configuring it
  //CHANGES for v1.6 HERE!!! **************PAY ATTENTION*************

  Serial.println("Setup Ended  ");
  
}



void loop()
{

      /* You must Read Gamepad to get new values and set vibration values
    ps2x.read_gamepad(small motor on/off, larger motor strenght from 0-255)
    if you don't enable the rumble, use ps2x.read_gamepad(); with no values
    You should call this at least once a second
  */
  
Serial_Control();
    delay(20);
  }
