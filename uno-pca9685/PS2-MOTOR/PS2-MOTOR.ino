#include "FaBoPWM_PCA9685.h"

//#include "servo.hpp"
// DFFENItion
//Serial DEF
const byte numChars = 32;
char receivedChars[numChars];   // an array to store the received data
boolean newData = false;
int number_1;
int number_2;
int number_3;
int number_4;
int number_5;
//End Serial DEF

// Wheels Deff
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
SerialLoop();
    delay(20);
    
  }

////////////////////////Serial Methods ////

void SerialLoop()
{ 
  
   recvWithEndMarker();
   showNewData();
   if (newData)
   {
      parseData();
   }  
  
}



//Updating Arduino Methods
void UpdateArduino()
{
  
  Serial.println("Start Updating Arduino");
  Serial.println("Updating Arduino ~ ");
  ADVANCE(number_1,number_2,number_3,number_4);
  Serial.println("waiting ...");
  delay(number_5*10);
  ADVANCE(0,0,0,0);
  Serial.println("Stop");
  Serial.println(" End Updating ~ ");
}

// SerialMethods //
void recvWithEndMarker()
{
   static byte ndx = 0;
   char endMarker = '>';
   char startMarker = '<';
   char rc;
   while (Serial.available() > 0 && newData == false)
   {    
      rc = Serial.read();
      if (rc==startMarker)
      {
        rc = Serial.read();
      }
  switch(rc)
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
      
      if (rc != endMarker)
      {
         receivedChars[ndx] = rc;
         ndx++;
         if (ndx >= numChars)
         {
            ndx = numChars - 1;
         }
      }
      else
      {
         receivedChars[ndx] = '>'; // terminate the string
         ndx = 0;
         newData = true;
      }
   }
}

void showNewData()
{
   if (newData == true)
   {
      Serial.print("This just in ... ");
      Serial.println(receivedChars);
      //newData = false;
   }
}

void parseData()
{
   char *strings[8]; // an array of pointers to the pieces of the above array after strtok()
   char *ptr = NULL; byte index = 0;
   ptr = strtok(receivedChars, ",");  // delimiters, semicolon
   while (ptr != NULL)
   {
      strings[index] = ptr;
      index++;
      ptr = strtok(NULL, ",");
   }   
   // convert string data to numbers
   number_1 = atoi(strings[0]);
   number_2 = atoi(strings[1]);
   number_3 = atoi(strings[2]);
   number_4 = atoi(strings[3]);
   number_5 = atoi(strings[4]);
   UpdateArduino(); 
   newData = false;
}


  
