#include <ros.h>
#include <Servo.h> 
#include <std_msgs/UInt32.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>

#include "AutoControl.h"

Servo SOURCE_ESC;

#define Source_ESCPin 11



/******** 初始化 ********/
void setup()
{
  /* 上位机串口初始化 */
  Serial.begin(9600);
  
  SOURCE_ESC.attach(Source_ESCPin);
  SOURCE_ESC.writeMicroseconds(200*Source_Pwm_Init);
}


/******** 主循环 ********/
void loop()
{  

  /* 用串口调试助手控制舵机 */
  static long lastMillis = millis();

  static float Source_Pwm_Out = Source_Pwm_Init;
  
  static uint8_t ch = 0;
  static uint8_t ch_h = 0;
  static uint8_t ch_l = 0;

  static uint8_t mode = 0;
  /*
   mode 0 - start/stop (Source_Pwm_Out = 5.00f), used to intialize/stop the pump
   mode 1 - static run (Source_Pwm_Out = 6.00f), used under normal operation of the pump
   mode 2 - switching run (Source_Pwm_Out = 5.00f for 500ms, Source_Pwm_Out = 6.00f for 2000ms), used under abnormal situations where pump unexpectedly halts
  */
  
  if(Serial.available()) 
  {
    ch = Serial.read();
    ch_h = ch >> 4;
    ch_l = ch & 0x0F;

    switch(ch)
    {
      case 's':  Source_Pwm_Out= 5.00f; mode = 0; Serial.println("Mode 0: Initialize/Stop pump"); break;
      case 'm': Source_Pwm_Out = 6.00f; mode = 1; Serial.println("Mode 1: Run pump normally"); break;
      case 'd': Source_Pwm_Out = 6.00f; mode = 2; Serial.println("Mode 2: Run pump with abnormal pattern"); break;
      default: /*Source_Pwm_Out = Source_Pwm_Init + 0.8f + (float)ch_l/4; Serial.print("Motor gears:");*/ Serial.println(ch_l); break;
    }
  }

  
  if(mode == 2)
  {
    SOURCE_ESC.writeMicroseconds(200.0*Source_Pwm_Out);
    delay(2000);
    SOURCE_ESC.writeMicroseconds(200.0*Source_Pwm_Init);
    delay(500);
  }
  else
  {
    SOURCE_ESC.writeMicroseconds(200.0*Source_Pwm_Out);
  }
}



/******************** finished ********************/
