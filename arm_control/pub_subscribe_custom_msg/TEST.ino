#include <ros.h>
#include <Servo.h> 
#include <std_msgs/UInt32.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>

#include "AutoControl.h"


#define Bucket_ServoPin 4
#define Arm_ServoPin 5
#define Boom_ServoPin 3

#define Swing_ESCPin 10


Servo BUCKET;
Servo ARM;
Servo BOOM;
Servo LEFT_ESC;
Servo RIGHT_ESC;
Servo SWING_ESC;

Servo SOURCE_ESC;

#define Source_ESCPin 11



/******** 初始化 ********/
void setup()
{
  /* 上位机串口初始化 */
  Serial.begin(9600);

  
  /* 舵机初始化 */
  BUCKET.attach(Bucket_ServoPin);
  ARM.attach(Arm_ServoPin);
  BOOM.attach(Boom_ServoPin);
  SWING_ESC.attach(Swing_ESCPin);
  SOURCE_ESC.attach(Source_ESCPin);
  
  BUCKET.writeMicroseconds(200*Bucket_Pwm_Init);
  ARM.writeMicroseconds(200*Small_Pwm_Init);
  BOOM.writeMicroseconds(200*Big_Pwm_Init);
  SWING_ESC.writeMicroseconds(200*Turn_Pwm_Init);
  SOURCE_ESC.writeMicroseconds(200*Source_Pwm_Init);
}


/******** 主循环 ********/
void loop()
{  

  /* 用串口调试助手控制舵机 */
  static long lastMillis = millis();
  static float Big_Pwm_Out = Big_Pwm_Init;
  static float Small_Pwm_Out = Small_Pwm_Init;
  static float Bucket_Pwm_Out = Bucket_Pwm_Init;

  static float Turn_Pwm_Out = Turn_Pwm_Init;

  static float Source_Pwm_Out = Source_Pwm_Init;
  bool flag = 0; 

  static float x=60,y=10, angle=10;
  
  static uint8_t ch = 0;
  static uint8_t ch_h = 0;
  static uint8_t ch_l = 0;
  /*
   * 我注释掉了之后可能用来控制机械臂的部分
   * 现在它就是能转到最大值
   * 转到用户指定位置的部分之后再说吧。。
   */
  /*
  //Raw input of the position 
  static uint8_t read_x;
  static unint8_t read_y;
  static unint8_t read_angle; 
  */
  if(Serial.available()) 
  {
    ch = Serial.read();
    ch_h = ch >> 4;
    ch_l = ch & 0x0F;

    switch(ch)
    {
      case 0x31: Big_Pwm_Out = Big_Pwm_Max; Serial.println("Big_Arm:Down");          break;
      case 0x32: Big_Pwm_Out = Big_Pwm_Min; Serial.println("Big_Arm:Up");            break;
      case 0x33: Small_Pwm_Out = Small_Pwm_Min; Serial.println("Small_Arm:Down");        break;
      case 0x34: Small_Pwm_Out = Small_Pwm_Max; Serial.println("Small_Arm:Up");          break;
      case 0x35: Bucket_Pwm_Out = Bucket_Pwm_Min; Serial.println("Bucket:Down");           break;
      case 0x36: Bucket_Pwm_Out = Bucket_Pwm_Max; Serial.println("Bucket:Up");             break;
      case 0x37: Turn_Pwm_Out = 6.5f;             Serial.println("Turn_Right");            break;
      case 0x38: Turn_Pwm_Out = 8.5f;             Serial.println("Turn_Left");             break;
      case 'l': Source_Pwm_Out = 10.00f; Serial.println("Source Gear Max"); break;
      case 's':  Source_Pwm_Out= 5.00f; Serial.println("Source Gearm Min"); break;
      case 'm': Source_Pwm_Out = 7.00f; Serial.println("Source Gear Medium"); break;
      case 'd': Serial.println("Show Source_Pwm_Out:"); Serial.println(Source_Pwm_Out); break;
      /*
      case 0x08: Get_Pos(boom, arm);                                         break;
      case 0x11: read_x = Serial.read(); read_y = Serial.read(); x=(float)read_x; y=(float)read_y; Pos_Calcu(x, y); read_angle = Serial.read(); angle=(float)read_angle; break;
      case 0x21: Serial.print("Big_Arm_Angle:");       Serial.println(boom);           break;
      case 0x22: Serial.print("Small_Arm_Angle:");     Serial.println(arm);           break;
      case 0x23: Serial.print("Bucket_Angle:");        Serial.println(bucket);           break;*/
      default: /*Source_Pwm_Out = Source_Pwm_Init + 0.8f + (float)ch_l/4; Serial.print("Motor gears:");*/ Serial.println(ch_l); break;
    }

  }

/*
  * If the user specified a new coordinate, 
  * determine the PWM output 
  * based on the desired angle
   
   if(flag){
    Big_Pwm_Out = boom;
    Small_Pwm_Out = arm;
    Turn_Pwm_Out = turn_table; 
   }*/
   
  
  
  /* 写舵机占空比 */
  BOOM.writeMicroseconds(200*Big_Pwm_Out);
  ARM.writeMicroseconds(200*Small_Pwm_Out);
  BUCKET.writeMicroseconds(200*Bucket_Pwm_Out);
  SWING_ESC.writeMicroseconds(200*Turn_Pwm_Out);

  SOURCE_ESC.writeMicroseconds(200*Source_Pwm_Out);
}



/******************** finished ********************/
