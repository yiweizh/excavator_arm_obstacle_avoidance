#include <Wire.h>
#include <JY901.h>
#include <SoftwareSerial.h> 

// Define the Software serials
SoftwareSerial serial0(10,11); //RX=10, TX=11;


/*
Test on Uno R3.
JY901   UnoR3
TX <---> 0(Rx)
*/

/*
 * /////////////////////////
 * ////////COMMENT /////////
 * ///////////////////////// 
 * If you want to define more than one pair of RX/TX and control multiple sensors
 * JY901_0 ---> IIC ---> boom
 * JY901_1 ---> Serial ---> stick
 */

 
 static float raw_stick_angle;
 static float stick_angle;
 static float true_stick_angle;
 static float boom_angle;

 static float raw_base_angle;
 static float base_angle; 
 static float base_init;
 static float boom_stick_angle; 

 static float static_diff = 91.5;
 static bool location; // 0: RHP; 1: LHP
 void read_stick();
 void read_boom();
 void read_base();
//float real_base_angle(float raw_base_angle);
//Set up all the serials
void setup() 
{
  Serial.begin(9600);
  serial0.begin(9600);
  JY901_0.StartIIC(0x10);
  JY901_2.StartIIC(); //Horizontal sensor
  JY901.StartIIC(0x20); //Reference sensor
}

void loop() 
{
    //Read the new data in and process the corresponding data
    read_boom();
    read_stick();
    read_base();

    stick_angle = (location == 0)? raw_stick_angle : (-180-raw_stick_angle);
    if(stick_angle < - 75 && stick_angle > -105){
      JY901.GetAngle();
      true_stick_angle =((float)JY901.stcAngle.Angle[1]/32768*180) - static_diff;
    }
    else{
      true_stick_angle = stick_angle; 
    }
    
      
    //print received data. Data was received in serialEvent;
    Serial.print(true_stick_angle);
//    Serial.print(stick_angle);
    Serial.print(',');
    Serial.print(boom_angle);
    Serial.print(',');

    //Calculate the angle between the boom and the stick
    boom_stick_angle = 180-(boom_angle-true_stick_angle);
//    boom_stick_angle = 180-(boom_angle-stick_angle);
    Serial.print(boom_stick_angle);
    Serial.print(',');

    //Serial.print(base_angle);
    Serial.print(raw_base_angle);
//    Serial.print(',');
//
//    Serial.print(base_init);
//    Serial.print(',');
//    Serial.print(raw_base_angle);
    Serial.println("");
    
    delay(10);
}

void read_stick(){

   while (serial0.available()) 
  {
    JY901_1.CopeSerialData(serial0.read()); //Call JY901 data cope function
  }
  raw_stick_angle = -((float)JY901_1.stcAngle.Angle[1]/32768*180);
  location = ((float)JY901_1.stcAngle.Angle[0] < 0 )? 1:0;
}
void read_base(){
  JY901_2.GetAngle();
  raw_base_angle = ((float)JY901_2.stcAngle.Angle[2]/32768*180);
}
void read_boom(){
 JY901_0.GetAngle();
 boom_angle = ((float)JY901_0.stcAngle.Angle[1]/32768*180);
}
//
//float real_base_angle(float raw_base_angle){
//  float result = raw_base_angle - base_init;
//  if(base_init < -90 && raw_base_angle >= 0){
//    result = -360-base_init+raw_base_angle;// -180-base_init-(180-raw_base_angle)
//  }
//  else if (base_init > 90 && raw_base_angle <0){
//    result = 360-base_init + raw_base_angle;//180-base_init + raw_base_angle+180;
//  }
//  return result;
//}



