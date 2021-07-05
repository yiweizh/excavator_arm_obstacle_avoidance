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

 
 static float stick_angle;
 static float boom_angle;
 static float base_angle; 
 float read_stick();
 float read_boom();
 
//Set up all the serials
void setup() 
{
  Serial.begin(9600);
  serial0.begin(9600);
  JY901_0.StartIIC();

}

void loop() 
{

    //Read the new data in and process the corresponding data
    stick_angle = read_stick();
    base_angle = read_base();
    boom_angle = read_boom();
      
    //print received data. Data was received in serialEvent;
    Serial.print(stick_angle);
    Serial.print(',');
      
    Serial.print(boom_angle);
    Serial.print(',');

    Serial.print(base_angle);
      
      
    Serial.println("");
    
    delay(10);
}

float read_stick(){

   while (serial0.available()) 
  {
    JY901_1.CopeSerialData(serial0.read()); //Call JY901 data cope function
  }
  return -((float)JY901_1.stcAngle.Angle[1]/32768*180);
  
}

float read_boom(){
 JY901_0.GetAngle();
 return ((float)JY901_0.stcAngle.Angle[1]/32768*180);
}

float read_base(){
  JY901_0.GetAngle();
  return ((float)JY901_0.stcAngle.Angle[2]/32768*180);
}
/*
  SerialEvent occurs whenever a new data comes in the
 {\bf hardware serial RX}.  This routine is run between each
 time loop() runs, so using delay inside loop can delay
 response.  Multiple bytes of data may be available.
 */
 /*

//serialEvent is a function that is called between loops when there are interrupts at the hardware serial
//Temporarily it is of no use, since I am processing the data manually at the end of the loop function
//For more details, check https://www.arduino.cc/en/Tutorial/BuiltInExamples/SerialEvent

void serialEvent() 
{
  
  while (mySerial.available()) 
  {

    JY901.CopeSerialData(mySerial.read()); //Call JY901 data cope function
  }
  
}
*/

