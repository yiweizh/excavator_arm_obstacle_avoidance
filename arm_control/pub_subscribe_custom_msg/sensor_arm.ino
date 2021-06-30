#include <Wire.h>
#include <JY901.h>
#include <SoftwareSerial.h> 

// Define the Software serials
SoftwareSerial serial0(8,9); //RX=8, TX=9;
SoftwareSerial serial1(10,11); //RX=10, TX=11;


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
 * at least you need to do two things
 * 1. Check JY901.h and export more than one instance of the class CJY901 --> I have not seen any hint suggesting that the instance can be only be created uniquely
 * 2. Add more pairs of RX/TX pins by using SoftwareSerial. Remember to use "listen()" method to switch between different serials, since 
 *    Arduino can only "listen to" one softwareSerial at one time. Please check the bulit-in examples
 */

 
 static float stick_angle;
 static float boom_angle;
 float read_stick();
 float read_boom();
 
//Set up all the serials
void setup() 
{
  Serial.begin(9600);
  serial0.begin(9600);
  serial1.begin(9600);

}

void loop() 
{
  if(Serial.available()){
    char read_in = Serial.read();
    if(read_in == 's'){
      //Print out the data when the user pressed 's'
      
      //print received data. Data was received in serialEvent;
      Serial.println("Stick (horizantal):");
      Serial.print("Angle:");Serial.print(stick_angle);
      Serial.println("");
      
      Serial.println("Boom (horizantal):");
      Serial.print("Angle:");Serial.print(boom_angle);
      
      
      Serial.println("");
    }
 }
  
  //Read the new data in and process the corresponding data
  stick_angle = read_stick();
  boom_angle = read_boom();
  
  delay(200);
}

float read_stick(){
  serial0.listen();
  delay(100);
   while (serial0.available()) 
  {
    JY901_0.CopeSerialData(serial0.read()); //Call JY901 data cope function
  }
  return -((float)JY901_0.stcAngle.Angle[1]/32768*180);
}

float read_boom(){
  serial1.listen();
  delay(100);
  while (serial1.available()){
    JY901_1.CopeSerialData(serial1.read());
  }
  return -((float)JY901_1.stcAngle.Angle[1]/32768*180);
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

