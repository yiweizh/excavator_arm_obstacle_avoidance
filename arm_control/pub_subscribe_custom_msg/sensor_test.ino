#include <Wire.h>
#include <JY901.h>
#include <SoftwareSerial.h> 

// Define the Software serials
SoftwareSerial mySerial(10,11); //RX=10, TX=11;



/*
Test on Uno R3.
JY901   UnoR3
TX <---> 0(Rx)
*/

/*
 * /////////////////////////
 * ////////COMMENT /////////
 * ///////////////////////// 
 * JY901_1 -> mySerial -> software serial
 * JY901_0 -> IIC
 */
//Set up all the serials
void setup() 
{
  Serial.begin(9600);
  mySerial.begin(9600);
  JY901_0.StartIIC();

}

void loop() 
{
  //if(Serial.available()){
    //char read_in = Serial.read();
    //if(read_in == 's'){
      //Print out the data when the user pressed 's'
      
      //print received data. Data was received in serialEvent;
     JY901_0.GetAngle();
    
     Serial.print((float)JY901_0.stcAngle.Angle[1]/32768*180);
     Serial.print(',');
     Serial.print((float)JY901_1.stcAngle.Angle[1]/32768*180);
      
      
      Serial.println("");
   // }
// }
  
  //Read the new data in and process the corresponding data
   while (mySerial.available()) 
  {
    JY901_1.CopeSerialData(mySerial.read()); //Call JY901 data cope function
  }

  delay(10);
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

