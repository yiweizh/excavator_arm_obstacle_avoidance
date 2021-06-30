#include <Wire.h>
#include <JY901.h>
#include <SoftwareSerial.h> 

// Define the Software serials
SoftwareSerial mySerial(10,11); //RX=10, TX=11;
// TODO: to add more


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
//Set up all the serials
void setup() 
{
  Serial.begin(9600);
  mySerial.begin(9600);
  
}

void loop() 
{
  if(Serial.available()){
    char read_in = Serial.read();
    if(read_in == 'd'){
      //Print out the data when the user pressed 'd'
      
      //print received data. Data was received in serialEvent;
      Serial.print("Time:20");Serial.print(JY901.stcTime.ucYear);Serial.print("-");Serial.print(JY901.stcTime.ucMonth);Serial.print("-");Serial.print(JY901.stcTime.ucDay);
      Serial.print(" ");Serial.print(JY901.stcTime.ucHour);Serial.print(":");Serial.print(JY901.stcTime.ucMinute);Serial.print(":");Serial.println((float)JY901.stcTime.ucSecond+(float)JY901.stcTime.usMiliSecond/1000);
                   
      Serial.print("Acc:");Serial.print((float)JY901.stcAcc.a[0]/32768*16);Serial.print(" ");Serial.print((float)JY901.stcAcc.a[1]/32768*16);Serial.print(" ");Serial.println((float)JY901.stcAcc.a[2]/32768*16);
      
      Serial.print("Gyro:");Serial.print((float)JY901.stcGyro.w[0]/32768*2000);Serial.print(" ");Serial.print((float)JY901.stcGyro.w[1]/32768*2000);Serial.print(" ");Serial.println((float)JY901.stcGyro.w[2]/32768*2000);
      
      Serial.print("Angle:");Serial.print((float)JY901.stcAngle.Angle[0]/32768*180);Serial.print(" ");Serial.print((float)JY901.stcAngle.Angle[1]/32768*180);Serial.print(" ");Serial.println((float)JY901.stcAngle.Angle[2]/32768*180);
      
      Serial.print("Mag:");Serial.print(JY901.stcMag.h[0]);Serial.print(" ");Serial.print(JY901.stcMag.h[1]);Serial.print(" ");Serial.println(JY901.stcMag.h[2]);
      
      Serial.print("Pressure:");Serial.print(JY901.stcPress.lPressure);Serial.print(" ");Serial.println((float)JY901.stcPress.lAltitude/100);
      
      Serial.print("DStatus:");Serial.print(JY901.stcDStatus.sDStatus[0]);Serial.print(" ");Serial.print(JY901.stcDStatus.sDStatus[1]);Serial.print(" ");Serial.print(JY901.stcDStatus.sDStatus[2]);Serial.print(" ");Serial.println(JY901.stcDStatus.sDStatus[3]);
      
      Serial.print("Longitude:");Serial.print(JY901.stcLonLat.lLon/10000000);Serial.print("Deg");Serial.print((double)(JY901.stcLonLat.lLon % 10000000)/1e5);Serial.print("m Lattitude:");
      Serial.print(JY901.stcLonLat.lLat/10000000);Serial.print("Deg");Serial.print((double)(JY901.stcLonLat.lLat % 10000000)/1e5);Serial.println("m");
      
      Serial.print("GPSHeight:");Serial.print((float)JY901.stcGPSV.sGPSHeight/10);Serial.print("m GPSYaw:");Serial.print((float)JY901.stcGPSV.sGPSYaw/10);Serial.print("Deg GPSV:");Serial.print((float)JY901.stcGPSV.lGPSVelocity/1000);Serial.println("km/h");
      
      Serial.println("");
    }
 }
  
  //Read the new data in and process the corresponding data
   while (mySerial.available()) 
  {
    JY901.CopeSerialData(mySerial.read()); //Call JY901 data cope function
  }

  delay(200);
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

