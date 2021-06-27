/* Sweep
 by BARRAGAN <http://barraganstudio.com>
 This example code is in the public domain.

 modified 8 Nov 2013
 by Scott Fitzgerald
 http://www.arduino.cc/en/Tutorial/Sweep
*/
#define Serial_port 115200
#include <Servo.h>

Servo myservo_boom;  // create servo object to control a servo
Servo myservo_stick;
// twelve servo objects can be created on most boards

int pos_boom = 90;    // variable to store the servo 
int pos_stick = 90;
int max_pos = 180;
int incr = 45;
int pin_boom = 8;
int pin_stick = 9;


void setup() {
  myservo_boom.attach(pin_boom);  // attaches the servo on pin 9 to the servo object
  myservo_stick.attach(pin_stick); // 
  Serial.begin(Serial_port);
  while (!Serial) {
      ; // wait for serial port to connect. Needed for Native USB only
    }
}

void loop() {

  if(Serial.available()){
    char read_in = Serial.read();
    


      if(read_in == 'a'){
        pos_boom += incr;
        if(pos_boom >= max_pos){
          pos_boom = max_pos;
          }
        Serial.print("boom: ");
        Serial.print(pos_boom);
        Serial.print("\n");
        }

      else if(read_in == 'd'){
        pos_boom -= incr;
        if(pos_boom <= 0){
          pos_boom = 0;
          }
        Serial.print("boom: ");
        Serial.print(pos_boom);
        Serial.print("\n");
        }

      else if(read_in == '+'){
        pos_stick += incr;
        if(pos_stick >= max_pos){
          pos_stick = max_pos;
          }
        Serial.print("stick: ");
        Serial.print(pos_stick);
        Serial.print("\n");
        }

      else if(read_in == '-'){
        pos_stick -= incr;
        if(pos_stick <= 0){
          pos_stick = 0;
          }
        Serial.print("stick: ");
        Serial.print(pos_stick);
        Serial.print("\n");
        }


    
     myservo_boom.write(pos_boom);
     myservo_stick.write(pos_stick);
     
        
      
    }
  delay(15);
}
