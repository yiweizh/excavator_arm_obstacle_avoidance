// arduino interface that
// - receives control commands for base, boom servo, stick servo and pump from other nodes
// - apply these commands onto the excavator joints

// Authors: Shiji Liu, JInjie Liu


#include <ros.h>
#include "motion_planning/excavator_control_signals.h"
#include <Servo.h> 
#include "AutoControl.h"


// output pins
#define PUMP_PIN 11
#define BASE_PWM_PIN 10
#define BOOM_PIN 8
#define STICK_PIN 9
#define BASE_IN1 7
#define BASE_IN2 6



// custom function definition
// callback for exacavator control subscriber

void excavator_control_callback(const motion_planning::excavator_control_signals& msg);

// ros initialization
ros::NodeHandle_<ArduinoHardware, 1, 1, 128, 128>  nh;
ros::Subscriber<motion_planning::excavator_control_signals> excavator_control_subscriber("excavator_control",&excavator_control_callback);//subscriber



// create variables
float   boom_servo;  // boom servo angle, valid values: 45,90,135
float   stick_servo; // boom servo angle, valid values: 45,90,135
float   pump_mode; // pwm mode, 0 for stop, 1 for normal operation, 2 for special operation (used when the pump motor may be stuck due to hight pressure)
float   base_pwm; // pwm for bases

float Source_Pwm_Out = Source_Pwm_Init;

// servos
Servo PUMP_ESC;
Servo BASE_ESC;
Servo Boom_SERVO;
Servo STICK_SERVO;


// custom functions
void controlBoomServo(){
  Boom_SERVO.write(boom_servo);
  }


void controlStickServo(){
  STICK_SERVO.write(stick_servo);
  }

void controlPUMP(){
  // three cases
  if(pump_mode == 2.0){
    Source_Pwm_Out = 6.0;
    PUMP_ESC.writeMicroseconds(200.0*Source_Pwm_Out);
    delay(2000);
    PUMP_ESC.writeMicroseconds(200.0*Source_Pwm_Init);
    delay(500);
    }
  else if(pump_mode == 1.0){
    Source_Pwm_Out = 6.0;
    PUMP_ESC.writeMicroseconds(200.0*Source_Pwm_Out);
    }
  else{
    Source_Pwm_Out = 5.0;
    PUMP_ESC.writeMicroseconds(200.0*Source_Pwm_Out);
    }
    
  }

void controlBase(){
  float pwm_limit = 0.0;
  if (base_pwm > pwm_limit){
    BASE_ESC.writeMicroseconds(200.0*base_pwm);
    // turn left, positive
    digitalWrite(BASE_IN1, LOW);
    digitalWrite(BASE_IN2, HIGH);
    }
  else if(base_pwm <= -pwm_limit){
    BASE_ESC.writeMicroseconds(200.0*base_pwm);
    // turn right, negative
    digitalWrite(BASE_IN1, HIGH);
    digitalWrite(BASE_IN2, LOW);
    }
  else{
    analogWrite(BASE_PWM_PIN,0);
    // stop turning
    digitalWrite(BASE_IN1, LOW);
    digitalWrite(BASE_IN2, LOW);
    }
  
  
  }

void excavator_control_callback(const motion_planning::excavator_control_signals& msg){

     // loginfo
     char log_msg[256];
  
     
     char boom_servo_char[32];
     char stick_servo_char[32];
     char pump_mode_char[32];
     char base_pwm_char[32];
     
     dtostrf((float)msg.boom_servo,6,2,boom_servo_char);
     dtostrf((float)msg.stick_servo,6,2,stick_servo_char);
     dtostrf((float)msg.pump_mode,6,2,pump_mode_char);
     dtostrf(msg.base_pwm,6,2,base_pwm_char);
     sprintf(log_msg, "boom_servo: %s, stick_servo: %s, pump_mode: %s, base_pwm_char: %s",boom_servo_char,stick_servo_char,pump_mode_char, base_pwm_char);
     
     nh.loginfo(log_msg);
     
     boom_servo = msg.boom_servo;
     stick_servo = msg.stick_servo;
     pump_mode = msg.pump_mode;
     base_pwm = msg.base_pwm;
     
  }




void setup() {
  // put your setup code here, to run once:
  // ros related initialization
  nh.initNode();
  nh.subscribe(excavator_control_subscriber);



  // initialize variables
  boom_servo = 90; // servo at center, close the tube
  stick_servo = 90; // servo at center, close the tube
  pump_mode = 0; // stop the pump
  base_pwm = 0.0; // no base motion

  PUMP_ESC.attach(PUMP_PIN);
  BASE_ESC.attach(BASE_PWM_PIN);
  Boom_SERVO.attach(BOOM_PIN);
  STICK_SERVO.attach(STICK_PIN);
  pinMode(BASE_IN1,OUTPUT);
  pinMode(BASE_IN2,OUTPUT);

  // apply control
  controlBoomServo();
  controlStickServo();
  controlPUMP();
  controlBase();
}

void loop() {
  // put your main code here, to run repeatedly:
  
  // apply control
  controlBoomServo();
  controlStickServo();
  controlPUMP();
  controlBase();

  
  nh.spinOnce();
  delay(10);
}
