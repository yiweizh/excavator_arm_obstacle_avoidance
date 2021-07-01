// arduino interface that
// - receives control commands for base, boom servo, stick servo and pump from other nodes
// - apply these commands onto the excavator joints

// Authors: Shiji Liu, JInjie Liu


#include <ros.h>
#include "motion_planning/excavator_control_signals.h"

// custom function definition
// callback for target angle subscriber

void excavator_control_callback(const motion_planning::excavator_control_signals& msg);

// ros initialization
ros::NodeHandle_<ArduinoHardware, 1, 1, 128, 128>  nh;
ros::Subscriber<motion_planning::excavator_control_signals> excavator_control_subscriber("excavator_control",&excavator_control_callback);//subscriber



// create variables
unsigned int   boom_servo;  // boom servo angle, valid values: 45,90,135
unsigned int   stick_servo; // boom servo angle, valid values: 45,90,135
unsigned int   pump_mode; // pwm mode, 0 for stop, 1 for normal operation, 2 for special operation (used when the pump motor may be stuck due to hight pressure)
float base_pwm; // pwm for base



// custom functions




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
  
}

void loop() {
  // put your main code here, to run repeatedly:
  
  
  nh.spinOnce();
  delay(10);
}
