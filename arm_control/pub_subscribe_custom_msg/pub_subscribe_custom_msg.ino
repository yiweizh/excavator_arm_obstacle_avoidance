#include <ros.h>
#include "motion_planning/excavator_angles.h"

// custom function definition
// callback for target angle subscriber

void target_angle_callback(const motion_planning::excavator_angles& msg);

// ros initialization
ros::NodeHandle  nh;

motion_planning::excavator_angles measured_angle; // measured angle from sensor readings

ros::Publisher measured_angle_publisher("measured_angles", &measured_angle);//publisher
ros::Subscriber<motion_planning::excavator_angles> target_angle_subscriber("target_angles",&target_angle_callback);//subscriber




// initialize variables
float init_value = 0.0;
float angle0 = 0.0;
float angle1 = 0.0;
float angle2 = 0.0;

// custom functions


void target_angle_callback(const motion_planning::excavator_angles& msg){

     // loginfo
     char log_msg[128];
  
     // only 3 angle in msg
     char angle1_char[32];
     char angle2_char[32];
     char angle3_char[32];
     dtostrf(msg.angle0,6,2,angle1_char);
     dtostrf(msg.angle1,6,2,angle2_char);
     dtostrf(msg.angle2,6,2,angle3_char);
     sprintf(log_msg, "angle 1: %s, angle 2: %s, angle 3: %s",angle1_char,angle2_char,angle3_char);
     
     nh.loginfo(log_msg);
     angle0 = msg.angle0;
     angle1 = msg.angle1;
     angle2 = msg.angle2;
  }




void setup() {
  // put your setup code here, to run once:
  // ros related initialization
  nh.initNode();
  nh.advertise(measured_angle_publisher);
  nh.subscribe(target_angle_subscriber);
}

void loop() {
  // put your main code here, to run repeatedly:
  measured_angle.angle0 = angle0;
  measured_angle.angle1 = angle1;
  measured_angle.angle2 = angle2;
  measured_angle_publisher.publish(&measured_angle);
  
  nh.spinOnce();
  delay(10);
}
