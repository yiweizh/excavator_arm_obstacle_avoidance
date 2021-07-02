// arduino interface for getting the joint angles of
// excavator base, boom and stick
// and sending those angles through rosserial interface

// Authors:
// Shiji Liu, Xun Tu

// ros includes
#include <ros.h>
#include "motion_planning/excavator_angles.h"

// JY901 sensor includes
#include <Wire.h>
#include "JY901.h"
#include <SoftwareSerial.h>


// ros initialization
ros::NodeHandle_<ArduinoHardware, 1, 1, 128, 128>  nh;

/*std_msgs::Int16 angle_base;
std_msgs::Int16 angle_boom;
std_msgs::Int16 angle_stick;*/

/*ros::Publisher base_angle_publisher("base_angle",&angle_base);
ros::Publisher boom_angle_publisher("boom_angle",&angle_boom);
ros::Publisher stick_angle_publisher("stick_angle",&angle_stick);*/

motion_planning::excavator_angles measured_angle; // measured angle from sensor readings
ros::Publisher measured_angle_publisher("measured_angles", &measured_angle);//publisher

// Define the Software serials
SoftwareSerial serial0(8,9); //RX=8, TX=9;
SoftwareSerial serial1(10,11); //RX=10, TX=11;

CJY901 JY901_0 = CJY901();
CJY901 JY901_1 = CJY901();

// custom functions

// get joint angle of stick
float read_stick(){
  serial0.listen();
  delay(100);
   while (serial0.available()) 
  {
    JY901_0.CopeSerialData(serial0.read()); //Call JY901 data cope function
  }
  return -((float)JY901_0.stcAngle.Angle[1]/32768*180);
}

// get joint angle of boom
float read_boom(){
  serial1.listen();
  delay(100);
  while (serial1.available()){
    JY901_1.CopeSerialData(serial1.read());
  }
  return -((float)JY901_1.stcAngle.Angle[1]/32768*180);
}

// get joint angle of base
float read_base(){
  serial1.listen();
  delay(100);
  while (serial1.available()){
    JY901_1.CopeSerialData(serial1.read());
  }
  return -((float)JY901_1.stcAngle.Angle[2]/32768*180);
}

void setup() {
  // put your setup code here, to run once:
  // ros related initialization
  nh.initNode();
  nh.advertise(measured_angle_publisher);
  /*nh.advertise(base_angle_publisher);
  nh.advertise(boom_angle_publisher);
  nh.advertise(stick_angle_publisher);*/

  // initialize software uart
  serial0.begin(9600);
  serial1.begin(9600);

  measured_angle.angle_base = 0;
  measured_angle.angle_boom = 0;
  measured_angle.angle_stick = 0;

  /*angle_base.data = 0;
  angle_boom.data = 0;
  angle_stick.data = 0;*/
}

void loop() {
  
  // put your main code here, to run repeatedly:
  float angle_base = read_base();
  float angle_boom = read_boom();
  float angle_stick = read_stick();

  measured_angle.angle_base = angle_base;
  measured_angle.angle_boom = angle_boom;
  measured_angle.angle_stick = angle_stick;
  measured_angle_publisher.publish(&measured_angle);

  char log_msg[128];

  char angle_base_char[32];
  dtostrf(angle_base,6,2,angle_base_char);
  
  char angle_boom_char[32];
  dtostrf(angle_boom,6,2,angle_boom_char);
  
  char angle_stick_char[32];
  dtostrf(angle_stick,6,2,angle_stick_char);

  sprintf(log_msg, "angle_base: %s, angle_boom: %s, angle_stick: %s",angle_base_char,angle_boom_char,angle_stick_char);
  nh.loginfo(log_msg);
  /*angle_base.data = 0;
  angle_boom.data = 0;//read_boom();
  angle_stick.data = 0;//read_stick();
  base_angle_publisher.publish(&angle_base);
  boom_angle_publisher.publish(&angle_boom);
  stick_angle_publisher.publish(&angle_stick);*/
  
  
  nh.spinOnce();
  delay(10);
}
