#include <ros.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include "JY901.h"
#include "motion_planning/joint_angle_control.h"

// enum for joint choice
enum JointChoice{
  base,
  boom,
  stick,
  };

// custom function definition

// callback for target angle subscriber
void target_angle_callback(const motion_planning::joint_angle_control& msg);

// function to read a specific angle
float read_angle(JointChoice choice);


// ros initialization
ros::NodeHandle  nh;

motion_planning::joint_angle_control measured_angle; // measured angle from sensor readings

ros::Publisher measured_angle_publisher("measured_angles", &measured_angle);//publisher
ros::Subscriber<motion_planning::joint_angle_control> target_angle_subscriber("target_angles",&target_angle_callback);//subscriber


// initialize variables
JointChoice joint_choice = base;
float angle_real = 0.0; // angle derived by angle sensor
float target_angle = 0.0; // target
float tolerance = 2.0;// max allowed difference between angle_real and target_angle
bool working = false; // whether we are working or not
/*float angle_boom_real = 0.0;
float angle_stick_real = 0.0;*/

// variables for sensor readings
// Define the Software serials
SoftwareSerial serial0(8,9); //RX=8, TX=9;
SoftwareSerial serial1(10,11); //RX=10, TX=11;

static float stick_angle;
static float boom_angle;
float read_stick();
float read_boom();

// custom functions


void target_angle_callback(const motion_planning::joint_angle_control& msg){

     // loginfo
     char log_msg[128];

     // get joint choice:
     if(msg.joint_name == "base"){
        joint_choice = base;
        measured_angle.joint_name = "base"; 
        sprintf(log_msg,"choice: base ");
      }

      else if(msg.joint_name == "boom"){
        joint_choice = base;
        measured_angle.joint_name = "boom";
        sprintf(log_msg,"choice: boom ");
      }

      else if(msg.joint_name == "stick"){
        joint_choice = base;
        measured_angle.joint_name = "stick";
        sprintf(log_msg,"choice: stick ");
      }

     nh.loginfo(log_msg);

     target_angle = msg.angle;
     working = true;
  
     // log info, only 1 angle in msg
     char angle_char[32];
     char target_char[32];

     dtostrf(msg.angle,6,2,angle_char);
     dtostrf(angle_real,6,2,target_char);

     sprintf(log_msg, "target: %s, angle_real: %s\n",angle_char,target_char);
     
     nh.loginfo(log_msg);
     
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

// read specific angles
float read_angle(JointChoice choice){
  
  
  
  }




void setup() {
  // put your setup code here, to run once:
  // ros related initialization
  nh.initNode();
  nh.advertise(measured_angle_publisher);
  nh.subscribe(target_angle_subscriber);

  
  serial0.begin(57600);
  serial1.begin(57600);

  
}

void loop() {
  // put your main code here, to run repeatedly:
  measured_angle.angle = angle_real;

  if(working && fabs(angle_real - target_angle) <= tolerance){ // we have been ordered to work
    
    working = false;
    }


  if(working){
    angle_real += 0.5;
    }
  
  
  
  measured_angle_publisher.publish(&measured_angle);
  nh.spinOnce();
  delay(10);
}
