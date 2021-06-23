#include <HardwareSerial.h>
#include "AutoControl.h"

#include "math.h"

//Constraint the PWM value
float PWM_Limit(float value, float val_max, float val_min)
{
  if(value > val_max) value = val_max;
  if(value < val_min) value = val_min;
  return value;
}

// Calculate the position using the angles 
void Get_Pos(float boom_angle, float arm_angle)
{
  float x = Big_Len * cos((boom_angle + 16)/57.2978f) + Small_Len * cos(arm_angle/57.2978f);
  float y = Big_Len * sin((boom_angle + 16)/57.2978f) + Small_Len * sin(arm_angle/57.2978f);
  // Interesting ... what and why is 57.2978 ?

  Serial.print("boom_angle = "); Serial.println((boom_angle-16));
  Serial.print("small_angle = "); Serial.println(arm_angle);
  
  Serial.print("x = "); Serial.println(x);
  Serial.print("y = "); Serial.println(y);
}



//Calculating the arm angles reversely using the coordinates 坐标点逆解算臂角度
int err=0;
void Pos_Calcu(float aim_x, float aim_y)
{
	float big_angle_rad, small_angle_rad, len, tempA, tempB, tempC, errX, errY;
  
	if(aim_y==0) aim_y = 0.001f;
	len = aim_x * aim_x + aim_y * aim_y;

	if((len>Max_Len||len<Min_Len) || (aim_x>Max_x||aim_x<Min_x) || (aim_y>Max_y||aim_y<Min_y)) err = 1;

	tempA = len - Big_Len * Big_Len + Small_Len * Small_Len;
	tempB = (-aim_x * sqrt((len-(Big_Len-Small_Len)*(Big_Len-Small_Len)) * ((Big_Len+Small_Len)*(Big_Len+Small_Len)-len)) + tempA * aim_y)/len;
	tempC = (tempA-aim_y*tempB)/(2*Small_Len*aim_x);

	big_angle_rad   = asin((aim_y - 0.5f*tempB)/Big_Len);
	if(tempC>0) small_angle_rad = asin(0.5f*tempB/Small_Len);
	else        small_angle_rad = - asin(0.5f*tempB/Small_Len) - 3.1415926f;

	boom = big_angle_rad   * 57.29578f - 16;
	arm = small_angle_rad * 57.29578f; //57.29578=180/pi;

  //Constrain the pwm out
  boom = PWM_Limit(boom, Big_Pwm_Max, Big_Pwm_Min);
  arm =  PWM_Limit(arm, Small_Pwm_Max, Small_Pwm_Min); 

  
	errX = Big_Len * cos(big_angle_rad) + Small_Len * cos(small_angle_rad) - aim_x;
	errY = Big_Len * sin(big_angle_rad) + Small_Len * sin(small_angle_rad) - aim_y;
	if(fabs(errX) > 1 || fabs(errY) > 1) err = 2;
  if(err>0) {Serial.print("ERROR:"); Serial.println(err);}
}






