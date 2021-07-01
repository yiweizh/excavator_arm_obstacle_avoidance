#ifndef __AutoControl_H
#define __AutoControl_H			 

//Pwm初始值
#define Big_Pwm_Init           11.5f //单位：%
#define Small_Pwm_Init         11.5f //7.22
#define Bucket_Pwm_Init        7.12f
#define Turn_Pwm_Init          7.50f
#define Source_Pwm_Init        5.00f

//Pwm最大值
#define Big_Pwm_Max           11.50f //单位：%
#define Small_Pwm_Max         10.50f
#define Bucket_Pwm_Max        10.05f
#define Turn_Pwm_Max          10.00f

//Pwm最小值
#define Big_Pwm_Min            5.00f //单位：%
#define Small_Pwm_Min          4.00f
#define Bucket_Pwm_Min         4.00f
#define Turn_Pwm_Min           5.00f

//Pwm死区
#define Big_Pwm_Deadzone       2.20f //单位：%
#define Small_Pwm_Deadzone     2.40f
#define Bucket_Pwm_Deadzone    2.35f
#define Turn_Pwm_Deadzone      0.70f

//PID
#define Big_Arm_Kp            0.000f
#define Small_Arm_Kp          0.000f
#define Bucket_Kp             0.001f
#define Turn_Table_Kp         0.005f
#define Big_Arm_Kd            0.000f
#define Small_Arm_Kd          0.000f
#define Bucket_Kd             0.000f

//挖斗水平位置
#define Bucket_Horizon           130 //单位：°

//稳态误差
#define Steady_State_Error         2.0f //单位：°

//臂长度
#define Big_Len                48.2f //单位：cm  
#define Small_Len              25.5f

//输入坐标点最远距离
#define Max_Len              4900.0f //单位：cm²  
#define Min_Len              1024.0f //单位：cm² 

//输入坐标点最大值
#define Max_x                  70.0f //单位：cm  
#define Max_y                  55.0f

//输入坐标点最小值
#define Min_x                 21.25f //单位：cm  
#define Min_y                 -42.5f



static float boom, arm, turn_table;

void  Pos_Calcu(float aim_x, float aim_y);
float PWM_Limit(float value, float val_max, float val_min);
void Get_Pos(float boom_angle, float arm_angle);

#endif
