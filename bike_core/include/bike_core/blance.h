#ifndef _BLANCE_H_
#define _BLANCE_H_

#define Servo_Center_Mid 1500  //舵机中值
#define Servo_Center_Max 2000  //舵机最大值
#define Servo_Center_Min 1000  //舵机最小值

typedef struct
{
  float SumOut;    //增量式out
  float SumError;  //误差累积

  float Kp;  //比例系数
  float Ki;  //积分系数
  float Kd;  //微分系数

  float LastError;   //上一次误差
  float LLastError;  //上上次误差
} PID;

void Balance_endocyclic();
void Balance_outcyclic();
void Speed_control();

#endif
