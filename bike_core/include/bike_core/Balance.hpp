#ifndef BLANCE_H
#define BLANCE_H

#include <glog/logging.h>

#include <cmath>
#include <iomanip>
#include <iostream>
#define Servo_Center_Mid 1500  //舵机中值
#define Servo_Center_Max 2000  //舵机最大值
#define Servo_Center_Min 1000  //舵机最小值

struct PID {
  PID(float kp, float ki, float kd) : Kp(kp), Ki(ki), Kd(kd){};
  float SumOut;    //增量式out
  float SumError;  //误差累积

  float Kp;  //比例系数
  float Ki;  //积分系数
  float Kd;  //微分系数

  float LastError;   //上一次误差
  float LLastError;  //上上次误差
};

void Balance_endocyclic();
void Balance_outcyclic();
void Speed_control();
void Integral_clear(void);

#endif
