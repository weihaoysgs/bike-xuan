#include "bike_core/blance.h"
/*
该文件使用需要传入

两个电机的速度反馈、速度控制函数
陀螺仪读取函数、角度、角速度
舵机控制函数、舵机中值、最大值、最小值
开启一个1ms的定时中断

*/

extern float target_speed_;
extern float current_speed;

//零点
float Pitch_Zero=-2;                 //设置Pitch轴角度零点的误差值
float Zero_error = 0.0;              //Pitch校准后的值
float Pitch_error=0.0;               //实际-压弯
float angle_error=0.0;               //压弯角度

//陀螺仪
float   Pitch;                       //陀螺仪角度值
short   gyro;                        //陀螺仪角速度值

//开启标志位
int start_pid=0;                     //动量轮开启标志位
int start_speed=0;                   //前进和摄像头标志位

//电机
int Target_speed;                    //目标速度
short MotorDutyA = 0;                // 飞轮电机驱动占空比数值
short MotorDutyB = 0;                // 后轮电机驱动占空比数值
short BLDCduty;                      //无刷电机pwm
int target_s;                        //当前目标速度
short ECPULSE1;                      //动量轮反馈速度
short ECPULSE2;                      //后轮反馈速度

//舵机
signed short eduty=Servo_Center_Mid;
signed short duty_target=Servo_Center_Mid;;
float dy=1.0;                        //压弯模型动态参数
float cam_err;                       //摄像头反馈舵机打脚


//pid
float Tar_Ang_Vel_Y,Target_Angle_Y;
float Speed_out,Angle_out;


//快速cos
float _fast_cos[91] = { 1,
0.999848,0.999391,0.99863,0.997564,0.996195,0.994522,0.992546,0.990268,0.987688,0.984808,
0.981627,0.978148,0.97437,0.970296,0.965926,0.961262,0.956305,0.951057,0.945519,0.939693,
0.93358,0.927184,0.920505,0.913545,0.906308,0.898794,0.891007,0.882948,0.87462,0.866025,
0.857167,0.848048,0.838671,0.829038,0.819152,0.809017,0.798635,0.788011,0.777146,0.766044,
0.75471,0.743145,0.731354,0.71934,0.707107,0.694658,0.681998,0.669131,0.656059,0.642788,
0.62932,0.615661,0.601815,0.587785,0.573576,0.559193,0.544639,0.529919,0.515038,0.5,
0.48481,0.469471,0.45399,0.438371,0.422618,0.406737,0.390731,0.374606,0.358368,0.34202,
0.325568,0.309017,0.292372,0.275637,0.258819,0.241922,0.224951,0.207912,0.190809,0.173648,
0.156434,0.139173,0.121869,0.104528,0.0871556,0.0697563,0.0523358,0.0348993,0.0174522,-1.73205e-07};

float fast_cos(int angle)
{
    if (angle>=0 && angle <= 90)
    {
        return _fast_cos[angle];
    }
    else if (angle > 90 && angle <=180)
    {
        return -(_fast_cos[180-angle]);
    }
    else if (angle > 180 && angle <=360)
    {
        return fast_cos(360-angle);
    }
    else if (angle > 360)
    {
        return fast_cos(angle - 360);
    }
    else if (angle < 0)
    {
        return (fast_cos(-angle));
    }
    return 0;
}



/*
调参的个人经验：调参的话从内环调到外环，先调角速度环，再调角度环，最后调速度环。调内环的时候外环的输出乘0，比如角速度环，把目标角度乘0，调好之后把0去掉调角度环
，角度环也是这样。另外比较重要的是极性，极性可以一个环一个环试，不对的话就把PID的形参换一下或者加负号，总之就是让它PID误差反过来。角速度环车往那边到轮往那边转
，调PID参数到你左右轻摇车时明显感觉到阻尼就差不多了，角度环也是一样的，往那边到往那边转就对了，角度环调好它就能立了。重要的是车机械结构好，两边重量分布均匀
再者就是机械零点要找好。速度环极性判断，你把两个内环关了，然后用速度环的输出给电机，p先给大点，拨动动量轮，往那边剥越转越快说明极性对，转一下停了就是反的。速度环
调好就能走了
我的参数，PWM最大是10000，仅供参考哈
*/

//无刷参数  8.4

PID Ang_Vel_PID ={    //角速度环参数
                    .Kp=0.0,
                    .Ki=0.0,
                    .Kd=0.0
                 };
PID Angle_PID ={    //角度环
                    .Kp=0.0,
                    .Ki=0.0,
                    .Kd=0.0
               };
PID MOTOR_PID ={      //速度环
                    .Kp=0.0,
                    .Ki=0.0,
                    .Kd=0.0
               };

PID Speed_PID={
                    .Kp=0.0,
                    .Ki=0.0,
                    .Kd=0.0
              };





 int sloap(int target,int add)  //斜坡函数
 {
     int speed;
	   speed=target_s;
     if(target>speed)
     {
         speed+=add;
         if(speed>target)speed=target;
     }
     else
         if(target<speed)
         {
             speed-=add;
             if(speed<target)speed=target;
         }
     return speed;
 }

 signed short duty_sloap(signed short target,int add)  //舵机斜坡函数
 {
     signed short duty;
     duty=duty_target;
     if(target>Servo_Center_Max)target=Servo_Center_Max;
     else if(target<Servo_Center_Min)target=Servo_Center_Min;
     if(target>duty)
     {
         duty+=add;
         if(duty>target)duty=target;
     }
     else
         if(target<duty)
     {
         duty-=add;
         if(duty<target)duty=target;
     }
     return duty;
 }
 
float turn_model(signed short duty,short ecpulse)
{
    float angle=0.0;                         //转弯角度
    float speed=0.0;                         //速度
    //float L=0.193;                         //两轮轴距
    float error=0.0;
    float co=0.0;
    float sp2=0.0;
    duty=duty-Servo_Center_Mid;
    angle=duty*0.1;                          //单位度
    speed=ecpulse*0.009;                     //m/s
    sp2=speed*speed;

    co=fast_cos(90-angle);
    error=sp2*co*0.529*57.32;                //arctan 一阶展开够用
    return error;
}



//增量式pid
float IncP_DCalc(PID*sptr,float true,float target)
{
    float incout;
    float Error;

    Error=true-target;
    incout=sptr->Kp*(Error-sptr->LastError)+sptr->Ki*Error+sptr->Kd*(Error-2*sptr->LastError+sptr->LLastError);
    sptr->LLastError=sptr->LastError;
    sptr->LastError=Error;
    sptr->SumOut+=incout;
    return sptr->SumOut;

}

//位置式pid
float LocP_DCalc(PID*sptr,float true,float target,float imax,float imin,float limit_max,float limit_min)
{
    float iError,dError;
    float output;

    iError=true-target;  //偏差

    if(iError>limit_max&&iError<limit_min)iError=iError*0.1;

    sptr->SumError+=iError;            //积分(采样时间很短时，用一阶差分代替一阶微分，用累加代替积分)
    dError=(float)(iError-(sptr->LastError));     //微分
    sptr->LastError=iError;

    if(sptr->SumError>imax) sptr->SumError=imax;
    if(sptr->SumError<imin) sptr->SumError=imin;
    output=(float)(sptr->Kp*iError  //比例项
          +(sptr->Ki*sptr->SumError)//积分项
          +sptr->Kd*dError);        //微分项
    return(output);
}

//限幅函数
float range_protect(float num,float min,float max)
{
    if(num>=max)num=max;
    if(num<=min)num=min;
    return num;
}

//积分清零
void Integral_clear(void)
{
    Ang_Vel_PID.SumError=0;
    Ang_Vel_PID.SumOut=0;
    Ang_Vel_PID.LLastError=0;
    Ang_Vel_PID.LastError=0;

    Angle_PID.SumError=0;
    Angle_PID.SumOut=0;
    Angle_PID.LLastError=0;
    Angle_PID.LastError=0;

    MOTOR_PID.SumError=0;
    MOTOR_PID.SumOut=0;
    MOTOR_PID.LLastError=0;
    MOTOR_PID.LastError=0;

    Speed_PID.SumOut=0;
    Speed_PID.SumError=0;
    Speed_PID.LLastError=0;
    Speed_PID.LastError=0;

}

//串级的基本思想，最内环角速度环，内环角度环，外环速度环，外环的输出作为内环的输入，外环输出内环目标值。比如速度环输出的是角度环的目标角度，目标角度
//与实际解算角度做偏差进行PID计算输出角速度环，角度环和角速度环相同道理，最内环角速度环输出PWM控制电机。
void Balance_endocyclic()  //角速度最内环2ms中断
{
    static short gyro_last;
/*
	  修改
    LQ_DMP_Read();  //陀螺仪读取
*/
    Zero_error=Pitch-Pitch_Zero;
    gyro=gyro*0.8+gyro_last*0.2;
    gyro_last=gyro;

    MotorDutyA=LocP_DCalc(&Ang_Vel_PID,(-gyro*0.08),Tar_Ang_Vel_Y,2000,-2000,70,-70);//角速度环输出PWM控制电机
    MotorDutyA=range_protect(MotorDutyA,-8000,8000 );  //PWM输出限幅
//    if(MotorDutyA<-0) MotorDutyA -=100;    //死区（暂时不用，没必要用，要用的话自己测一下死区占空比）
//    else if(MotorDutyA>0) MotorDutyA+=100; //死区
//    if((MotorDutyA<100)&&(MotorDutyA>-100))
//    MotorDutyA=0;
    if(((Zero_error) > 20) || ((Zero_error) < -20))       //摔倒停车判断
    {
       start_speed=0;
       start_pid=0;
       MotorDutyA=0;
       MotorDutyB=0;
       Integral_clear();
       eduty=Servo_Center_Mid;
       duty_target=Servo_Center_Mid;
    }

    target_speed_=MotorDutyA;//无刷电机pwm


}


void Balance_outcyclic()  //角度内环10ms
{

    static short ECPULSE2_LAST;
/*
	  修改
    ECPULSE2 = ENC_GetCounter(ENC2_InPut_P33_7);//后轮反馈
*/
    ECPULSE2=ECPULSE2*0.3+ECPULSE2_LAST*0.7;
    ECPULSE2_LAST=ECPULSE2;
    if(start_speed==1)
    {
        target_s=sloap(Target_speed,3);  //缓启动
        MotorDutyB=IncP_DCalc(&Speed_PID,ECPULSE2,target_s);
    }
    else MotorDutyB=0;
    MotorDutyB=range_protect(MotorDutyB,-3000,3000);
/*
		修改
    Motor(MotorDutyB); //后轮电机输出
*/
    eduty=cam_err;                                   //不融合舵机
    duty_target=duty_sloap(eduty,15);
		
/*
		修改
    ATOM_PWM_SetDuty(ATOMSERVO2, duty_target, 100);  //舵机改变占空比
*/
		
    angle_error=turn_model(duty_target,ECPULSE2)*dy;  //压弯模型

    if(angle_error>20.0)angle_error=20.0;
    else if(angle_error<-20.0)angle_error=-20.0;
    Pitch_error=Zero_error-angle_error;

    Angle_out=LocP_DCalc(&Angle_PID,Pitch_error,Target_Angle_Y,100,-100,20,-20);  //角度环输出目标角速度
    Tar_Ang_Vel_Y=range_protect(Angle_out, -300, 300);//目标角速度限幅

}
void Speed_control()  //速度外环100ms
{

     ECPULSE1 = current_speed; // 动量轮编码器反馈

     Speed_out = LocP_DCalc(&MOTOR_PID,-ECPULSE1,3.0,200,-200,30,-30);  //速度环输出目标角度
     Target_Angle_Y=range_protect(Speed_out, -30, 30);//目标角度限幅
}
