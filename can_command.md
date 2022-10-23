# Can  Bus Config Odirve

## 常用命令

- 电机编码器校准，正转一圈反转一圈

```shell
cansend can0 207#0700000000000000
```

- 检测电机参数，在该命令大约 2s 后执行后会听到电机"哔"的一声。

```shell
cansend can0 207#0400000000000000
```

> 如果是电机启动后进行的编码器校准，则校准完成后电机不会进入闭环模式，需要再次输入命令手动开启闭环。

- 开启电机的闭环模式

```shell
cansend can0 207#0800000000000000
```

- 重启 odrive

```shell
cansend can0 216#0000000000000000
```

- 通过 can 指令指控点击速度
```shell
cansend can0 20C# 00 00 00 00 07 D0 00 00 给点击速度指令
// data.at(4) = 0x07;
// data.at(5) = 0xD0;
```

## odrive 中的几个比较重要的参数设置

- 在速度控制模式下，该参数控制的是电机的最大转速，单位为 (转$/s$)
<pre>odrv0.axis0.controller.config.vel_limit = <font color="#008700">20</font></pre>
- 该参数控制的是电机的加速度的大小，当加速度过小的时候，是没办法达到一个及时相应的。
<pre>odrv0.axis0.controller.config.vel_ramp_rate = <font color="#008700">1000</font></pre>
- 该参数控制的是电机速度环的一个 kp 值，但初始化一般为 0.2 具体要根据自己的系统进行调节。
<pre>odrv0.axis0.controller.config.vel_gain = <font color="#008700"> 0.3</font></pre>
- 该参数控制的是 odrive 运行时的电流限制大小
<pre>odrv0.axis0.motor.config.current_lim = <font color="#008700">12</font></pre>
> 注意，按照道理来说，如果想要达到一个非常高的电机响应，上面的几个值都应该是比较大的，但是本人在配置的时候却发现过大容易出现电机脱离闭环，触发 odrive 的内部电流保护机制，此时将 current_limit 调低能够有所缓解。

## Erros

- MotorError.CURRENT_SENSE_SATURATION

  莫名其妙点击停止转动，突然停止

- AxisError.DC_BUS_UNDER_VOLTAGE

  莫名其妙点击停止转动

- MotorError.DRV_FAULT

- MotorError.CURRENT_LIMIT_VIOLATION

  闭环模式下，手动强行转动电机

- **socket can error write: No buffer space available**

```shell
1、查看
root@socfpga:/sys/class/net/can0# cat tx_queue_len
10
2、root@socfpga:/sys/class/net/can0# echo 4096 > tx_queue_len
3、root@socfpga:/sys/class/net/can0# cat tx_queue_len
4096
4、再运行程序
```
## Reference

- https://blog.csdn.net/gjy_skyblue/article/details/115412902 （速度模式配置）
- https://blog.csdn.net/gjy_skyblue/article/details/115134590  (电流环配置)

float angle_pid_out =
        bike_pid(balance_roll_angle, radian2angle(imu_ch100_pose_ptr_->roll_),
                 PidParams::POSITION, bike_pid.getAnglePid(), false);

    float angle_vel_pid_out =
        bike_pid(angle_pid_out, gyro_msg.x, PidParams::POSITION,
                 bike_pid.getAngleVelocityPid(), true);



```c++
// bike pour left the error is positive
float roll_angle_error =
balance_roll_angle - radian2angle(imu_ch100_pose_ptr_->roll_);
// 车子向右倾斜会输出正的角速度，向左会输出负的角速度
float gyro_speed_error = gyro_msg.x;
// 电机向右转速度为负数，向左转速度为正数
current_speed = odrive_can_parsed_msg_ptr_->speed;
```

## 小小的错误记录一下，出现了如下报错
在使用商家给的程序的时候在 `/usr/local/include/CSerialPort/SerialPort_global.h` 下有如下定义
```c++
typedef long long int64; ///< 64 bit signed unix 定义Unix int64
```
但是在这个程序中我又使用了 `opencv` 
因此出现了如下报错
```shell
n file included from /usr/include/opencv2/core/cvdef.h:67:0,
                 from /usr/include/opencv2/core.hpp:52,
                 from /usr/include/opencv2/opencv.hpp:52,
                 from /home/hll/code_space/bike_ws/src/bike_core/include/bike_core/OdriveMotorConfig.hpp:9,
                 from /home/hll/code_space/bike_ws/src/bike_core/include/bike_core/ServoControl.hpp:12,
                 from /home/hll/code_space/bike_ws/src/bike_core/src/ServoControl.cpp:1:
/usr/include/opencv2/core/hal/interface.h:57:20: error: conflicting declaration ‘typedef int64_t int64’
    typedef int64_t int64;
                    ^~~~~
In file included from /usr/local/include/CSerialPort/SerialPort.h:19:0,
                 from /home/hll/code_space/bike_ws/src/bike_core/include/bike_core/ServoControl.hpp:9,
                 from /home/hll/code_space/bike_ws/src/bike_core/src/ServoControl.cpp:1:
/usr/local/include/CSerialPort/SerialPort_global.h:30:19: note: previous declaration as ‘typedef long long int int64’
 typedef long long int64; ///< 64 bit signed unix 定义Unix int64
                   ^~~~~
In file included from /usr/include/opencv2/core/cvdef.h:67:0,
                 from /usr/include/opencv2/core.hpp:52,
                 from /usr/include/opencv2/opencv.hpp:52,
                 from /home/hll/code_space/bike_ws/src/bike_core/include/bike_core/OdriveMotorConfig.hpp:9,
                 from /home/hll/code_space/bike_ws/src/bike_core/include/bike_core/ServoControl.hpp:12,
                 from /home/hll/code_space/bike_ws/src/bike_core/test/servo_control_test_node.cpp:1:
/usr/include/opencv2/core/hal/interface.h:57:20: error: conflicting declaration ‘typedef int64_t int64’
    typedef int64_t int64;
                    ^~~~~
In file included from /usr/local/include/CSerialPort/SerialPort.h:19:0,
                 from /home/hll/code_space/bike_ws/src/bike_core/include/bike_core/ServoControl.hpp:9,
                 from /home/hll/code_space/bike_ws/src/bike_core/test/servo_control_test_node.cpp:1:
/usr/local/include/CSerialPort/SerialPort_global.h:30:19: note: previous declaration as ‘typedef long long int int64’
 typedef long long int64; ///< 64 bit signed unix 定义Unix int64
                   ^~~~~
In file included from /usr/include/opencv2/core/cvdef.h:67:0,
                 from /usr/include/opencv2/core.hpp:52,
                 from /usr/include/opencv2/opencv.hpp:52,
                 from /home/hll/code_space/bike_ws/src/bike_core/include/bike_core/OdriveMotorConfig.hpp:9,
                 from /home/hll/code_space/bike_ws/src/bike_core/include/bike_core/ServoControl.hpp:12,
                 from /home/hll/code_space/bike_ws/src/bike_core/src/ServoControl.cpp:1:
/usr/include/opencv2/core/hal/interface.h:57:20: error: conflicting declaration ‘typedef int64_t int64’
    typedef int64_t int64;
                    ^~~~~
```
大概意思就是多个地方对 int64 的定义冲突了。
因此我将商家给的程序部分的 `long long` 改为了一个 `long` 使用