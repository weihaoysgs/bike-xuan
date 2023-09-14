# Bike-XUAN-HLL
> Author: weihaoysgs@gmail.com <br>
> Webset: www.weihaoysgs.com

<div align=center>
<img src="./images/bike-xuan-2.jpg" width="500px">
</div>

$\quad$ ������Ŀ��Ҫ�������ɻӾ� `XUAN` ���г���������ļ��������ϴ���һ���Ĳ�𣬻�е�ṹ�϶Զ����ֵ�֧�Ż���������СС�ĸĽ���ȥ����ԭ�ȿ��Ի�Ľṹ�������������ʹ�õ��� `odrive`�����������Ƴ������ǻ��� `ROS` ��һͨ���м������� `CAN` ����ֱ�Ӻ�NUC���е����ݽ�����û��ʹ�õ����ĵ�Ƭ��ȥ����������������㷨ʹ�õ��Ǿ������� `PID` ���ơ�ͬʱӵ�л����Ӿ��ļ��ױ���ϵͳ��������������Ч��������ʾ��

<table>
    <tr>
        <td><center><img src="./images/run1.gif" width = 78%></center><center>avoid obstacle</center></td>
        <td><center><img src="./images/run2.gif" width = 95%></center><center>go straight</center></td>
    </tr>
</table>



$\quad$ ͬʱֵ��һ������� `AI` ����ģ����ʹ�õ����Ǻ��ɻӾ�ͬ��Ļ�Ϊ���� `Atlas 200 DK` ����оƬ����оƬ��ȫ����ѧ�����������������������췽���й�����ƹ������о�Ժ������

<div align=center>
<img src="./images/atlas_200_wh.jpg" width="300px">
</div>


## 1.0 ��ҪӲ��ѡ��

<center>

|           ����           |          �ͺ�           |
| :----------------------: | :---------------------: |
|           ���           | `DJI Roomaster 6S `��� |
|          ������          |       `Intel NUC`       |
| �����ֵ��������������� |      `X8018` ���       |
|          `IMU`           |       `IMU-CH100`       |
|       ��ˢ�������       |        `odrive`         |
|           ���           |     `60KG` ���ڶ��     |
|          ����ͷ          |        `Astra S`        |
|      ���ֻ�е�ṹ��      |  ��֬��ӡ���߽����ӹ�   |
|          ���г�          |          ����           |
|          ң����          | `DJI Robomaster`ң����  |

</center>


## 2.0 ���Ŀ��ƽڵ����

- `can_send_receive_node`

  �ýڵ���Ҫ����ʱ���� `CAN` ��� `Odrive` ʹ�� `Odrive` ���Իش�������ٶȺ�λ����Ϣ�� `NUC`�����������յ������ݽ����󷢲��� `ROS` ��ȥ��

- `parser_remote_data` 

  �ýڵ㸺����� `DJI` ң�����������ݣ������н��������� `ROS` ��ȥ��

- `bike_xuan_control_node`

  �ýڵ���Ϊ����ϵͳ�ĺ��Ľڵ㣬��������Ŀ����㷨����ͨ�� `CAN` ���߽�������Ϣ���͸� `Odrive` ʵ��ƽ�⡣

## 3.0 ���ٿ�ʼ

- ���� `IMU`

```shell
roslaunch imu_launch imu_msg.launch
```

- ����ң���������ڵ�

```shell
rosrun bike_core parser_remote_data_node
```

- ���� `CAN` ��Ϣ���ͽڵ�

```shell
 rosrun bike_core can_send_receive_node
```

- �������Ŀ��ƽڵ�

```shell
 rosrun bike_core bike_xuan_core_control_node
```

## 4.0 ��������

$\quad$ ����Ŀ�ж����г�����̬�����㷨��ʹ�õ������� `PID` �����㷨�����⵽���������ٶȻ����ǶȻ������ٶȻ�û������˳��������ڶ����ԭ����е��ԣ�����Ŀ�г���ƽ��� `PID` ����������ʾ��ͨ������д�� `YAML` �ļ��У����ļ��ɷ���ı������

```yaml
%YAML:1.0
---
Pid.Name: bike_angle_pid
Kp: 5.213
Ki: 0.015
Kd: -0.15
CalculateTime: 90
Debug: 0
Integal.Limit: 1.0
Use.Integal.Limit: 1
Output.Limit: 10.0
Use.Output.Limit: 1
```

```yaml
%YAML:1.0
---
Pid.Name: bike_angle_velocity_pid
Kp: 420.0
Ki: 0.21
Kd: -92.1
CalculateTime: 30
Debug: 0
Integal.Limit: 200.0
Use.Integal.Limit: 1
Output.Limit: 2000.0
Use.Output.Limit: 1
```

```yaml
%YAML:1.0
---
Pid.Name: bike_speed_pid
Kp: -0.001
Ki: 0.000
Kd: -0.000
CalculateTime: 480
Debug: 0
Integal.Limit: 2.0
Use.Integal.Limit: true
Output.Limit: 6.0
Use.Output.Limit: true
```

