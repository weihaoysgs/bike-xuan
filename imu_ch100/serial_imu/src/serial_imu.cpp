//serial_imu.cpp
#include <ros/ros.h>
#include <serial/serial.h>
#include <iostream>
#include <stdio.h>
#include <sensor_msgs/Imu.h>

#ifdef __cplusplus 
extern "C"{
#endif

#include <stdint.h>
#include <stdbool.h>

#include "ch_serial.h"

#define IMU_SERIAL   "/dev/ttyUSB1"
#define BAUD         (115200)
#define GRA_ACC      (9.8)
#define DEG_TO_RAD   (0.01745329)
#define BUF_SIZE     1024

void publish_imu_data(raw_t *data, sensor_msgs::Imu *imu_data);

#ifdef __cplusplus
}
#endif

static raw_t raw;
ros::Publisher IMU_pub;
serial::Serial sp;
sensor_msgs::Imu imu_data;

void callback(const ros::TimerEvent& event)
{
	int rev = 0;
	size_t num = sp.available();
	if(num!=0)
	{
		uint8_t buffer[BUF_SIZE]; 

		if(num > BUF_SIZE)
			num = BUF_SIZE;

		num = sp.read(buffer, num);
		if(num > 0)
		{
			imu_data.header.stamp = ros::Time::now();
			imu_data.header.frame_id = "base_link";

			for (int i = 0; i < num; i++)
			{
				rev = ch_serial_input(&raw, buffer[i]);

				if(raw.item_code[raw.nitem_code - 1] != KItemGWSOL)
				{
					if(rev)
					{
						publish_imu_data(&raw, &imu_data);
						IMU_pub.publish(imu_data);
					}
				}
			}
		}
	}
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "serial_imu");
	ros::NodeHandle n;

    IMU_pub = n.advertise<sensor_msgs::Imu>("/IMU_data", 20);

	serial::Timeout to = serial::Timeout::simpleTimeout(100);

	sp.setPort(IMU_SERIAL);

	sp.setBaudrate(BAUD);

	sp.setTimeout(to);
	

	try
	{
		sp.open();
	}
	catch(serial::IOException& e)
	{
		ROS_ERROR_STREAM("Unable to open port.");
		return -1;
	}
    
	if(sp.isOpen())
	{
		ROS_INFO_STREAM("/dev/ttyUSB0 is opened.");
	}
	else
	{
		return -1;
	}
	
	ros::Timer timer = n.createTimer(ros::Duration(0.0025), callback);

	ros::spin();

	sp.close();
 
	return 0;
}

void publish_imu_data(raw_t *data, sensor_msgs::Imu *imu_data)
{	
	imu_data->orientation.x = data->imu[data->nimu - 1].quat[1];
	imu_data->orientation.y = data->imu[data->nimu - 1].quat[2];
	imu_data->orientation.z = data->imu[data->nimu - 1].quat[3];
	imu_data->orientation.w = data->imu[data->nimu - 1].quat[0];
	imu_data->angular_velocity.x = data->imu[data->nimu - 1].gyr[0] * DEG_TO_RAD;
	imu_data->angular_velocity.y = data->imu[data->nimu - 1].gyr[1] * DEG_TO_RAD;
	imu_data->angular_velocity.z = data->imu[data->nimu - 1].gyr[2] * DEG_TO_RAD;
	imu_data->linear_acceleration.x = data->imu[data->nimu - 1].acc[0] * GRA_ACC;
	imu_data->linear_acceleration.y = data->imu[data->nimu - 1].acc[1] * GRA_ACC;
	imu_data->linear_acceleration.z = data->imu[data->nimu - 1].acc[2] * GRA_ACC;
}


