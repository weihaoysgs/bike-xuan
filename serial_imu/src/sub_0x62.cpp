//subscriber 0x62 data package

#include <ros/ros.h>
#include <iostream>
#include <unistd.h>
#include <serial_imu/Imu_0x62_msg.h>


void imu_0x62_callback(const serial_imu::Imu_0x62_msg imu_0x62_msg);

int main(int argc, char **argv)
{

	ros::init(argc, argv, "sub_0x62");
	
	ros::NodeHandle n;

	ros::Subscriber imu_0x62_sub = n.subscribe("/imu_0x62_package", 10,imu_0x62_callback);

	ros::spin();

}

void imu_0x62_callback(const serial_imu::Imu_0x62_msg imu_0x62_msg)
{
	int i = 0;


	printf("\033c");
	
	printf(" Device GWID: %6d\n", imu_0x62_msg.gw_id);
	printf(" Device number:%6d\n", imu_0x62_msg.node_num);
	if(imu_0x62_msg.node_num == 1)
	{
		i = 2;
		goto ONE_NODE;	
	}

	for (i = 0; i < imu_0x62_msg.node_num - (imu_0x62_msg.node_num % 2); i += 2)
	{
		putchar(10);
#if 1 

		printf("#     Devie ID:%6d					#    Device ID:%6d					#\r\n", imu_0x62_msg.node_data[i].id, imu_0x62_msg.node_data[i + 1].id);

		printf("#     Prs(hPa): %6f				#     Prs(hPa): %6f				#\r\n", imu_0x62_msg.node_data[i].prs, imu_0x62_msg.node_data[i + 1].prs);

		printf("#     Run time: %d days  %d:%d:%d:%d				#     Run time: %d days %d:%d:%d:%d				#\r\n",imu_0x62_msg.node_data[i].time / 86400000, imu_0x62_msg.node_data[i].time / 3600000 % 24, imu_0x62_msg.node_data[i].time / 60000 % 60, imu_0x62_msg.node_data[i].time / 1000 % 60, imu_0x62_msg.node_data[i].time % 1000,
				                                                                                          imu_0x62_msg.node_data[i + 1].time / 86400000, imu_0x62_msg.node_data[i + 1].time / 3600000 % 24, imu_0x62_msg.node_data[i + 1].time / 60000 % 60, imu_0x62_msg.node_data[i + 1].time / 1000 % 60, imu_0x62_msg.node_data[i + 1].time % 1000);

		printf("#   Frame Rate:  %4dHz					#   Frame Rate:  %4d					#\r\n", imu_0x62_msg.node_data[i].frame_rate, imu_0x62_msg.node_data[i + 1].frame_rate);

		printf("#       Acc(G):%8.3f %8.3f %8.3f		#       Acc(G):%8.3f %8.3f %8.3f		#\r\n", imu_0x62_msg.node_data[i].acc_x, imu_0x62_msg.node_data[i].acc_y, imu_0x62_msg.node_data[i].acc_z, imu_0x62_msg.node_data[i + 1].acc_x, imu_0x62_msg.node_data[i + 1].acc_y, imu_0x62_msg.node_data[i + 1].acc_z);

		printf("#   Gyr(deg/s):%8.2f %8.2f %8.2f		#   Gyr(deg/s):%8.2f %8.2f %8.2f		#\r\n", imu_0x62_msg.node_data[i].gyr_x, imu_0x62_msg.node_data[i].gyr_y, imu_0x62_msg.node_data[i].gyr_z, imu_0x62_msg.node_data[i + 1].gyr_x, imu_0x62_msg.node_data[i + 1].gyr_y, imu_0x62_msg.node_data[i + 1].gyr_z);

		printf("#      Mag(uT):%8.2f %8.2f %8.2f		#      Mag(uT):%8.2f %8.2f %8.2f		#\r\n", imu_0x62_msg.node_data[i].mag_x, imu_0x62_msg.node_data[i].mag_y, imu_0x62_msg.node_data[i].mag_z, imu_0x62_msg.node_data[i + 1].mag_x, imu_0x62_msg.node_data[i + 1].mag_y, imu_0x62_msg.node_data[i + 1].mag_z);

		printf("#   Eul(R P Y):%8.2f %8.2f %8.2f		#   Eul(R P Y):%8.2f %8.2f %8.2f		#\r\n", imu_0x62_msg.node_data[i].eul_r, imu_0x62_msg.node_data[i].eul_p, imu_0x62_msg.node_data[i].eul_y, imu_0x62_msg.node_data[i + 1].eul_r, imu_0x62_msg.node_data[i + 1].eul_p, imu_0x62_msg.node_data[i + 1].eul_y);

		printf("#Quat(W X Y Z):%8.3f %8.3f %8.3f %8.3f	#Quat(W X Y Z):%8.3f %8.3f %8.3f %8.3f	#\r\n", imu_0x62_msg.node_data[i].quat_w, imu_0x62_msg.node_data[i].quat_x, imu_0x62_msg.node_data[i].quat_y, imu_0x62_msg.node_data[i].quat_z, imu_0x62_msg.node_data[i + 1].quat_w, imu_0x62_msg.node_data[i + 1].quat_x, imu_0x62_msg.node_data[i + 1].quat_y, imu_0x62_msg.node_data[i + 1].quat_z);
#endif
	}

ONE_NODE:
	if(i - 1 == imu_0x62_msg.node_num || i + 1 == imu_0x62_msg.node_num)
	{
		putchar(10);

		if(imu_0x62_msg.node_num == 1)
			i -= 2;

		printf("#     Devie ID:%6d					#\n",imu_0x62_msg.node_data[i ].id);

		printf("#     Prs(hPa): %6f				#\n", imu_0x62_msg.node_data[i].prs);

		printf("#    Run times: %d days  %d:%d:%d:%d				#\n",imu_0x62_msg.node_data[i].time / 86400000, imu_0x62_msg.node_data[i].time / 3600000 % 24, imu_0x62_msg.node_data[i].time / 60000 % 60, imu_0x62_msg.node_data[i].time / 1000 % 60, imu_0x62_msg.node_data[i].time % 1000);

		printf("#   Frame Rate:  %4dHz					#\r\n", imu_0x62_msg.node_data[i].frame_rate);

		printf("#       Acc(G):%8.3f %8.3f %8.3f		#\r\n", imu_0x62_msg.node_data[i].acc_x, imu_0x62_msg.node_data[i].acc_y, imu_0x62_msg.node_data[i].acc_z);

		printf("#   Gyr(deg/s):%8.2f %8.2f %8.2f		#\r\n", imu_0x62_msg.node_data[i].gyr_x, imu_0x62_msg.node_data[i].gyr_y, imu_0x62_msg.node_data[i].gyr_z);

		printf("#      Mag(uT):%8.2f %8.2f %8.2f		#\r\n", imu_0x62_msg.node_data[i].mag_x, imu_0x62_msg.node_data[i].mag_y, imu_0x62_msg.node_data[i].mag_z);

		printf("#   Eul(R P Y):%8.2f %8.2f %8.2f		#\r\n", imu_0x62_msg.node_data[i].eul_r, imu_0x62_msg.node_data[i].eul_p, imu_0x62_msg.node_data[i].eul_y);

		printf("#Quat(W X Y Z):%8.3f %8.3f %8.3f %8.3f	#\r\n", imu_0x62_msg.node_data[i].quat_w, imu_0x62_msg.node_data[i].quat_x, imu_0x62_msg.node_data[i].quat_y, imu_0x62_msg.node_data[i].quat_z);
	}

}
