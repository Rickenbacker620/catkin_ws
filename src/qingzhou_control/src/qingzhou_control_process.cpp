#include "qingzhou_control.h"

void Actuator::processBattery()
{
	currentBattery.data = batteryVoltage;
	pub_battery.publish(currentBattery);
}

void Actuator::processOdom()
{
	if (encoderLeft > 220 || encoderLeft < -220)
		encoderLeft = 0;
	if (encoderRight > 220 || encoderRight < -220)
		encoderRight = 0;

	encoderRight = -encoderRight;

	detEncode = (encoderLeft + encoderRight) / 2; //求编码器平均值
	detDistance = detEncode / ticksPerMeter;
	detTh = (encoderRight - encoderLeft) * 2 * PI / ticksPer2PI; //计算当前角度 通过标定获得ticksPer2PI

	linearSpeed = detDistance / delta_time_;
	angularSpeed = detTh / delta_time_;

	if (detDistance != 0)
	{
		x += detDistance * cos(th); //x坐标
		y += detDistance * sin(th); //y坐标
	}
	if (detTh != 0)
	{
		th += detTh; //总角度
	}

	if (calibrate_lineSpeed == 1)
	{
		printf("x=%.2f,y=%.2f,th=%.2f,linearSpeed=%.2f,,detEncode=%.2f,LeftticksPerMeter = %lld,rightticksPerMeter = %lld,batteryVoltage = %.2f\n", x, y, th, linearSpeed, detEncode, LeftticksPerMeter, rightticksPerMeter, batteryVoltage);
	}

	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

	nav_msgs::Odometry odom; //创建nav_msgs::Odometry类型的消息odom
	odom.header.stamp = current_time_;
	odom.header.frame_id = "odom";
	odom.child_frame_id = "base_link";

	//set the position
	odom.pose.pose.position.x = x;
	odom.pose.pose.position.y = y;
	odom.pose.pose.position.z = 0.0;
	odom.pose.pose.orientation = odom_quat;

	odom.twist.twist.linear.x = linearSpeed; //线速度
	odom.twist.twist.linear.y = 0;
	odom.twist.twist.linear.z = 0;
	odom.twist.twist.angular.x = 0;
	odom.twist.twist.angular.y = 0;
	odom.twist.twist.angular.z = angularSpeed; //角速度

	if (encoderLeft == 0 && encoderRight == 0)
	{
		odom.pose.covariance = {1e-9, 0, 0, 0, 0, 0,
								0, 1e-3, 1e-9, 0, 0, 0,
								0, 0, 1e6, 0, 0, 0,
								0, 0, 0, 1e6, 0, 0,
								0, 0, 0, 0, 1e6, 0,
								0, 0, 0, 0, 0, 1e-9};
		odom.twist.covariance = {1e-9, 0, 0, 0, 0, 0,
								 0, 1e-3, 1e-9, 0, 0, 0,
								 0, 0, 1e6, 0, 0, 0,
								 0, 0, 0, 1e6, 0, 0,
								 0, 0, 0, 0, 1e6, 0,
								 0, 0, 0, 0, 0, 1e-9};
	}
	else
	{
		odom.pose.covariance = {1e-3, 0, 0, 0, 0, 0,
								0, 1e-3, 0, 0, 0, 0,
								0, 0, 1e6, 0, 0, 0,
								0, 0, 0, 1e6, 0, 0,
								0, 0, 0, 0, 1e6, 0,
								0, 0, 0, 0, 0, 1e3};
		odom.twist.covariance = {1e-3, 0, 0, 0, 0, 0,
								 0, 1e-3, 0, 0, 0, 0,
								 0, 0, 1e6, 0, 0, 0,
								 0, 0, 0, 1e6, 0, 0,
								 0, 0, 0, 0, 1e6, 0,
								 0, 0, 0, 0, 0, 1e3};
	}
	//publish the message
	pub_odom.publish(odom);
}

void actuator::process_imu()
{
	sensor_msgs::Imu imuMsg;

	imuMsg.header.stamp = current_time_;
	imuMsg.header.frame_id = "imu_link";
	imuMsg.angular_velocity.x = gyroX;
	imuMsg.angular_velocity.y = gyroY;
	imuMsg.angular_velocity.z = gyroZ;
	imuMsg.angular_velocity_covariance = {
		0.04, 0.0, 0.0,
		0.0, 0.04, 0.0,
		0.0, 0.0, 0.04};

	imuMsg.linear_acceleration.x = accelX;
	imuMsg.linear_acceleration.y = accelY;
	imuMsg.linear_acceleration.z = accelZ;
	imuMsg.linear_acceleration_covariance =
		{0.04, 0.0, 0.0,
		 0.0, 0.04, 0.0,
		 0.0, 0.0, 0.04};
	pub_imu.publish(imuMsg);

	sensor_msgs::MagneticField magMsg;
	magMsg.header.stamp = current_time_;
	magMsg.header.frame_id = "imu_link";
	magMsg.magnetic_field.x = magX;
	magMsg.magnetic_field.y = magY;
	magMsg.magnetic_field.z = magZ;
	magMsg.magnetic_field_covariance =
		{0.0, 0.0, 0.0,
		 0.0, 0.0, 0.0,
		 0.0, 0.0, 0.0};
	pub_mag.publish(magMsg);
}
