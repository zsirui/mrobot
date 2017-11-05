#include <string>
#include <ros/ros.h>											// 包含ROS的头文件
#include <ros/time.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <boost/asio.hpp>										//包含boost库函数
#include <boost/bind.hpp>
#include <math.h>
#include <geometry_msgs/Twist.h>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>

using namespace std;
using namespace boost::asio;									//定义一个命名空间，用于后面的读写操作
ros::Time current_time, last_time;
void writeSerial(const geometry_msgs::Twist& msg);

io_service iosev;												//节点文件
serial_port sp(iosev, "/dev/mrobot");
char* FILENAME;
void readConfigFile(char* filename);
void writeConfigFile(char* filename);


#define		R			105.00
#define		L			210.50
#define		fastAbs(n)	n > 0 ? n : -n
#define		PI			3.1415926535897932f

int _ID_FLAG = 0, _READ = 1;
unsigned char Get_Crc8(unsigned char *ptr, unsigned short len)
{
	unsigned char crc;
	unsigned char i;
	crc = 0;
	while(len--)
	{
		crc ^= *ptr++;
		for(i = 0; i < 8; i++)
		{
			if(crc&0x01)crc=(crc>>1)^0x8C;
			else crc >>= 1;
		}
	}
	return crc;
}

const string toHexString(const unsigned char* input, const int datasize)
{
	string output;
	char ch[3];

	for(int i = 0; i < datasize; ++i)
	{
		snprintf(ch, 3, "%02x", input[i]);
		output += ch;
	}
	return output;
}

bool compareID(const unsigned char* ID, string ID_FROM_FILE)
{
	string temp;
	temp = toHexString(ID, 12);
	if (temp == ID_FROM_FILE)
	{
		return true;
	} else
	{
		return false;
	}
}

double x = 0.0;
double y = 0.0;
double th = 0.0;

double vx = 0.0;
double vy = 0.0;
double vth = 0.0;

enum SendCommandType {
	SIMPLE_INFO = 23130,
	WITH_SONAR_INFO = 23125,
	WITH_SIX_AXIS_SENSOR_INFO = 23210,
	ALL_INFO = 23205,
	SPEED_INFO = 42330,
	BATTERY_INFO = 42325,
	SONAR_INFO = 42410,
	SIX_AXIS_SENSOR_INFO = 42405,
};

enum ReceiveCommandType {
	REQUIRE_WITH_SIMPLE_INFO = 0,
	REQUIRE_WITH_SONAR_INFO,
	REQUIRE_WITH_SIX_AXIS_SENSOR_INFO,
	REQUIRE_WITH_ALL_INFO,
	REQUIRE_WITH_SPEED_INFO,
	REQUIRE_WITH_BATTERY_INFO,
	REQUIRE_WITH_ONLY_SONAR_INFO,
	REQUIRE_WITH_ONLY_SIX_AXIS_SENSOR_INFO,
};

union sendData
{
	int d;
	unsigned char data[4];
}leftdata, rightdata;

union checkSum
{
	short d;
	unsigned char data[1];
}SendCheckSum, ReceiveCheckSum;

union receiveHeader
{
	int d;
	unsigned char data[2];
}receive_command, receive_header;

union receiveEnder
{
	int d;
	unsigned char data[2];
}receive_ender;

union sendCommand
{
	int d;
	unsigned char data[2];
}send_command;

union odometry													//里程计数据共用体
{
	float odoemtry_float;
	unsigned char odometry_char[4];
}vel_left, vel_right, temprature, odom_yaw, odom_pitch, odom_roll, sonar_front, sonar_back, sonar_left, sonar_right;

union battery													//电池数据共用体
{
	float battery_float;
	unsigned char battery_char[4];
}electricity, battery_percentage;

unsigned char buf[200];											//定义字符串长度
unsigned char _ID[12];											//存放芯片ID的临时变量
string read_str;
short command = -1;
const unsigned char ender[2] = {0x0d, 0x0a};
const unsigned char header[2] = {0x55, 0xaa};

void readSerial()
{
	int i, _length = 0;
	unsigned char _checkSum;
	read(sp, buffer(buf));
	ros::Time curr_time;
	for (i = 0; i < 2; i++)
	{
		receive_header.data[i] = buf[i];
	}
	if (receive_header.data[0] == header[0] && receive_header.data[1] == header[1])
	{
		for (i = 0; i < 2; i++)
		{
			receive_command.data[i] = buf[i + 2];
		}
		_length = buf[4];
		_checkSum = Get_Crc8(buf, 5 + _length);
		switch(receive_command.d)
		{
			case SIMPLE_INFO:
				command = REQUIRE_WITH_SIMPLE_INFO;
				if (buf[6 + _length] == ender[0] && buf[6 + _length + 1] == ender[1])
				{
					ReceiveCheckSum.data[0] = buf[5 + _length];
					if (_checkSum == ReceiveCheckSum.d)
					{
						for(i = 0; i < 4; i++)
						{
							vel_left.odometry_char[i] = buf[i + 5];
							vel_right.odometry_char[i] = buf[i + 9];
							electricity.battery_char[i] = buf[i + 13];
							battery_percentage.battery_char[i] = buf[i + 17];
						}
						vx = (vel_right.odoemtry_float + vel_left.odoemtry_float) / 2 / 1000;
						vth = (vel_right.odoemtry_float - vel_left.odoemtry_float) / L;
						
						curr_time = ros::Time::now();

						double dt = (curr_time - last_time).toSec();
						double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
						double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
						double delta_th = vth * dt;

						x += delta_x;
						y += delta_y;
						th += delta_th;
						last_time = curr_time;
					}
					else
					{
						ROS_INFO("Received data check sum error!");
					}
				}
			break;
			case WITH_SONAR_INFO:
				command = REQUIRE_WITH_SONAR_INFO;
				if (buf[6 + _length] == ender[0] && buf[6 + _length + 1] == ender[1])
				{
					ReceiveCheckSum.data[0] = buf[5 + _length];
					if (_checkSum == ReceiveCheckSum.d)
					{
						for(i = 0; i < 4; i++)
						{
							vel_left.odometry_char[i] = buf[i + 5];
							vel_right.odometry_char[i] = buf[i + 9];
							electricity.battery_char[i] = buf[i + 13];
							battery_percentage.battery_char[i] = buf[i + 17];
							sonar_front.odometry_char[i] = buf[i + 21];
							sonar_back.odometry_char[i] = buf[i + 25];
							sonar_left.odometry_char[i] = buf[i + 29];
							sonar_right.odometry_char[i] = buf[i + 33];
						}
						vx = (vel_right.odoemtry_float + vel_left.odoemtry_float) / 2 / 1000;
						vth = (vel_right.odoemtry_float - vel_left.odoemtry_float) / L;
						
						curr_time = ros::Time::now();

						double dt = (curr_time - last_time).toSec();
						double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
						double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
						double delta_th = vth * dt;

						x += delta_x;
						y += delta_y;
						th += delta_th;
						last_time = curr_time;
					}
					else
					{
						ROS_INFO("Received data check sum error!");
					}
				}
			break;
			case WITH_SIX_AXIS_SENSOR_INFO:
				command = REQUIRE_WITH_SIX_AXIS_SENSOR_INFO;
				if (buf[6 + _length] == ender[0] && buf[6 + _length + 1] == ender[1])
				{
					ReceiveCheckSum.data[0] = buf[5 + _length];
					if (_checkSum == ReceiveCheckSum.d)
					{
						for(i = 0; i < 4; i++)
						{
							vel_left.odometry_char[i] = buf[i + 5];
							vel_right.odometry_char[i] = buf[i + 9];
							electricity.battery_char[i] = buf[i + 13];
							battery_percentage.battery_char[i] = buf[i + 17];
							odom_pitch.odometry_char[i] = buf[i + 21];
							odom_roll.odometry_char[i] = buf[i + 25];
							odom_yaw.odometry_char[i] = buf[i + 29];
							temprature.odometry_char[i] = buf[i + 33];
						}
						vx = (vel_right.odoemtry_float + vel_left.odoemtry_float) / 2 / 1000;
						vth = (vel_right.odoemtry_float - vel_left.odoemtry_float) / L;
						
						curr_time = ros::Time::now();

						double dt = (curr_time - last_time).toSec();
						double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
						double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
						double delta_th = vth * dt;

						x += delta_x;
						y += delta_y;
						th += delta_th;
						last_time = curr_time;
					}
					else
					{
						ROS_INFO("Received data check sum error!");
					}
				}
			break;
			case ALL_INFO:
				command = REQUIRE_WITH_ALL_INFO;
				if (buf[6 + _length] == ender[0] && buf[6 + _length + 1] == ender[1])
				{
					ReceiveCheckSum.data[0] = buf[5 + _length];
					if (_checkSum == ReceiveCheckSum.d)
					{
						for(i = 0; i < 4; i++)
						{
							vel_left.odometry_char[i] = buf[i + 5];
							vel_right.odometry_char[i] = buf[i + 9];
							electricity.battery_char[i] = buf[i + 13];
							battery_percentage.battery_char[i] = buf[i + 17];
							odom_pitch.odometry_char[i] = buf[i + 21];
							odom_roll.odometry_char[i] = buf[i + 25];
							odom_yaw.odometry_char[i] = buf[i + 29];
							temprature.odometry_char[i] = buf[i + 33];
							sonar_front.odometry_char[i] = buf[i + 37];
							sonar_back.odometry_char[i] = buf[i + 41];
							sonar_left.odometry_char[i] = buf[i + 45];
							sonar_right.odometry_char[i] = buf[i + 49];
						}
						vx = (vel_right.odoemtry_float + vel_left.odoemtry_float) / 2 / 1000;
						vth = (vel_right.odoemtry_float - vel_left.odoemtry_float) / L;
						
						curr_time = ros::Time::now();

						double dt = (curr_time - last_time).toSec();
						double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
						double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
						double delta_th = vth * dt;

						x += delta_x;
						y += delta_y;
						th += delta_th;
						last_time = curr_time;
					}
					else
					{
						ROS_INFO("Received data check sum error!");
					}
				}
			break;
			case SPEED_INFO:
				command = REQUIRE_WITH_SPEED_INFO;
				if (buf[6 + _length] == ender[0] && buf[6 + _length + 1] == ender[1])
				{
					ReceiveCheckSum.data[0] = buf[5 + _length];
					if (_checkSum == ReceiveCheckSum.d)
					{
						for(i = 0; i < 4; i++)
						{
							vel_left.odometry_char[i] = buf[i + 5];
							vel_right.odometry_char[i] = buf[i + 9];
						}
						vx = (vel_right.odoemtry_float + vel_left.odoemtry_float) / 2 / 1000;
						vth = (vel_right.odoemtry_float - vel_left.odoemtry_float) / L;
						
						curr_time = ros::Time::now();

						double dt = (curr_time - last_time).toSec();
						double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
						double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
						double delta_th = vth * dt;

						x += delta_x;
						y += delta_y;
						th += delta_th;
						last_time = curr_time;
					}
					else
					{
						ROS_INFO("Received data check sum error!");
					}
				}
			break;
			case BATTERY_INFO:
				command = REQUIRE_WITH_BATTERY_INFO;
				if (buf[6 + _length] == ender[0] && buf[6 + _length + 1] == ender[1])
				{
					ReceiveCheckSum.data[0] = buf[5 + _length];
					if (_checkSum == ReceiveCheckSum.d)
					{
						for(i = 0; i < 4; i++)
						{
							electricity.battery_char[i] = buf[i + 5];
							battery_percentage.battery_char[i] = buf[i + 9];
						}
						vx = (vel_right.odoemtry_float + vel_left.odoemtry_float) / 2 / 1000;
						vth = (vel_right.odoemtry_float - vel_left.odoemtry_float) / L;
						
						curr_time = ros::Time::now();

						double dt = (curr_time - last_time).toSec();
						double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
						double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
						double delta_th = vth * dt;

						x += delta_x;
						y += delta_y;
						th += delta_th;
						last_time = curr_time;
					}
					else
					{
						ROS_INFO("Received data check sum error!");
					}
				}
			break;
			case SONAR_INFO:
				command = REQUIRE_WITH_ONLY_SONAR_INFO;
				if (buf[6 + _length] == ender[0] && buf[6 + _length + 1] == ender[1])
				{
					ReceiveCheckSum.data[0] = buf[5 + _length];
					if (_checkSum == ReceiveCheckSum.d)
					{
						for(i = 0; i < 4; i++)
						{
							sonar_front.odometry_char[i] = buf[i + 5];
							sonar_back.odometry_char[i] = buf[i + 9];
							sonar_left.odometry_char[i] = buf[i + 13];
							sonar_right.odometry_char[i] = buf[i + 17];
						}
						vx = (vel_right.odoemtry_float + vel_left.odoemtry_float) / 2 / 1000;
						vth = (vel_right.odoemtry_float - vel_left.odoemtry_float) / L;
						
						curr_time = ros::Time::now();

						double dt = (curr_time - last_time).toSec();
						double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
						double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
						double delta_th = vth * dt;

						x += delta_x;
						y += delta_y;
						th += delta_th;
						last_time = curr_time;
					}
					else
					{
						ROS_INFO("Received data check sum error!");
					}
				}
			break;
			case SIX_AXIS_SENSOR_INFO:
				command = REQUIRE_WITH_ONLY_SIX_AXIS_SENSOR_INFO;
				if (buf[6 + _length] == ender[0] && buf[6 + _length + 1] == ender[1])
				{
					ReceiveCheckSum.data[0] = buf[5 + _length];
					if (_checkSum == ReceiveCheckSum.d)
					{
						for(i = 0; i < 4; i++)
						{
							odom_pitch.odometry_char[i] = buf[i + 5];
							odom_roll.odometry_char[i] = buf[i + 9];
							odom_yaw.odometry_char[i] = buf[i + 13];
							temprature.odometry_char[i] = buf[i + 17];
						}
						vx = (vel_right.odoemtry_float + vel_left.odoemtry_float) / 2 / 1000;
						vth = (vel_right.odoemtry_float - vel_left.odoemtry_float) / L;
						
						curr_time = ros::Time::now();

						double dt = (curr_time - last_time).toSec();
						double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
						double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
						double delta_th = vth * dt;

						x += delta_x;
						y += delta_y;
						th += delta_th;
						last_time = curr_time;
					}
					else
					{
						ROS_INFO("Received data check sum error!");
					}
				}
			break;
		}
		for(i = 0; i < 12; i++)
		{
			_ID[i] = buf[8 + _length + i];
		}
		readConfigFile(FILENAME);
	}
}

void writeSerial(const geometry_msgs::Twist& msg)
{
	double RobotV = msg.linear.x * 1000;
	double YawRate = msg.angular.z;
	unsigned char buf_[16] = {0};
	int i, length = 0;
	double r = RobotV / YawRate;
	if(RobotV == 0)
	{
		leftdata.d = -YawRate * R;
		rightdata.d = YawRate * R;
	} else if(YawRate == 0)
	{
		leftdata.d = RobotV;
		rightdata.d = RobotV;
	}
	else
	{
		leftdata.d = YawRate * (r - R);
		rightdata.d = YawRate * (r + R);
	}
	for(i = 0; i < 2; i++)
	{
		buf_[i] = header[i];
	}
	send_command.d = 21850;
	for(i = 0; i < 2; i++)
	{
		buf_[i + 2] = send_command.data[i];
	}
	length = 8;
	buf_[4] = length;
	for(i = 0; i < 4; i++)
	{
		buf_[i + 5] = rightdata.data[i];
		buf_[i + 9] = leftdata.data[i];
	}
	buf_[5 + length] = Get_Crc8(buf_, 5 + length);
	buf_[6 + length] = ender[0];
	buf_[6 + length + 1] = ender[1];
	write(sp, buffer(buf_));
	readSerial();
}

void readConfigFile(char* filename)
{
	if (_READ)
	{
		ifstream file;
		file.open(filename, ios::in | ios::binary);
		if (file)
		{
			file >> read_str;
			if (compareID(_ID, read_str))
			{
				ROS_INFO("ID Compare Successful");
			}else
			{
				ROS_ERROR("ID Compare Failed");
				exit(-1);
			}
		} else
		{
			writeConfigFile(filename);
		}
		file.close();
		_READ = 0;
	}
}

void writeConfigFile(char* filename)
{
	ofstream file;
	file.open(filename, ios::out | ios::app | ios::binary);
	if (file)
	{
		file << toHexString(_ID, 12);
	} else
	{
		ROS_INFO("Error opening Config file %s!", filename);
	}
	file.close();
	_READ = 1;
}

int main(int argc, char** argv)
{
	FILENAME = strcat(getenv("HOME"), "/mrobot.config");
	try
	{
		// 设置参数
		sp.set_option(serial_port::baud_rate(115200));
		sp.set_option(serial_port::flow_control(serial_port::flow_control::none));
		sp.set_option(serial_port::parity(serial_port::parity::none));
		sp.set_option(serial_port::stop_bits(serial_port::stop_bits::one));
		sp.set_option(serial_port::character_size(8));
		ROS_INFO("Serial Port initialized successful.");
		ros::init(argc, argv, "serial_node");									//初始化节点
		ros::Time::init();
		current_time = ros::Time::now();
		last_time = ros::Time::now();
		ros::Rate loop_rate(50);
		ros::NodeHandle nh;
		ros::Publisher pub = nh.advertise<nav_msgs::Odometry>("odom", 50);		//定义发布消息的名称
		ros::Subscriber sub = nh.subscribe("cmd_vel", 50, &writeSerial);
		tf::TransformBroadcaster odom_broadcaster;
		ROS_INFO("ROS Node initialized successful.");
		
		while (ros::ok()) {
			ros::spinOnce();

			current_time = ros::Time::now();
			geometry_msgs::TransformStamped odom_trans;

			odom_trans.header.stamp = current_time;
			odom_trans.header.frame_id = "odom";
			odom_trans.child_frame_id = "base_footprint";

			geometry_msgs::Quaternion odom_quat;
			if (command == REQUIRE_WITH_ALL_INFO)
			{
				odom_quat = tf::createQuaternionMsgFromYaw(th);
				odom_trans.transform.translation.x = x;
				odom_trans.transform.translation.y = y;
				odom_trans.transform.translation.z = 0.0;
				odom_trans.transform.rotation = odom_quat;
				ROS_INFO("Left Speed: %.4f m/s.", vel_left.odoemtry_float / 1000);
				ROS_INFO("Right Speed: %.4f m/s.", vel_right.odoemtry_float / 1000);
				ROS_INFO("Battery Voltage: %.3f mV.", electricity.battery_float);
				ROS_INFO("Battery Electricity Percentage: %.3f%%.", battery_percentage.battery_float);
				ROS_INFO("Temprature: %.2f C.", temprature.odoemtry_float);
				ROS_INFO("Pitch: %.3f.", odom_pitch.odoemtry_float);
				ROS_INFO("Roll: %.3f.", odom_roll.odoemtry_float);
				ROS_INFO("Yaw: %.3f.", odom_yaw.odoemtry_float);
				ROS_INFO("Sonar front: %.3f m.", sonar_front.odoemtry_float);
				ROS_INFO("Sonar back: %.3f m.", sonar_back.odoemtry_float);
				ROS_INFO("Sonar left: %.3f m.", sonar_left.odoemtry_float);
				ROS_INFO("Sonar right: %.3f m.", sonar_right.odoemtry_float);
			} else if (command == REQUIRE_WITH_SIMPLE_INFO)
			{
				odom_quat = tf::createQuaternionMsgFromYaw(th);
				odom_trans.transform.translation.x = x;
				odom_trans.transform.translation.y = y;
				odom_trans.transform.translation.z = 0.0;
				odom_trans.transform.rotation = odom_quat;
				ROS_INFO("Left Speed: %.4f m/s.", vel_left.odoemtry_float / 1000);
				ROS_INFO("Right Speed: %.4f m/s.", vel_right.odoemtry_float / 1000);
				ROS_INFO("Battery Voltage: %.3f mV.", electricity.battery_float);
				ROS_INFO("Battery Electricity Percentage: %.3f%%.", battery_percentage.battery_float);
			} else if(command == REQUIRE_WITH_SIX_AXIS_SENSOR_INFO)
			{
				odom_quat = tf::createQuaternionMsgFromYaw(th);
				odom_trans.transform.translation.x = x;
				odom_trans.transform.translation.y = y;
				odom_trans.transform.translation.z = 0.0;
				odom_trans.transform.rotation = odom_quat;
				ROS_INFO("Left Speed: %.4f m/s.", vel_left.odoemtry_float / 1000);
				ROS_INFO("Right Speed: %.4f m/s.", vel_right.odoemtry_float / 1000);
				ROS_INFO("Battery Voltage: %.3f mV.", electricity.battery_float);
				ROS_INFO("Battery Electricity Percentage: %.3f%%.", battery_percentage.battery_float);
				ROS_INFO("Temprature: %.2f C.", temprature.odoemtry_float);
				ROS_INFO("Pitch: %.3f.", odom_pitch.odoemtry_float);
				ROS_INFO("Roll: %.3f.", odom_roll.odoemtry_float);
				ROS_INFO("Yaw: %.3f.", odom_yaw.odoemtry_float);
			} else
			{
				odom_quat = tf::createQuaternionMsgFromYaw(th);
				odom_trans.transform.translation.x = x;
				odom_trans.transform.translation.y = y;
				odom_trans.transform.translation.z = 0.0;
				odom_trans.transform.rotation = odom_quat;
			}

			//send the transform
			odom_broadcaster.sendTransform(odom_trans);

			nav_msgs::Odometry msgl;
			msgl.header.stamp = current_time;
			msgl.header.frame_id = "odom";

			msgl.pose.pose.position.x = x;
			msgl.pose.pose.position.y = y;
			msgl.pose.pose.position.z = 0.0;
			msgl.pose.pose.orientation = odom_quat;

			msgl.child_frame_id = "base_footprint";
			msgl.twist.twist.linear.x = vx;
			msgl.twist.twist.linear.y = vy;
			msgl.twist.twist.angular.z = vth;
			
			pub.publish(msgl);   											//发布消息
			command = -1;
			loop_rate.sleep();
		}
		iosev.run();
	}
	catch (exception& err)  
	{
		ROS_ERROR("Exception Error: %s", err.what());
		return -1; 
	}
	return 0;
}