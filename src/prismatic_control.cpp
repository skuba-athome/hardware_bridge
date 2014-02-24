#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <robot_connect/TiltLaserCmd.h>

#define BASEOFF	0.215
#define OFF	0.05
#define KINECTOFF	0.345
ros::Publisher pub_laser_height,pub_kinect_height, pub_tilt_cmd;
bool tilt_ang_rdy = false;
void tilt_laser_cmd(std::string cmdtype)
{
	robot_connect::TiltLaserCmd cmd;
	cmd.instruction = cmdtype;
	cmd.angle = M_PI/2;
	pub_tilt_cmd.publish(cmd);
}
void scanCallback(const sensor_msgs::LaserScanConstPtr &scan){
	static bool init = false;
	if(!init)
	{
		tilt_laser_cmd("REQUEST");
		init = true;
	}
	if(!tilt_ang_rdy) return;

	unsigned int center_index = scan->ranges.size()/2;
	float sum = 0.0;
	unsigned int k = 0;
	for(unsigned int n = center_index-10; n <= center_index+10;n++)
	{
		sum += scan->ranges[n]*cos((n-center_index)*scan->angle_increment);
		k++;
	}
	float height = sum/k;
	std_msgs::Float64 height_f64;
	height_f64.data = height;//laser height from base
	height_f64.data += OFF;
	height_f64.data += BASEOFF;//laser height from ground
	pub_laser_height.publish(height_f64);
	height_f64.data += KINECTOFF;//kinect height from ground
	pub_kinect_height.publish(height_f64);
	//ROS_INFO("%d %f %d",scan->ranges.size(),height,k);

	tilt_laser_cmd("RELEASE");
}

void tilt_laser_Callback(const std_msgs::StringConstPtr& msg)
{
	if(msg->data == "SUCCEEDED") tilt_ang_rdy = true;
	else tilt_ang_rdy = false;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "prismatic_control");

  ros::NodeHandle n;
  ros::Subscriber scan_sub = n.subscribe("/tilt_scan", 1, scanCallback);
  ros::Subscriber tilt_sub = n.subscribe("/tilt_laser_status", 1, tilt_laser_Callback);
  pub_tilt_cmd = n.advertise<robot_connect::TiltLaserCmd> ("/tilt_laser_cmd", 1);
  pub_laser_height = n.advertise<std_msgs::Float64> ("/laser_height", 1);
  pub_kinect_height = n.advertise<std_msgs::Float64> ("/kinect_height", 1);
  ros::spin();
  
  return 0;
}
