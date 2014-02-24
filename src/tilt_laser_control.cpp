#include <cstdio>
#include <ros/ros.h>

// Messages
#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Quaternion.h>
#include <dynamixel_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <dynamixel_controllers/SetSpeed.h>
#include <robot_connect/TiltLaserCmd.h>

#define MAX_ANG	0.7f
#define MIN_ANG -0.5f
#define FAST_SPEED	1.0f
#define SLOW_SPEED	0.5f

ros::Publisher pub_tilt_cmd, pub_status;
ros::ServiceClient client;
dynamixel_msgs::JointState tilt_fdb;
robot_connect::TiltLaserCmd new_servo_cmd;
float laser_height;bool get_valid_height = false;


void cmd_servo(float ang, float speed)
{
	dynamixel_controllers::SetSpeed srv;
	srv.request.speed = speed;
	client.call(srv);
	std_msgs::Float64 ang_cmd64;
	ang_cmd64.data = ang;
	pub_tilt_cmd.publish(ang_cmd64);
}

void servo_routine()
{
  float ang_cmd;  
  float cur_ang = tilt_fdb.current_pos;
  float cen_ang = (MAX_ANG-MIN_ANG)/2.0f;
  bool is_mov = tilt_fdb.is_moving;
  float speed_cmd;
  if(!is_mov)
  {
  	dynamixel_controllers::SetSpeed srv;
  	if(cur_ang>cen_ang)
  	{
  		speed_cmd = SLOW_SPEED;
  		ang_cmd = MIN_ANG;
  	}	
  	else
  	{
  		speed_cmd = FAST_SPEED;
  		ang_cmd = MAX_ANG;
  	}
	cmd_servo(ang_cmd, speed_cmd);
  }
}

void servo_zero()
{
	cmd_servo(0.0f, FAST_SPEED);
}

void tilt_callback(const dynamixel_msgs::JointStateConstPtr &tilt_in)
{
	static tf::TransformBroadcaster broadcaster;
	tilt_fdb = *tilt_in;
	
	//ros::Duration timeoffset(10.0/1000.0);
	if(get_valid_height) broadcaster.sendTransform(
	tf::StampedTransform(
	tf::Transform(tf::createQuaternionFromRPY(M_PI, tilt_in->current_pos, 0.105), tf::Vector3(0.125, 0.0, laser_height)),
	ros::Time::now()+ros::Duration(0.033),"base_link", "tilt_laser_link"));
	//ROS_INFO("%f",laser_height);

	std::string instr = new_servo_cmd.instruction;
	std_msgs::String msg;
	msg.data = "FREE";
	if(instr == "REQUEST")
	{
		cmd_servo(new_servo_cmd.angle, FAST_SPEED);
		float abs_deg_error = fabs(tilt_fdb.current_pos - new_servo_cmd.angle)*180.0/M_PI;
		if(abs_deg_error<4.0f) msg.data = "SUCCEEDED";
		else msg.data = "ACTIVE";	
		//ROS_INFO("%f",abs_deg_error);
	}
	else if(instr == "RELEASE")
	{
		servo_routine();
		//servo_zero();	
	}
	pub_status.publish(msg);
	
}

void tilt_laser_cmd_callback(const robot_connect::TiltLaserCmdConstPtr& msg)
{
	new_servo_cmd = *msg;
}

void laser_height_callback(const std_msgs::Float64ConstPtr& msg)
{
	laser_height = msg->data;
	get_valid_height = true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tilt_laser_control");
  ros::NodeHandle n;
  
  ros::service::waitForService("/tilt_laser/set_speed");
  
  pub_tilt_cmd = n.advertise<std_msgs::Float64> ("/tilt_laser/command", 1);
  pub_status = n.advertise<std_msgs::String> ("/tilt_laser_status", 1);
  
  ros::Subscriber sub_tilt = n.subscribe("/tilt_laser/state", 1, tilt_callback);
  ros::Subscriber sub_tilt_cmd = n.subscribe("/tilt_laser_cmd", 1, tilt_laser_cmd_callback);
  ros::Subscriber sub_hei = n.subscribe("/laser_height", 1, laser_height_callback);
  
  client = n.serviceClient<dynamixel_controllers::SetSpeed>("/tilt_laser/set_speed");
  
  ros::spin();
  return 0;
}
