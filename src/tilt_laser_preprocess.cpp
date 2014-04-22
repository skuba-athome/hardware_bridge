#include <cstdio>
#include <ros/ros.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include "laser_geometry/laser_geometry.h"
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <pcl/ros/conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/passthrough.h>

// Messages
#include <dynamixel_msgs/JointState.h>
#include <dynamixel_controllers/SetSpeed.h>
#include <robot_connect/TiltLaserCmd.h>

using namespace std;


#define MAX_ANG	0.7f
#define MIN_ANG -0.5f
#define FAST_SPEED	2.0f
#define SLOW_SPEED	0.5f

string robot_frame = "base_link";
ros::Publisher cloud_pub, pub_tilt_cmd, pub_status;
tf::TransformListener* listener;

ros::ServiceClient client;
dynamixel_msgs::JointState tilt_fdb;
robot_connect::TiltLaserCmd new_servo_cmd;
float laser_height;bool get_valid_height = false;
bool halt_laser = false;

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
  		halt_laser = false;
  	}	
  	else
  	{
  		speed_cmd = FAST_SPEED;
  		ang_cmd = MAX_ANG;
  		halt_laser = true;
  	}
	cmd_servo(ang_cmd, speed_cmd);
  }
}

void servo_zero()
{
	halt_laser = false;
	cmd_servo(0.0f, FAST_SPEED);
}

void tilt_callback(const dynamixel_msgs::JointStateConstPtr &tilt_in)
{
	static tf::TransformBroadcaster broadcaster;
	tilt_fdb = *tilt_in;
	
	if(get_valid_height) broadcaster.sendTransform(
	tf::StampedTransform(
	tf::Transform(tf::createQuaternionFromRPY(M_PI, tilt_in->current_pos, 0.105), tf::Vector3(0.125, 0.0, laser_height)),
	ros::Time::now()+ros::Duration(0.1), robot_frame, "tilt_laser_link"));

	std::string instr = new_servo_cmd.instruction;
	std_msgs::String msg;
	msg.data = "FREE";
	if(instr == "REQUEST")
	{
		halt_laser = false;
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

void scanCallback(const sensor_msgs::LaserScanConstPtr &scan_in)
{
  	try{
		sensor_msgs::PointCloud2 scan2cloud;
	  	laser_geometry::LaserProjection projector;
	  	listener->waitForTransform(robot_frame, scan_in->header.frame_id, scan_in->header.stamp, ros::Duration(1.0));
  		projector.transformLaserScanToPointCloud(robot_frame,*scan_in,scan2cloud,*listener);
  		
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  		pcl::fromROSMsg(scan2cloud,*cloud);
  		
  		// Create the filtering object
		pcl::PassThrough<pcl::PointXYZ> pass;
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_passth (new pcl::PointCloud<pcl::PointXYZ>);
		
		if(halt_laser == false)
		{
			pass.setInputCloud (cloud);
			pass.setFilterFieldName ("x");
			pass.setFilterLimits (0.4, 5.00);
			pass.setFilterLimitsNegative (false);
			pass.filter (*cloud_passth);
		}

		cloud_passth->width = cloud_passth->points.size ();
		cloud_passth->height = 1;
		cloud_passth->is_dense = true;

		cloud_passth->header = scan2cloud.header;
		sensor_msgs::PointCloud2 cloud_out;
		pcl::toROSMsg(*cloud_passth,cloud_out);
		
		cloud_pub.publish(cloud_out);
  	}
  	catch(tf::TransformException& ex){
  		ROS_ERROR("Received an exception trying to transform a point from %s to %s: %s", scan_in->header.frame_id.c_str(),robot_frame.c_str(),ex.what());
  	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "tilt_laser_preprocess");
	ros::NodeHandle n;
	
	ros::service::waitForService("/tilt_laser/set_speed");
	
	ros::Subscriber cloud_sub = n.subscribe("/tilt_scan", 1, scanCallback);
	
  	ros::Subscriber sub_tilt = n.subscribe("/tilt_laser/state", 1, tilt_callback);
	ros::Subscriber sub_tilt_cmd = n.subscribe("/tilt_laser_cmd", 1, tilt_laser_cmd_callback);
  	ros::Subscriber sub_hei = n.subscribe("/laser_height", 1, laser_height_callback);
	
	cloud_pub = n.advertise<sensor_msgs::PointCloud2>("tilt_scan_cloud",1);
	pub_tilt_cmd = n.advertise<std_msgs::Float64> ("/tilt_laser/command", 1);
    pub_status = n.advertise<std_msgs::String> ("/tilt_laser_status", 1);
    
    client = n.serviceClient<dynamixel_controllers::SetSpeed>("/tilt_laser/set_speed");
    
	listener = new tf::TransformListener();
	
	ros::spin();  
	return 0;
}
