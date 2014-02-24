/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include <cstdio>
#include <ros/ros.h>

// Services
#include <laser_assembler/AssembleScans.h>

// Messages
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Quaternion.h>
#include <dynamixel_msgs/JointState.h>
#include <std_msgs/Float64.h>

#define MAX_ANG	1.0f
#define MIN_ANG 0.0f

ros::Publisher pub_assemble_cloud;
ros::Publisher pub_tilt_cmd;
ros::ServiceClient client;
dynamixel_msgs::JointState tilt_fdb;

void assemblerRequest()
{
	static ros::Time last_time;
	ros::Time pres_time;
	static bool init = false;
	pres_time = ros::Time::now();
	if(!init)
	{
		last_time = pres_time;
		init = true;
		return;
	}
  // Populate our service request based on our timer callback times
  laser_assembler::AssembleScans srv;
  srv.request.begin = last_time;
  srv.request.end   = pres_time;
  last_time = pres_time;
  // Make the service call
  if (client.call(srv))
  {
    ROS_INFO("Published Cloud with %u points", (uint32_t)(srv.response.cloud.points.size())) ;
    pub_assemble_cloud.publish(srv.response.cloud);
  }
  else
  {
    ROS_ERROR("Error making service call\n") ;
  }
}

void servo_pattern1()
{
  float ang_cmd;  
  float cur_ang = tilt_fdb.current_pos;
  float cen_ang = (MAX_ANG-MIN_ANG)/2.0f;
  bool is_mov = tilt_fdb.is_moving;
  if(!is_mov)
  {
  	assemblerRequest();
  	if(cur_ang>cen_ang)
  		ang_cmd = MIN_ANG;	
  	else
  		ang_cmd = MAX_ANG;
  	std_msgs::Float64 ang_cmd64;
  	ang_cmd64.data = ang_cmd;
  	pub_tilt_cmd.publish(ang_cmd64);
  }
}

void tilt_callback(const dynamixel_msgs::JointStateConstPtr &tilt_in)
{
	static tf::TransformBroadcaster broadcaster;
	tilt_fdb = *tilt_in;
	servo_pattern1();
	broadcaster.sendTransform(
	tf::StampedTransform(
	tf::Transform(tf::createQuaternionFromRPY(M_PI, tilt_in->current_pos, 0.0), tf::Vector3(0.0, 0.0, 0.0)),
	ros::Time::now(),"base_link", "laser"));
}

/*void timerCallback(const ros::TimerEvent& e)
{
  // We don't want to build a cloud the first callback, since we we
  //   don't have a start and end time yet
  static bool first_time = true;
  if (first_time)
  {
    first_time = false;
    return;
  }
   // Populate our service request based on our timer callback times
  laser_assembler::AssembleScans srv;
  srv.request.begin = e.last_real;
  srv.request.end   = e.current_real;
  // Make the service call
  if (client.call(srv))
  {
    ROS_INFO("Published Cloud with %u points", (uint32_t)(srv.response.cloud.points.size())) ;
    pub_assemble_cloud.publish(srv.response.cloud);
  }
  else
  {
    ROS_ERROR("Error making service call\n") ;
  }
}*/

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tilt_laser_assembler");
  ros::NodeHandle n;
  
  //ROS_INFO("Waiting for [build_cloud] to be advertised");
  ros::service::waitForService("assemble_scans");
  //ROS_INFO("Found build_cloud! Starting the snapshotter");
  pub_assemble_cloud = n.advertise<sensor_msgs::PointCloud> ("assembled_cloud", 1);
  pub_tilt_cmd = n.advertise<std_msgs::Float64> ("/tilt_laser/command", 1);
  
  ros::Subscriber sub_tilt = n.subscribe("/tilt_laser/state", 1, tilt_callback);
 
  // Create the service client for calling the assembler
  client = n.serviceClient<laser_assembler::AssembleScans>("assemble_scans");
 	//ros::Timer timer = n.createTimer(ros::Duration(0.5), &timerCallback); 
  ros::spin();
  return 0;
}
