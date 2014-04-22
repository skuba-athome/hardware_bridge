#ifndef MY_DWA_H_
#define MY_DWA_H_

#include <ros/ros.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf/transform_listener.h>
#include <navfn/navfn_ros.h>
#include <eband_local_planner/eband_local_planner.h>
#include <eband_local_planner/eband_visualization.h>

#define VEL_MAX										0.2f
#define VEL_MIN										0.00f
#define ACCEL_MAX									0.3f
#define DEACCEL_MAX									0.1f
#define P_SAMPLE_TIMES								3
#define PHI_SAMPLE_TIMES 							10
#define OBSERVE_DIST								1.0f
#define MAXIMUM_COLLISION_FREE_COSTMAP_VALUE		250
#define MAXIMUM_COSTMAP_VALUE						255
#define GOAL_REGION_DIST							0.20f
#define GOAL_FINISH_DIST							0.10f
#define OBSTABLE_COST_GAIN							0.0f
#define VELOCITY_COST_GAIN							1.0f
#define ALIGNMENT_COST_GAIN							8.0f

#ifndef	PI
	#define PI	3.14159265359f
#endif

costmap_2d::Costmap2DROS *my_costmap_ros;
costmap_2d::Costmap2D my_costmap;
navfn::NavfnROS *my_navfn;
eband_local_planner::EBandPlanner *my_eband;
boost::shared_ptr<eband_local_planner::EBandVisualization> eband_visual;
ros::Publisher pub_vel;
geometry_msgs::PoseStamped robot_goal;
double my_goal[3];

bool isVelAdmissible(float* pose,float vel_dir,float breaking_dist,float observe_dist, float* obstacle_cost);
float modified_goalheading(float* pose,float goal,float dist);

#endif
