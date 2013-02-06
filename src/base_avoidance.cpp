#include <tf/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <nav_core/base_global_planner.h>
#include <nav_core/base_local_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <pluginlib/class_loader.h>

static geometry_msgs::PoseStamped::Ptr goal(new geometry_msgs::PoseStamped);
ros::Publisher pub_vel;
costmap_2d::Costmap2DROS* planner_costmap_ros;
nav_core::BaseGlobalPlanner* planner;
nav_core::BaseLocalPlanner* tc;

static bool pass_goalcb = false;
#define GOAL_RADIUS	0.5f

void goalCallback(const geometry_msgs::PoseStamped::Ptr& new_goal)
{
	pass_goalcb = true;
	goal = new_goal;
	//robot_goal.header.frame_id = planner_costmap_ros->getGlobalFrameID();
	//planner->makePlan(robot_pose,*new_goal,path);
	//ROS_INFO("%f %f %f %c",new_goal->pose.position.x,new_goal->pose.position.y,new_goal->pose.position.z,new_goal->header.frame_id[1]);
	//ROS_INFO("%f,%f",path[0].pose.position.x,path[1].pose.position.y);
} 
void goalrvizCallback(const geometry_msgs::PoseStamped::Ptr& new_goal)
{
	pass_goalcb = true;
	goal = new_goal;
	//robot_goal.header.frame_id = planner_costmap_ros->getGlobalFrameID();
	//planner->makePlan(robot_pose,*new_goal,path);
	//ROS_INFO("%f %f %f %c",new_goal->pose.position.x,new_goal->pose.position.y,new_goal->pose.position.z,new_goal->header.frame_id[1]);
	//ROS_INFO("%f,%f",path[0].pose.position.x,path[1].pose.position.y);
}



void trajectory_con(const tf::TransformListener& listener)
{
	if(!pass_goalcb) return;
	
	geometry_msgs::PoseStamped goal_tmp;
	std::string frame_id = "base_link";
	if(1)//goal->header.frame_id == frame_id)
	{
		listener.waitForTransform("odom", "base_link", goal->header.stamp, ros::Duration(10.0));
		listener.transformPose("odom",*goal,goal_tmp);

		ROS_INFO("pass tf");
	}
	
	std::vector<geometry_msgs::Point> clear_poly;
	double x = goal_tmp.pose.position.x;
	double y = goal_tmp.pose.position.y;
	geometry_msgs::Point pt;
	
	pt.x = x - GOAL_RADIUS;
	pt.y = y - GOAL_RADIUS;
	clear_poly.push_back(pt);
	
	pt.x = x + GOAL_RADIUS;
	pt.y = y - GOAL_RADIUS;
	clear_poly.push_back(pt);
	
	pt.x = x + GOAL_RADIUS;
	pt.y = y + GOAL_RADIUS;
	clear_poly.push_back(pt);
	
	pt.x = x - GOAL_RADIUS;
	pt.y = y + GOAL_RADIUS;
	clear_poly.push_back(pt);
	
	planner_costmap_ros->setConvexPolygonCost(clear_poly, costmap_2d::FREE_SPACE);
	
	costmap_2d::Costmap2D my_costmap;
	geometry_msgs::PoseStamped newgoal_tmp;
	planner_costmap_ros->getCostmapCopy(my_costmap);
	
	newgoal_tmp = goal_tmp;
	float goal_ang = atan2(goal_tmp.pose.position.y,goal_tmp.pose.position.x);
	for(float n = goal_ang; n<=goal_ang+M_PI; n+=M_PI/20)
	{
		newgoal_tmp.pose.position.x = goal_tmp.pose.position.x - GOAL_RADIUS*cos(n);
		newgoal_tmp.pose.position.y = goal_tmp.pose.position.y - GOAL_RADIUS*sin(n);
		unsigned int x,y;
		my_costmap.worldToMap(newgoal_tmp.pose.position.x, newgoal_tmp.pose.position.y, x,y);
		if(my_costmap.getCost(x,y)<200) break;
	}
	
	tf::Stamped<tf::Pose> robot_pose_tf;
	geometry_msgs::PoseStamped robot_pose;
	std::vector<geometry_msgs::PoseStamped> path;
	
	planner_costmap_ros->getRobotPose(robot_pose_tf);
	tf::poseStampedTFToMsg(robot_pose_tf, robot_pose);
	
	
	geometry_msgs::Twist cmd_vel;
	cmd_vel.linear.x = 0.0f;
	cmd_vel.linear.y = 0.0f;
	cmd_vel.angular.z = 0.0f;
	
	if(!planner->makePlan(robot_pose,newgoal_tmp,path))
	{
		pub_vel.publish(cmd_vel);
		ROS_WARN("Could not make a plan.........");
		return;
	}
	
	//path.push_back(robot_pose);
	//path.push_back(*goal);
	if(!tc->setPlan(path))
	{
		pub_vel.publish(cmd_vel);
		ROS_WARN("Could not set a plan.........");
		return;	
	}
	
	if(!tc->computeVelocityCommands(cmd_vel))
	{
		pub_vel.publish(cmd_vel);
		ROS_WARN("Could not generate velocity.........");
		return;	
	}	
	pub_vel.publish(cmd_vel);
	ROS_INFO("Command velocity %f %f %f",cmd_vel.linear.x,cmd_vel.linear.y,cmd_vel.angular.z);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "base_planner");

	tf::TransformListener tf(ros::Duration(10));
	
    planner_costmap_ros = new costmap_2d::Costmap2DROS("costmap", tf);

	pluginlib::ClassLoader<nav_core::BaseGlobalPlanner> bgp_loader("nav_core", "nav_core::BaseGlobalPlanner");
	
	planner = bgp_loader.createClassInstance("navfn/NavfnROS");//carrot_planner/CarrotPlanner");//"navfn/NavfnROS"
	planner->initialize(bgp_loader.getName("navfn/NavfnROS"), planner_costmap_ros);

	pluginlib::ClassLoader<nav_core::BaseLocalPlanner> blp_loader("nav_core", "nav_core::BaseLocalPlanner");
	tc = blp_loader.createClassInstance("eband_local_planner/EBandPlannerROS");//"base_local_planner/TrajectoryPlannerROS");
	tc->initialize(blp_loader.getName("eband_local_planner/EBandPlannerROS"), &tf, planner_costmap_ros);

	ros::NodeHandle n;
	ros::Timer timer = n.createTimer(ros::Duration(0.1), boost::bind(&trajectory_con, boost::ref(tf)));
	ros::Subscriber goal_rviz = n.subscribe("move_base_simple/goal", 1, goalrvizCallback);
	ros::Subscriber goal_target = n.subscribe("target_pose", 1, goalCallback);
	pub_vel = n.advertise<geometry_msgs::Twist>("cmd_vel",1);

	ros::spin();
}	
