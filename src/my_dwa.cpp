#include <my_dwa.h>

/*void velEstCallback(const geometry_msgs::TwistStamped::ConstPtr& vel)
{
	static unsigned int init = 0;
	static ros::Time last_time;
	ros::Time current_time;
	float dt;

	tf::Stamped<tf::Pose> robot_pose_tf;
	geometry_msgs::PoseStamped robot_pose;
	float pose[3];
	my_costmap_ros->getCostmapCopy(my_costmap);
	my_costmap_ros->getRobotPose(robot_pose_tf);
	tf::poseStampedTFToMsg(robot_pose_tf, robot_pose);

	pose[0] = robot_pose.pose.position.x;
	pose[1] = robot_pose.pose.position.y;
	pose[2] = tf::getYaw(robot_pose.pose.orientation);

	float phi_it_range,p_it_range;
	float dphi_it,dp_it;
	float vel_x,vel_y;
	float vel_samp[2],vel_samp_dir;
	float vel_samp_r_sq;
	float breaking_dis;
	vel_x = vel->twist.linear.x;
	vel_y = vel->twist.linear.y;

	float best_vel[3] = {0,0,0};
	float obstacle_cost,velocity_cost,alignment_cost, total_cost, optimal_cost;
	optimal_cost = 0;
	float curr_goal_distX, curr_goal_distY, curr_goal_dist, curr_goal_th, sub_curr_goal_th, alignment_th;

	current_time = vel->header.stamp;
	if(!init)
	{
		dt = 1;
		init = 1;
	}
	else
	{
		dt = (current_time - last_time).toSec();
		last_time = current_time;
	}
	
	//prepare for velocity iteration.
	p_it_range = ACCEL_MAX*dt;
	dp_it = p_it_range/P_SAMPLE_TIMES;

	phi_it_range = 2*PI;
	dphi_it = phi_it_range/PHI_SAMPLE_TIMES;

	curr_goal_distX = my_goal[0]-pose[0];
	curr_goal_distY = my_goal[1]-pose[1];
	curr_goal_dist = sqrt(curr_goal_distX*curr_goal_distX + curr_goal_distY*curr_goal_distY);
	curr_goal_th = atan2(curr_goal_distY,curr_goal_distX) - pose[2];
	if(curr_goal_th >= PI) curr_goal_th -= 2*PI;
	else if(curr_goal_th < -PI) curr_goal_th += 2*PI;

	if(curr_goal_dist > GOAL_FINISH_DIST)
	{
		sub_curr_goal_th = curr_goal_th;
	/////////////////////////////////////////////////////////////
		std::vector<geometry_msgs::PoseStamped> path;
		
		if(0)//my_navfn->makePlan(robot_pose,robot_goal,path))// && my_eband->setPlan(path))
		{			
			//std::vector<eband_local_planner::Bubble> current_band;
			//my_eband->optimizeBand();
			//my_eband->getBand(current_band);
			//eband_visual->publishBand("my_bubbles", current_band);
			if(path.size()>4)//current_band.size()>1)
			{
				//sub_curr_goal_th = atan2(current_band.at(1).center.pose.position.y-pose[1],current_band.at(1).center.pose.position.x-pose[0]) - pose[2];
			sub_curr_goal_th = atan2(path[4].pose.position.y-pose[1],path[4].pose.position.x-pose[0]) - pose[2];	 
				if(sub_curr_goal_th >= PI) sub_curr_goal_th -= 2*PI;
				else if(sub_curr_goal_th < -PI) sub_curr_goal_th += 2*PI;
				//ROS_INFO("band1: %f, %f .....................",sub_curr_goal_th,curr_goal_th);//,);
			}
		}
	////////////////////////////////////////////////////////////
	//Modify goal heading
	//sub_curr_goal_th = modified_goalheading(pose,sub_curr_goal_th,0.5);
	//ROS_INFO("Goal heading: %f.....................",sub_curr_goal_th);

		
		//iteration velocity in polar coordinate.
		for(float p_it = 0; p_it <= p_it_range; p_it+=dp_it){
			for(float phi_it = 0; phi_it < phi_it_range; phi_it+=dphi_it){
	
				//add iteration velocity to current velocity in cartesian coordinate.
				vel_samp[0] = vel_x + p_it*cos(phi_it);
				vel_samp[1] = vel_y + p_it*sin(phi_it);
	
				//check that it is over the maximum velocity.
				vel_samp_r_sq = vel_samp[0]*vel_samp[0] + vel_samp[1]*vel_samp[1];
				if(vel_samp_r_sq > VEL_MAX*VEL_MAX || vel_samp_r_sq < VEL_MIN*VEL_MIN) break;
				
				//breaking distance at that velocity.
				breaking_dis = vel_samp_r_sq/(2*DEACCEL_MAX); //vel^2/(2*accel_break)
	
				vel_samp_dir = atan2(vel_samp[1],vel_samp[0]);
				if(isVelAdmissible(pose,vel_samp_dir,breaking_dis+0.3,OBSERVE_DIST,&obstacle_cost))
				{
					if(curr_goal_dist>GOAL_REGION_DIST) velocity_cost = sqrt(vel_samp_r_sq)/VEL_MAX;
					else velocity_cost = 1 - sqrt(vel_samp_r_sq)/VEL_MAX;
	
					alignment_th = vel_samp_dir-sub_curr_goal_th;
					if(alignment_th >= PI) alignment_th -= 2*PI;
					else if(alignment_th < -PI) alignment_th += 2*PI;
	
					alignment_cost = 1 - (fabs(alignment_th)/PI);
	
					//total_cost = OBSTABLE_COST_GAIN*obstacle_cost + VELOCITY_COST_GAIN*velocity_cost + ALIGNMENT_COST_GAIN*alignment_cost;
					total_cost = OBSTABLE_COST_GAIN*obstacle_cost + VELOCITY_COST_GAIN*velocity_cost + ALIGNMENT_COST_GAIN*alignment_cost;
					if(total_cost > optimal_cost)
					{
						best_vel[0] = vel_samp[0];
						best_vel[1] = vel_samp[1];
						optimal_cost = total_cost;
					}
	
					//ROS_INFO("vel_x: %f, vel_y: %f, cost: %f", vel_samp[0],vel_samp[1],obstacle_cost);	
				}
				if(p_it == 0) break;
			}
		}
		//ROS_INFO("END LOOP..................");
		best_vel[2] = curr_goal_th;	
	}

	
	//if(best_vel[2] >= PI) best_vel[2] -= 2*PI;
	//else if(best_vel[2] < -PI) best_vel[2] += 2*PI;
	
	best_vel[2] = 4*best_vel[2];
	if(best_vel[2]>0.4) best_vel[2] = 0.4;
	else if(best_vel[2] <-0.4) best_vel[2] = -0.4;

	geometry_msgs::TwistStamped dwa_cmd_vel;
	dwa_cmd_vel.header.stamp = ros::Time::now();
	dwa_cmd_vel.header.frame_id = my_costmap_ros->getBaseFrameID();
	dwa_cmd_vel.twist.linear.x = best_vel[0];
	dwa_cmd_vel.twist.linear.y = best_vel[1];
	dwa_cmd_vel.twist.angular.z = best_vel[2];
	pub_vel.publish(dwa_cmd_vel);
	//ROS_INFO("Best velocity: %f, %f .....................",best_vel[0],best_vel[1]);
}*/

//for mecanum robot
geometry_msgs::TwistStamped vel;
void velEstCallback(const geometry_msgs::TwistStamped::ConstPtr& vel_in)
{
	vel = *vel_in;
}

float modified_goalheading(float* pose,float goal,float dist)
{
	unsigned int x,y;
	unsigned char cost;
	float new_goal = goal;
	for(float iter_ang = 0;iter_ang <= PI;iter_ang+=0.3)
	{
		for(float search_dist = 0; search_dist <= dist; search_dist+=my_costmap.getResolution())
		{
			my_costmap.worldToMap(pose[0] + search_dist*cos(goal+pose[2]+iter_ang), pose[1] + search_dist*sin(goal+pose[2]+iter_ang), x,y);
			cost = my_costmap.getCost(x,y);
			if(cost>MAXIMUM_COLLISION_FREE_COSTMAP_VALUE) break;
		}
		if(cost<=MAXIMUM_COLLISION_FREE_COSTMAP_VALUE)
		{
			new_goal +=iter_ang;
			if(new_goal >= PI) new_goal -= 2*PI;
			else if(new_goal < -PI) new_goal += 2*PI;
			return new_goal;
		}
		for(float search_dist = 0; search_dist <= dist; search_dist+=my_costmap.getResolution())
		{
			my_costmap.worldToMap(pose[0] + search_dist*cos(goal+pose[2]-iter_ang), pose[1] + search_dist*sin(goal+pose[2]-iter_ang), x,y);
			cost = my_costmap.getCost(x,y);
			if(cost>MAXIMUM_COLLISION_FREE_COSTMAP_VALUE) break;
		}
		if(cost<=MAXIMUM_COLLISION_FREE_COSTMAP_VALUE)
		{
			new_goal -=iter_ang;
			if(new_goal >= PI) new_goal -= 2*PI;
			else if(new_goal < -PI) new_goal += 2*PI;
			return new_goal;
		}
	}
	return new_goal;
}
bool isVelAdmissible(float* pose,float vel_dir,float breaking_dist,float observe_dist,float* obstacle_cost)
{
	unsigned int x,y;
	unsigned char cost;
	float max_dist;
	
	if(observe_dist<breaking_dist) observe_dist = breaking_dist;
	max_dist = observe_dist;
	
	for(float raytrace_dist = 0; raytrace_dist <= observe_dist; raytrace_dist+=my_costmap.getResolution())
	{	
		my_costmap.worldToMap(pose[0] + raytrace_dist*cos(vel_dir+pose[2]), pose[1] + raytrace_dist*sin(vel_dir+pose[2]), x,y);
		cost = my_costmap.getCost(x,y);

		if(raytrace_dist <= breaking_dist && cost>MAXIMUM_COLLISION_FREE_COSTMAP_VALUE)
		{
			*obstacle_cost = 0.0;
			return false;
		}
		else if(cost>MAXIMUM_COLLISION_FREE_COSTMAP_VALUE)
		{ 
			max_dist = raytrace_dist;
			break;
		}
	}
	*obstacle_cost = max_dist/observe_dist;	
	return true;
}

void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& goal)
{
	//tf::Stamped<tf::Pose> robot_pose_tf;
	//geometry_msgs::PoseStamped robot_pose;
	//my_costmap_ros->getCostmapCopy(my_costmap);
	//my_costmap_ros->getRobotPose(robot_pose_tf);
	//tf::poseStampedTFToMsg(robot_pose_tf, robot_pose);

	my_goal[0] = goal->pose.position.x;
	my_goal[1] = goal->pose.position.y;
	my_goal[2] = tf::getYaw(goal->pose.orientation);
	robot_goal = *goal;

	//std::vector<geometry_msgs::PoseStamped> path;
	//planner_->makePlan(robot_pose,robot_goal,path);
	//ROS_INFO("Best velocity: %f,%f...........................",path[0].pose.position.x,robot_pose.pose.position.x);
}

void doAvoidance(void)
{
	static unsigned int init = 0;
	static ros::Time last_time;
	ros::Time current_time;
	float dt;

	tf::Stamped<tf::Pose> robot_pose_tf;
	geometry_msgs::PoseStamped robot_pose;
	float pose[3];
	my_costmap_ros->getCostmapCopy(my_costmap);
	my_costmap_ros->getRobotPose(robot_pose_tf);
	tf::poseStampedTFToMsg(robot_pose_tf, robot_pose);

	pose[0] = robot_pose.pose.position.x;
	pose[1] = robot_pose.pose.position.y;
	pose[2] = tf::getYaw(robot_pose.pose.orientation);

	float phi_it_range,p_it_range;
	float dphi_it,dp_it;
	float vel_x,vel_y;
	float vel_samp[2],vel_samp_dir;
	float vel_samp_r_sq;
	float breaking_dis;
	vel_x = vel.twist.linear.x;
	vel_y = vel.twist.linear.y;

	float best_vel[3] = {0,0,0};
	float obstacle_cost,velocity_cost,alignment_cost, total_cost, optimal_cost;
	optimal_cost = 0;
	float curr_goal_distX, curr_goal_distY, curr_goal_dist, curr_goal_th, sub_curr_goal_th, alignment_th;

	current_time = vel.header.stamp;
	if(!init)
	{
		dt = 1;
		init = 1;
	}
	else
	{
		dt = (current_time - last_time).toSec();
		last_time = current_time;
	}
	
	//prepare for velocity iteration.
	p_it_range = ACCEL_MAX*dt;
	dp_it = p_it_range/P_SAMPLE_TIMES;

	phi_it_range = 2*PI;
	dphi_it = phi_it_range/PHI_SAMPLE_TIMES;

	curr_goal_distX = my_goal[0]-pose[0];
	curr_goal_distY = my_goal[1]-pose[1];
	curr_goal_dist = sqrt(curr_goal_distX*curr_goal_distX + curr_goal_distY*curr_goal_distY);
	curr_goal_th = atan2(curr_goal_distY,curr_goal_distX) - pose[2];
	
	if(curr_goal_th >= PI) curr_goal_th -= 2*PI;
	else if(curr_goal_th < -PI) curr_goal_th += 2*PI;

	if(curr_goal_dist > GOAL_FINISH_DIST)
	{
		sub_curr_goal_th = curr_goal_th;
	/////////////////////////////////////////////////////////////
		std::vector<geometry_msgs::PoseStamped> path;
		
		//iteration velocity in polar coordinate.
		for(float p_it = 0; p_it <= p_it_range; p_it+=dp_it){
			for(float phi_it = 0; phi_it < phi_it_range; phi_it+=dphi_it){
	
				//add iteration velocity to current velocity in cartesian coordinate.
				vel_samp[0] = vel_x + p_it*cos(phi_it);
				vel_samp[1] = vel_y + p_it*sin(phi_it);
	
				//check that it is over the maximum velocity.
				vel_samp_r_sq = vel_samp[0]*vel_samp[0] + vel_samp[1]*vel_samp[1];
				if(vel_samp_r_sq > VEL_MAX*VEL_MAX || vel_samp_r_sq < VEL_MIN*VEL_MIN) break;
				
				//breaking distance at that velocity.
				breaking_dis = vel_samp_r_sq/(2*DEACCEL_MAX); //vel^2/(2*accel_break)
	
				vel_samp_dir = atan2(vel_samp[1],vel_samp[0]);
				if(isVelAdmissible(pose,vel_samp_dir,breaking_dis+0.3,OBSERVE_DIST,&obstacle_cost))
				{
					if(curr_goal_dist>GOAL_REGION_DIST) velocity_cost = sqrt(vel_samp_r_sq)/VEL_MAX;
					else velocity_cost = 1 - sqrt(vel_samp_r_sq)/VEL_MAX;
	
					alignment_th = vel_samp_dir-sub_curr_goal_th;
					if(alignment_th >= PI) alignment_th -= 2*PI;
					else if(alignment_th < -PI) alignment_th += 2*PI;
	
					alignment_cost = 1 - (fabs(alignment_th)/PI);
	
					total_cost = OBSTABLE_COST_GAIN*obstacle_cost + VELOCITY_COST_GAIN*velocity_cost + ALIGNMENT_COST_GAIN*alignment_cost;
					//total_cost = VELOCITY_COST_GAIN*velocity_cost + ALIGNMENT_COST_GAIN*alignment_cost;
					if(total_cost > optimal_cost)
					{
						best_vel[0] = vel_samp[0];
						best_vel[1] = vel_samp[1];
						optimal_cost = total_cost;
					}
	
					//ROS_INFO("vel_x: %f, vel_y: %f, cost: %f", vel_samp[0],vel_samp[1],obstacle_cost);	
				}
				if(p_it == 0) break;
			}
		}
		//ROS_INFO("END LOOP..................");
		best_vel[2] = curr_goal_th;	
	}

	
	//if(best_vel[2] >= PI) best_vel[2] -= 2*PI;
	//else if(best_vel[2] < -PI) best_vel[2] += 2*PI;
	
	best_vel[2] = 4*best_vel[2];
	if(best_vel[2]>0.4) best_vel[2] = 0.4;
	else if(best_vel[2] <-0.4) best_vel[2] = -0.4;

	geometry_msgs::TwistStamped dwa_cmd_vel;
	dwa_cmd_vel.header.stamp = ros::Time::now();
	dwa_cmd_vel.header.frame_id = my_costmap_ros->getBaseFrameID();
	dwa_cmd_vel.twist.linear.x = best_vel[0];
	dwa_cmd_vel.twist.linear.y = best_vel[1];
	dwa_cmd_vel.twist.angular.z = best_vel[2];
	pub_vel.publish(dwa_cmd_vel);
	//ROS_INFO("Best velocity: %f, %f .....................",best_vel[0],best_vel[1]);
	
}
int main(int argc, char** argv)
{
	ros::init(argc, argv, "dma_planner");

	tf::TransformListener tf(ros::Duration(10));
	costmap_2d::Costmap2DROS costmap_ros("costmap", tf);
	/*my_costmap_ros = &costmap_ros;

	//navfn::NavfnROS navfn("my_navfn_planner", &costmap_ros);
	//my_navfn = &navfn;
	robot_goal.header.frame_id = my_costmap_ros->getGlobalFrameID();

	//eband_local_planner::EBandPlanner eband("eband_planner",&costmap_ros);
	//my_eband = &eband;

	ros::NodeHandle n;
	//eband_local_planner::EBandVisualization eband_visual;//(n,&costmap_ros);
	
	//eband_visual = boost::shared_ptr<eband_local_planner::EBandVisualization>(new eband_local_planner::EBandVisualization);
	//eband.setVisualization(eband_visual);
	//eband_visual->initialize(n,&costmap_ros);
	//my_eband_visual = &eband_visual;

	//pluginlib::ClassLoader<nav_core::BaseGlobalPlanner> bgp_loader_("nav_core", "nav_core::BaseGlobalPlanner");
	//ROS_INFO("%u",bgp_loader_.isClassAvailable("SBPLLatticePlanner"));
	//planner_ = bgp_loader_.createClassInstance("SBPLLatticePlanner");
	//planner_->initialize(bgp_loader_.getName("SBPLLatticePlanner"), &costmap_ros);
	
	//nav_core::BaseGlobalPlanner nav_core;
	//nav_core.initialize("my_nav_core",&costmap_ros);
	//sbpl_lattice_planner::SBPLLatticePlanner my_sbpl("SBPLLatticePlanner",&costmap_ros);
	

	//ros::NodeHandle n;
	ros::Subscriber sub_vel, sub_goal, sub_estvel;

	sub_estvel = n.subscribe("vel_est", 1, velEstCallback);
	//sub_vel = n.subscribe("vel_icp", 1000, velCallback);
	sub_goal = n.subscribe("move_base_simple/goal", 1, goalCallback);
	pub_vel = n.advertise<geometry_msgs::TwistStamped>("cmd_vel",1);
	ros::Timer timer = n.createTimer(ros::Duration(0.1), boost::bind(&doAvoidance));*/
	ros::spin();
}	
