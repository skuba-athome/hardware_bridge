#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

void robot_transformBroadcaster()
{
	static tf::TransformBroadcaster broadcaster;
	broadcaster.sendTransform(
	tf::StampedTransform(
	tf::Transform(tf::createQuaternionFromRPY(0.0, 0.0, 0.0), tf::Vector3(0.0, 0.0, 0.95)),
	ros::Time::now(),"base_link", "camera_link"));
}

int main(int argc, char** argv){
  	ros::init(argc, argv, "cam_tf_publisher");
	ros::NodeHandle n;

  	
	ros::Timer timer = n.createTimer(ros::Duration(0.01), boost::bind(&robot_transformBroadcaster));
	ros::spin();
	return 0;
}
