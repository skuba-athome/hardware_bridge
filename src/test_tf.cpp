#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_listener");

  ros::NodeHandle node;


  tf::TransformListener listener;

  ros::Rate rate(10.0);
  while (node.ok()){
    tf::StampedTransform transform;
    try{
      listener.lookupTransform("/camera_link", "/camera_depth_optical_frame",  
                               ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
    }	
		
		Eigen::Matrix4f T;
		pcl_ros::transformAsMatrix(transform,T);

		Eigen::MatrixXf coeff_in(1,4); coeff_in << 0.00370782, 0.0210134, 0.999772, 1.03884;
		Eigen::MatrixXf coeff_out(1,4);
		coeff_out = coeff_in*T;
		ROS_INFO("coeff: %f, %f, %f, %f", coeff_out(0,0),coeff_out(0,1),coeff_out(0,2),coeff_out(0,3));
		ROS_INFO("%.2f %.2f %.2f %.2f; %.2f %.2f %.2f %.2f; %.2f %.2f %.2f %.2f; %.2f %.2f %.2f %.2f",T(0,0),T(0,1),T(0,2),T(0,3),
T(1,0),T(1,1),T(1,2),T(1,3),
T(2,0),T(2,1),T(2,2),T(2,3),
T(3,0),T(3,1),T(3,2),T(3,3));
		

    rate.sleep();
  }
  return 0;
};
