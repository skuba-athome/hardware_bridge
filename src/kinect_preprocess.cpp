#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/ros/conversions.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>

std::string robot_frame = "base_link";
std::string pan_frame = "pan_link";
ros::Publisher cloud_pub,cloud_pub2;
tf::TransformListener* listener;


void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_in)
{ 		
  	try{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_tf_new (new pcl::PointCloud<pcl::PointXYZRGB>);
	  	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	  	pcl::fromROSMsg(*cloud_in,*cloud);
	  	listener->waitForTransform(pan_frame, cloud_in->header.frame_id, cloud_in->header.stamp, ros::Duration(1.0));
	  	pcl_ros::transformPointCloud(pan_frame, *cloud, *cloud_tf_new, *listener);
  		sensor_msgs::PointCloud2 cloud_tf_out;
		pcl::toROSMsg(*cloud_tf_new,cloud_tf_out);
  		cloud_pub.publish(cloud_tf_out);


  	}
  	catch(tf::TransformException& ex){
  		ROS_ERROR("Received an exception trying to transform a point from %s to %s: %s", cloud_in->header.frame_id.c_str(),pan_frame.c_str(),ex.what());
  	}
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "kinect_preprocess");
  ros::NodeHandle n;
  ros::Subscriber	cloub_sub = n.subscribe("/camera/depth_registered/points", 1, cloudCallback);
  cloud_pub = n.advertise<sensor_msgs::PointCloud2>("cloud_tf",1);
  listener = new tf::TransformListener();
  ros::spin();
  return 0;
}
