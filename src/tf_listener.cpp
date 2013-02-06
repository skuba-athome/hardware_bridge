#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/ros/conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

sensor_msgs::PointCloud2 cloud_new;
//ros::Publisher pub_cloud;
bool pass_cloud_cb = false;
void cloudcb(const sensor_msgs::PointCloud2::ConstPtr& cloud_in)
{	
	//pub_cloud.publish(cloud_in);
	pass_cloud_cb = true;
	cloud_new = *cloud_in;
	//ROS_INFO("test");
}

void transformPointCloud(const tf::TransformListener& listener, const ros::Publisher& pub_cloud)
{
	if(!pass_cloud_cb) return;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromROSMsg(cloud_new,*cloud);
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tf (new pcl::PointCloud<pcl::PointXYZ>);
	pcl_ros::transformPointCloud("base_link", *cloud, *cloud_tf, listener);
	
	ROS_INFO("Point in: %u",cloud_tf->points.size());
	
  	pcl::VoxelGrid<pcl::PointXYZ> vg;
 	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_voxel (new pcl::PointCloud<pcl::PointXYZ>);
  	vg.setInputCloud (cloud_tf);
  	vg.setLeafSize (0.1f, 0.1f, 0.1f);
  	vg.filter (*cloud_voxel);

	ROS_INFO("Point out voxel: %u",cloud_voxel->points.size());
	
  	// Create the filtering object
  	pcl::PassThrough<pcl::PointXYZ> pass;
  	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_passth (new pcl::PointCloud<pcl::PointXYZ>);
  	pass.setInputCloud (cloud_voxel);
  	pass.setFilterFieldName ("z");
  	pass.setFilterLimits (0.1, 2.00);
  	//pass.setFilterLimitsNegative (true);
  	pass.filter (*cloud_passth);
  	
	ROS_INFO("Point out passthrough: %u",cloud_passth->points.size());
	
	sensor_msgs::PointCloud2 cloud_out;
	pcl::toROSMsg(*cloud_passth,cloud_out);
	pub_cloud.publish(cloud_out);
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "cam_tf_listener");
  ros::NodeHandle n;
  tf::TransformListener listener(ros::Duration(10.0));
  ros::Subscriber sub = n.subscribe("/camera/depth/points", 1, cloudcb);
  ros::Publisher pub_cloud = n.advertise<sensor_msgs::PointCloud2>("cloud_tf",1);
  ros::Timer timer = n.createTimer(ros::Duration(0.1), boost::bind(&transformPointCloud, boost::ref(listener),boost::ref(pub_cloud)));
  ros::spin();
  return 0;
}
