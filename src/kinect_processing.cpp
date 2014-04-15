#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud.h>
#include <shape_msgs/Plane.h>
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


const float voxel_size = 0.02f;
const float min_height = -0.3f;
const float max_height = 2.0f;
const float ground_approx_height = 0.3f;
const float ground_disto_thres = 0.07f;

#define ROBOT_RADIUS	0.55f
//4m/0.05m = 80 (L:-2.0m, R:+2.0m, Step_size: 0.05m)
#define SCAN_START 	-2.0f
#define SCAN_STOP		2.0f
#define STEP_SIZE		0.05f
#define ARRAY_SIZE 	80 //((SCAN_STOP - SCAN_START)/STEP_SIZE)
#define MAX_DIS			4.0f

std::string robot_frame = "/base_link";
std::string pan_frame = "/pan_link";
ros::Publisher cloud_pub,cloud_pub2,ground_coeffs_pub,cloud_tf;
tf::TransformListener* listener;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

PointCloudT::Ptr cloud_obj (new PointCloudT);
bool new_cloud_available_flag = false;

void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_in)
{ 		
  	try{
	  	PointCloudT::Ptr cloud (new PointCloudT);
	  	pcl::fromROSMsg(*cloud_in,*cloud);
	  	listener->waitForTransform(robot_frame, cloud_in->header.frame_id, cloud_in->header.stamp, ros::Duration(1.0));
	  	pcl_ros::transformPointCloud(robot_frame, *cloud, *cloud_obj, *listener);
		sensor_msgs::PointCloud2 cloud_tf_out;
 		pcl::toROSMsg(*cloud_obj,cloud_tf_out);
 		cloud_tf.publish(cloud_tf_out);
  		new_cloud_available_flag = true;
  	}
  	catch(tf::TransformException& ex){
  		ROS_ERROR("Received an exception trying to transform a point from %s to %s: %s", cloud_in->header.frame_id.c_str(),pan_frame.c_str(),ex.what());
  	}
}

float getCameraHeight()
{
	  tf::StampedTransform transform;
    try{
      listener->lookupTransform(robot_frame, pan_frame, ros::Time(0), transform);
			return transform.getOrigin().z();
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
			return -1;
    }	
}
void doProcessing()
{
	float camera_height = getCameraHeight();
	if(camera_height < 0)
	{
		ROS_ERROR("Invalid camera height");
		return;
	}
	ROS_INFO("camera_height %f", camera_height);
	ROS_INFO("cloud_raw: %d",cloud_obj->points.size()); 

  pcl::VoxelGrid<PointT> vg;
 	PointCloudT::Ptr cloud_voxel (new PointCloudT);
  vg.setInputCloud (cloud_obj);
  vg.setLeafSize (voxel_size, voxel_size, voxel_size);
  vg.filter (*cloud_voxel);

	//ROS_INFO("cloud_voxel: %d",cloud_voxel->points.size());

 	// Create the filtering object
 	pcl::PassThrough<PointT> pass;
  PointCloudT::Ptr cloud_passth (new PointCloudT);
  pass.setInputCloud (cloud_voxel);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (min_height - camera_height, max_height - camera_height);
  pass.filter (*cloud_passth);
	
ROS_INFO("cloud_passth: %d",cloud_passth->points.size());

	/*pcl::PointIndices::Ptr robot_inliers (new pcl::PointIndices ());
	robot_inliers->header = cloud_passth->header;
	for(unsigned int n = 0; n < cloud_passth->points.size(); n++)
	{
		float x_tmp = cloud_passth->points[n].x;
		float y_tmp = cloud_passth->points[n].y;
		if(x_tmp*x_tmp + y_tmp*y_tmp < ROBOT_RADIUS*ROBOT_RADIUS) robot_inliers->indices.push_back(n);		
	}

	pcl::ExtractIndices<PointT> robot_extract;
	PointCloudT::Ptr cloud_no_robot (new PointCloudT);

	robot_extract.setInputCloud (cloud_passth);
	robot_extract.setIndices (robot_inliers);
	robot_extract.setNegative (true);
	robot_extract.filter (*cloud_no_robot);*/	

//ROS_INFO("cloud_no_robot: %d",cloud_no_robot->points.size());
	
	PointCloudT::Ptr cloud_upper (new PointCloudT);
	PointCloudT::Ptr cloud_lower (new PointCloudT);  
	pass.setInputCloud (cloud_passth);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (min_height - camera_height, ground_approx_height - camera_height);
  pass.filter (*cloud_lower);
	pass.setFilterLimitsNegative (true);
	pass.filter (*cloud_upper);

	PointCloudT::Ptr cloud_no_ground (new PointCloudT);
	PointCloudT::Ptr cloud_no_ground_merged (new PointCloudT);

	shape_msgs::Plane coeffs_tmp;
	coeffs_tmp.coef[0] = 0;
	coeffs_tmp.coef[1] = 0;
	coeffs_tmp.coef[2] = 1;			
	coeffs_tmp.coef[3] = camera_height;
	if(cloud_lower->points.size() > 30)
	{
		pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
		pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
		// Create the segmentation object
		pcl::SACSegmentation<PointT> seg;
		// Optional
		seg.setOptimizeCoefficients (true);
		// Mandatory
		seg.setModelType (pcl::SACMODEL_PLANE);
		seg.setMethodType (pcl::SAC_RANSAC);
		seg.setMaxIterations (20);
		seg.setDistanceThreshold (ground_disto_thres);

		seg.setInputCloud (cloud_lower);
		seg.segment (*inliers, *coefficients);


		if (inliers->indices.size () > 5 && coefficients->values[2] > 0.8)
		{
			std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
		                                    << coefficients->values[1] << " "
		                                    << coefficients->values[2] << " " 
		                                    << coefficients->values[3] << std::endl;
			
			coeffs_tmp.coef[0] = coefficients->values[0];
			coeffs_tmp.coef[1] = coefficients->values[1];
			coeffs_tmp.coef[2] = coefficients->values[2];			
			coeffs_tmp.coef[3] = coefficients->values[3];
			
			// Create the filtering object
			pcl::ExtractIndices<PointT> extract;

			extract.setInputCloud (cloud_lower);
			extract.setIndices (inliers);
			extract.setNegative (true);
			extract.filter (*cloud_no_ground);

			cloud_no_ground_merged->points.insert(cloud_no_ground_merged->points.end(),cloud_no_ground->points.begin(),cloud_no_ground->points.end());
		}
		else std::cerr << "Could not estimate a planar model for the given data." << std::endl;

	}
	ground_coeffs_pub.publish(coeffs_tmp);
	cloud_no_ground_merged->points.insert(cloud_no_ground_merged->points.end(),cloud_upper->points.begin(),cloud_upper->points.end());
	

	cloud_no_ground_merged->width = cloud_no_ground_merged->points.size();
 	cloud_no_ground_merged->height = 1;
  cloud_no_ground_merged->is_dense = true;
	cloud_no_ground_merged->header = cloud_upper->header;

 	sensor_msgs::PointCloud2 cloud_out;
	pcl::toROSMsg(*cloud_no_ground_merged,cloud_out);
 	cloud_out.header.stamp = ros::Time::now();
 	//cloud_out.header.frame_id = robot_frame;
	cloud_pub.publish(cloud_out);


	float col_min[ARRAY_SIZE];
	unsigned int min_index[ARRAY_SIZE];

	for(unsigned int n = 0; n < ARRAY_SIZE; n++) col_min[n] = 1000.0f;

	for(unsigned int n = 0; n < cloud_no_ground_merged->points.size(); n++)
	{
		float horizon_val = cloud_no_ground_merged->points[n].y;
		if(horizon_val < SCAN_START || horizon_val > SCAN_STOP-STEP_SIZE) continue;
		unsigned int ind = (unsigned int)((horizon_val - SCAN_START)/STEP_SIZE);
		
		float dis_val = cloud_no_ground_merged->points[n].x;
		if(dis_val > ROBOT_RADIUS && dis_val < col_min[ind]) 
		{
			col_min[ind] = dis_val;
			min_index[ind] = n;
		}
	}
	
	PointCloudT::Ptr virtual_scan (new PointCloudT);
	for(unsigned int n = 0; n < ARRAY_SIZE; n++)
	{
		if(col_min[n]>MAX_DIS) continue;
		virtual_scan->points.push_back(cloud_no_ground_merged->points[min_index[n]]);
		
		unsigned int last_ind = virtual_scan->points.size()-1;
		virtual_scan->points[last_ind].y = n*STEP_SIZE + SCAN_START;
		virtual_scan->points[last_ind].z = 1.0f;
	}
	
	virtual_scan->width = virtual_scan->points.size();
 	virtual_scan->height = 1;
  virtual_scan->is_dense = true;
	virtual_scan->header = cloud_no_ground_merged->header;

 	sensor_msgs::PointCloud2 cloud_out2;
	pcl::toROSMsg(*virtual_scan,cloud_out2);
 	cloud_out2.header.stamp = ros::Time::now();
 	//cloud_out2.header.frame_id = robot_frame;
	cloud_pub2.publish(cloud_out2);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "kinect_processing");
  ros::NodeHandle n;
  ros::Subscriber cloub_sub = n.subscribe("/camera/depth_registered/points", 1, cloudCallback);
  cloud_pub = n.advertise<sensor_msgs::PointCloud2>("cloud_seg",1);
  cloud_tf = n.advertise<sensor_msgs::PointCloud2>("cloud_tf",1);
  cloud_pub2 = n.advertise<sensor_msgs::PointCloud2>("virtual_scan",1);
	ground_coeffs_pub = n.advertise<shape_msgs::Plane>("ground_coeffs",1);
  listener = new tf::TransformListener();
	ros::Rate loop_rate(10);
  while (ros::ok())
  {
		if(new_cloud_available_flag)
		{
			doProcessing();
			new_cloud_available_flag = false;
			loop_rate.sleep();	
		}
	ros::spinOnce();
	}
  return 0;
}
