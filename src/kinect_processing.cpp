#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <sensor_msgs/PointCloud2.h>

#include <shape_msgs/Plane.h>
#include <pcl_ros/transforms.h>

//#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/radius_outlier_removal.h>

#define ROBOT_RADIUS    0.55f
#define MAX_DIS         3.5f

#define VOXEL_SIZE  0.02f

std::string robot_frame = "/base_link";
std::string pan_frame = "/pan_link";
ros::Publisher cloud_pub,virtual_scan_pub,cloud_tf;
tf::TransformListener* listener;
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
bool new_cloud_available = false;
PointCloudT::Ptr cloud_obj (new PointCloudT);

void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_in)
{         
    try{
        if(new_cloud_available)
            return;
	
        PointCloudT::Ptr cloud (new PointCloudT);
        pcl::fromROSMsg(*cloud_in,*cloud);
        listener->waitForTransform(robot_frame, cloud_in->header.frame_id, cloud_in->header.stamp, ros::Duration(1.0));
        pcl_ros::transformPointCloud(robot_frame, *cloud, *cloud_obj, *listener);
        sensor_msgs::PointCloud2 cloud_tf_out;
        pcl::toROSMsg(*cloud_obj,cloud_tf_out);
        cloud_tf.publish(cloud_tf_out);
        new_cloud_available = true;
      }
      catch(tf::TransformException& ex){
        ROS_ERROR("Received an exception trying to transform a point from %s to %s: %s", cloud_in->header.frame_id.c_str(),pan_frame.c_str(),ex.what());
      }
}

//void processObstacle(const sensor_msgs::PointCloud2::ConstPtr& cloud_in)
void processObstacle()
{         
    if(!new_cloud_available)
	return;

    ROS_INFO("Camera points : %d", (int)cloud_obj->points.size());
    
    // sampling data
    pcl::VoxelGrid<PointT> voxelGrid;
    PointCloudT::Ptr cloud_sampling (new PointCloudT);
    voxelGrid.setInputCloud (cloud_obj);
    voxelGrid.setLeafSize (VOXEL_SIZE, VOXEL_SIZE, VOXEL_SIZE);
    voxelGrid.filter (*cloud_sampling);

    // remove ground plane
    pcl::PassThrough<PointT> passThrough;
    PointCloudT::Ptr cloud_partition (new PointCloudT);
    passThrough.setInputCloud (cloud_sampling);
    passThrough.setFilterFieldName ("z");
    passThrough.setFilterLimits (0.2, 2.0);
    passThrough.filter (*cloud_partition);

    if (cloud_partition->points.size() == 0) return;
    // project point to ground plane
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    coefficients->values.resize (4);
    coefficients->values[0] = coefficients->values[1] = coefficients->values[3] = 0;
    coefficients->values[2] = 0.1;

    pcl::ProjectInliers<PointT> proj;
    PointCloudT::Ptr cloud_project (new PointCloudT);
    proj.setModelType (pcl::SACMODEL_PLANE);
    proj.setInputCloud (cloud_partition);
    proj.setModelCoefficients (coefficients);
    proj.filter (*cloud_project);

    // sampling again
    voxelGrid.setInputCloud (cloud_project);
    PointCloudT::Ptr cloud_sampling_2 (new PointCloudT);
    voxelGrid.setLeafSize (VOXEL_SIZE, VOXEL_SIZE, VOXEL_SIZE);
    voxelGrid.filter (*cloud_sampling_2);
    
    // remove outlier
    pcl::RadiusOutlierRemoval<PointT> outrem;
    PointCloudT::Ptr cloud_clustered (new PointCloudT);
    outrem.setInputCloud (cloud_sampling_2);
    outrem.setRadiusSearch (0.4);
    outrem.setMinNeighborsInRadius (6);
    outrem.filter (*cloud_clustered);

    // remove if near to robot
    PointCloudT::Ptr virtual_scan (new PointCloudT);
    for(unsigned int n = 0; n < cloud_clustered->points.size(); ++n)
    {
        float x = cloud_clustered->points[n].x;
        float y = cloud_clustered->points[n].y;
        float distance = x*x + y*y;
        if( distance  > ROBOT_RADIUS*ROBOT_RADIUS && distance < MAX_DIS*MAX_DIS)
        {
            virtual_scan->points.push_back(cloud_clustered->points[n]);
        }
    }

    virtual_scan->width = virtual_scan->points.size();
    virtual_scan->height = 1;
    virtual_scan->is_dense = true;
    virtual_scan->header = cloud_obj->header;

    ROS_INFO("Virtual scan : %d", (int)virtual_scan->points.size());
   
    sensor_msgs::PointCloud2 cloud_out;
    pcl::toROSMsg(*virtual_scan,cloud_out);
    cloud_out.header.stamp = ros::Time::now();
    virtual_scan_pub.publish(cloud_out);
    new_cloud_available = false;    
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "kinect_processing");
    ros::NodeHandle n;
    ros::Subscriber cloub_sub = n.subscribe("/camera/depth_registered/points", 1, cloudCallback);
    //ros::Subscriber cloub_sub = n.subscribe("/depth_registered/depth_registered/points", 1, cloudCallback);
    //ros::Subscriber obstacle = n.subscribe("/camera/cloud_tf", 1, processObstacle);
    cloud_tf = n.advertise<sensor_msgs::PointCloud2>("/camera/cloud_tf",1);
    virtual_scan_pub = n.advertise<sensor_msgs::PointCloud2>("/kinect/virtual_scan",1);
    listener = new tf::TransformListener();
    ros::Rate loop_rate(10);
    while(ros::ok())
    {
    	processObstacle();
        ros::spinOnce();
	loop_rate.sleep();
    }
    //ros::spin();

    return 0;
}
