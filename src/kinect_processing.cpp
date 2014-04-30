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
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/radius_outlier_removal.h>

#define ROBOT_RADIUS    0.55f
//4m/0.05m = 80 (L:-2.0m, R:+2.0m, Step_size: 0.05m)
#define SCAN_START     -2.0f
#define SCAN_STOP        2.0f
#define STEP_SIZE        0.05f
#define ARRAY_SIZE     80 //((SCAN_STOP - SCAN_START)/STEP_SIZE)
#define MAX_DIS            4.0f

#define VOXEL_SIZE  0.02f

std::string robot_frame = "/base_link";
std::string pan_frame = "/pan_link";
ros::Publisher cloud_pub,virtual_scan_pub,cloud_tf;
tf::TransformListener* listener;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

PointCloudT::Ptr cloud_obj (new PointCloudT);

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
      }
      catch(tf::TransformException& ex){
        ROS_ERROR("Received an exception trying to transform a point from %s to %s: %s", cloud_in->header.frame_id.c_str(),pan_frame.c_str(),ex.what());
      }
}

void processObstacle(const sensor_msgs::PointCloud2::ConstPtr& cloud_in)
{         
    PointCloudT::Ptr cloud_tmp, _cloud_tmp (new PointCloudT);
    ROS_INFO("Camera points : %d", cloud_obj->points.size());

    // sampling data
    pcl::VoxelGrid<PointT> voxelGrid;
    voxelGrid.setInputCloud (cloud_obj);
    voxelGrid.setLeafSize (VOXEL_SIZE, VOXEL_SIZE, VOXEL_SIZE);
    voxelGrid.filter (*cloud_tmp);

    // remove ground plane
    pcl::PassThrough<PointT> passThrough;
    passThrough.setInputCloud (cloud_tmp);
    passThrough.setFilterFieldName ("z");
    passThrough.setFilterLimits (0.15, 2.0);
    passThrough.filter (*_cloud_tmp);

    // project point to ground plane
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    coefficients->values.resize (4);
    coefficients->values[0] = coefficients->values[1] = coefficients->values[3] = 0;
    coefficients->values[2] = 0.1;

    pcl::ProjectInliers<PointT> proj;
    proj.setModelType (pcl::SACMODEL_PLANE);
    proj.setInputCloud (_cloud_tmp);
    proj.setModelCoefficients (coefficients);
    proj.filter (*cloud_tmp);

    // remove outlier
    pcl::RadiusOutlierRemoval<PointT> outrem;
    outrem.setInputCloud (cloud_tmp);
    outrem.setRadiusSearch (0.4);
    outrem.setMinNeighborsInRadius (6);
    outrem.filter (*_cloud_tmp);

    // sampling again
    //voxelGrid.setInputCloud (_cloud_tmp);
    //voxelGrid.setLeafSize (VOXEL_SIZE, VOXEL_SIZE, VOXEL_SIZE);
    //voxelGrid.filter (*cloud_tmp);

    // remove if near to robot
    PointCloudT::Ptr virtual_scan (new PointCloudT);
    for(unsigned int n = 0; n < _cloud_tmp->points.size(); ++n)
    {
        float x = _cloud_tmp->points[n].x;
        float y = _cloud_tmp->points[n].y;
        if( x*x + y*y  > ROBOT_RADIUS* ROBOT_RADIUS)
        {
            virtual_scan->points.push_back(_cloud_tmp->points[n]);
        }
    }

    virtual_scan->width = virtual_scan->points.size();
    virtual_scan->height = 1;
    virtual_scan->is_dense = true;
    virtual_scan->header = cloud_obj->header;

    ROS_INFO("Virtual scan : %d", virtual_scan->points.size());
    sensor_msgs::PointCloud2 cloud_out;
    pcl::toROSMsg(*virtual_scan,cloud_out);
    cloud_out.header.stamp = ros::Time::now();
    virtual_scan_pub.publish(cloud_out);
    
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "kinect_processing");
    ros::NodeHandle n;
    ros::Subscriber cloub_sub = n.subscribe("/camera/depth_registered/points", 1, cloudCallback);
    ros::Subscriber obstacle = n.subscribe("/cloud_tf", 1, processObstacle);
    cloud_tf = n.advertise<sensor_msgs::PointCloud2>("cloud_tf",1);
    virtual_scan_pub = n.advertise<sensor_msgs::PointCloud2>("virtual_scan",1);
    listener = new tf::TransformListener();

    ros::spin();

    return 0;
}
