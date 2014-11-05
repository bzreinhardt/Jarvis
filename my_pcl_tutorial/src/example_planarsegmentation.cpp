#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/region_growing_rgb.h>
//Include these for visualization and filtering ben 11/4
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/transforms.h>
//TODO: Filter by distance
//Add visualizer from PCL?
//
ros::Publisher pub;



  pcl::visualization::CloudViewer viewer("cloud viewer");
void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
    // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    //pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg (*input, *cloud);
    //make new cloud to operate on
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr transformed(new pcl::PointCloud<pcl::PointXYZRGBA>);
    //rotate 180
    Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
    //TODO: make the rotation interactive
    //  // Set a rotation around the Z axis.
    float theta = M_PI; // 180 degrees.
    transformation(0, 0) = cos(theta);
    transformation(0, 1) = -sin(theta);
    transformation(1, 0) = sin(theta);
    transformation(1, 1) = cos(theta);
    //
    pcl::transformPointCloud(*cloud, *transformed, transformation);
    pcl::IndicesPtr indices (new std::vector <int>);
    // Remove anything more than 1.5 meters away
    pcl::PassThrough<pcl::PointXYZRGBA> filter;
    filter.setInputCloud(transformed);
    filter.setFilterFieldName("z");
    filter.setFilterLimits(0.0, 1.5);
    filter.filter(*transformed);
    filter.filter(*indices);
    
    
    //** find the coefficients of the planes **
    pcl::ModelCoefficients coefficients;
    pcl::PointIndices inliers;
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZRGBA> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);

    seg.setInputCloud ((*transformed).makeShared ());
    seg.segment (inliers, coefficients);

    //** segment the cloud by color **
    //create a search tree for XYZRGB points
    pcl::search::Search <pcl::PointXYZRGBA>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGBA> > (new pcl::search::KdTree<pcl::PointXYZRGBA>);
    pcl::RegionGrowingRGB<pcl::PointXYZRGBA> reg;
    reg.setInputCloud(transformed);
    reg.setIndices (indices);
    reg.setSearchMethod (tree);
    reg.setDistanceThreshold (10);
    reg.setPointColorThreshold (6);
    reg.setRegionColorThreshold (5);
    reg.setMinClusterSize (600);
    std::vector <pcl::PointIndices> clusters;
    reg.extract (clusters);
    
    //view the cloud wout rviz
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
    //viewer.addPointCloud(cloud, "input_cloud");
	//viewer.addCoordinateSystem(1.0,"world",0);
	if (!viewer.wasStopped())
	{
	    viewer.showCloud (colored_cloud);
	    //viewer.spinOnce(100);
	}
    // Publish the model coefficients
    pcl_msgs::ModelCoefficients ros_coefficients;
    pcl_conversions::fromPCL(coefficients, ros_coefficients);
    pub.publish (ros_coefficients);
}

    int
main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "my_pcl_tutorial");
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);

    // Create a ROS publisher for the output model coefficients
    pub = nh.advertise<pcl_msgs::ModelCoefficients> ("output", 1);

    // Spin
    ros::spin ();
    //viewer.spinOnce();
}
