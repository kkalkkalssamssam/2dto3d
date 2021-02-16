#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

ros::Publisher pub;

typedef pcl::PointXYZ PointT;

void cloud_cb(const sensor_msgs::PointCloud& msg)
{
    sensor_msgs::PointCloud2 pc2_cloud;

    sensor_msgs::convertPointCloudToPointCloud2(msg, pc2_cloud);

    // convert point data type (PointCloud->)
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(pc2_cloud, cloud);

    // filtering
    pcl::VoxelGrid<pcl::PointXYZ>vg;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    vg.setInputCloud(cloud.makeShared());
    vg.setLeafSize(0.01f, 0.01f, 0.01f);
    vg.filter(*cloud_filtered);
    
    // create KdTree object
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud_filtered);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.0198333333333333333333333333333333333333333333333333333 ); // 19
    ec.setMinClusterSize(550);
    ec.setMaxClusterSize(10000);

    // ec.setClusterTolerance(0.04); // 2cm
    // ec.setMinClusterSize(850);
    // ec.setMaxClusterSize(10000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_filtered);
    ec.extract(cluster_indices);

    pcl::PointCloud<pcl::PointXYZI>TotalCloud;
    int j = 0;
     for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        {
            pcl::PointXYZ pt = cloud_filtered->points[*pit];
                pcl::PointXYZI pt2;
                pt2.x = pt.x, pt2.y = pt.y, pt2.z = pt.z;
                pt2.intensity = (float)(j + 1);

                TotalCloud.push_back(pt2);
        }
        j++;
    }
    ROS_INFO("%d", j);
    // Convert to ros data type
    pcl::PCLPointCloud2 cloud_p;
    pcl::toPCLPointCloud2(TotalCloud, cloud_p);

    sensor_msgs::PointCloud2 output;
    pcl_conversions::fromPCL(cloud_p, output);
    output.header.frame_id = "pt_cloud2";
    pub.publish(output);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "my_pcl");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("pcl_total", 1, cloud_cb);

    pub = nh.advertise<sensor_msgs::PointCloud2>("pcl_output", 1);

    ros::spin();
}

