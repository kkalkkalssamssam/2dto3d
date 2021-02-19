#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <std_msgs/Float64MultiArray.h>
#include <cmath>
#include "object_detection.h"


class DegreeFinder
{
public:
	DegreeFinder();

    void InitROS(void);
    void PclCallBack(const sensor_msgs::PointCloud::ConstPtr &msg);
    void RangeCallBack(const std_msgs::Float64MultiArray::ConstPtr & msg);
    void PubDegree(Gl::framedata_t frame_data);
    void Run(void);
    bool isSaved(void);

private:
    // ros
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv{"~"};

    ros::Publisher pcl_pub;
    ros::Subscriber pcl_sub;
    ros::Subscriber range_sub;

    sensor_msgs::PointCloud pcl_orig;
    std::vector<double> range_orig;

    // Launch variables
    std::string pub_topicname = std::string("pcl_rviz");
    std::string sub_topicname = std::string("pcl_total");

    float float_rgb;
    double height = 0.;

    bool pcl_saved = false;
    bool range_saved = false;

    ObjDetect obj_detect;
};

DegreeFinder::DegreeFinder()
{
    InitROS();
    obj_detect.InitObjDetect();
}

void DegreeFinder::InitROS(void)
{
    nh_priv.param("serial_port_name", obj_detect.serial_port_name, obj_detect.serial_port_name);
    nh_priv.param("pub_topicname", pub_topicname, pub_topicname);
    nh_priv.param("sub_topicname", sub_topicname, sub_topicname);
    
    pcl_pub = nh.advertise<sensor_msgs::PointCloud>(pub_topicname, 10);
    pcl_sub = nh.subscribe<sensor_msgs::PointCloud>(sub_topicname, 10, &DegreeFinder::PclCallBack, this);
    range_sub = nh.subscribe<std_msgs::Float64MultiArray>("lidar_range", 10, &DegreeFinder::RangeCallBack, this);


    unsigned int rgb = 0x00ff00;
    float_rgb = *reinterpret_cast<float*>(&rgb);
}



void DegreeFinder::PclCallBack(const sensor_msgs::PointCloud::ConstPtr &msg)
{
    pcl_orig = *msg;
    pcl_saved = true;
    ROS_INFO("pcl save success");
}

void DegreeFinder::RangeCallBack(const std_msgs::Float64MultiArray::ConstPtr & msg)
{
    range_orig = msg->data;
    range_saved = true;
    ROS_INFO("range save success");
}

bool DegreeFinder::isSaved(void)
{
    if (pcl_saved && range_saved) {
        return true;
    }
    else return false;
}

void DegreeFinder::Run(void)
{
    if (isSaved()) {
        obj_detect.Detection(false);
        Gl::framedata_t frame_data = obj_detect.GetRawData();
        PubDegree(frame_data);
    }
}

void DegreeFinder::PubDegree(Gl::framedata_t frame_data)
{
    int num_data = frame_data.distance.size();
    if(num_data>0)
    {
        sensor_msgs::PointCloud rviz_msg;
        rviz_msg.header.frame_id = "pcl_data";
        rviz_msg.header.stamp = ros::Time::now();
        rviz_msg.points = pcl_orig.points;
        rviz_msg.channels = pcl_orig.channels;

        int idx = 0;
        float min_sum = 100000.0;
        for (int i = 0; i < rviz_msg.points.size() / 1000; i++)
        {
            float sum = 0;
            for (int j = 0; j < 1000; j++)
            {
                sum += abs(frame_data.distance[j] - range_orig[i * 1000 + j]);
            }
            if (min_sum > sum) {
                min_sum = sum;
                idx = i;
            }
        }
        for (int j = 0; j < 1000; j++)
        {
            rviz_msg.channels[0].values[idx * 1000 + j] = float_rgb;
        }
        pcl_pub.publish(rviz_msg);
    }
}

int main(int argc,char** argv)
{
    ros::init(argc, argv, "degree_finder");

    DegreeFinder df;

    ros::Rate loop_rate(20);

    while (ros::ok())
    {
        df.Run();

        ros::spinOnce();
        loop_rate.sleep();    
    }

    return 0;
}
