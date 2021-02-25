#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int16MultiArray.h>
#include <sensor_msgs/PointCloud.h>
#include <cmath>
#include "gl_driver.h"

#define STOP -1
#define WAIT 0
#define SCAN 1

class Scanner
{
public:
	Scanner();

    void InitROS(void);
    
    void PubPointCloud(double theta);
    void msgCallback(const std_msgs::Int16::ConstPtr & msg);
    geometry_msgs::Point32 CalPoint(double r, double theta, double yaw);
    float CalColor(geometry_msgs::Point32 p);
    double GetDistance(geometry_msgs::Point32 p);

private:
    // ros
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv{"~"};

    ros::Publisher data_pub;
    ros::Publisher pcl_pub;
    ros::Publisher range_pub;
    ros::Subscriber data_sub;
    ros::Publisher mode_pub;

    sensor_msgs::PointCloud scan_msg;
    sensor_msgs::ChannelFloat32 channels;
    std_msgs::Float64MultiArray range;
    std_msgs::Int16MultiArray mode_msg;

    // Launch variables
    std::string serial_port_name = std::string("/dev/ttyUSB0");
    std::string frame_id = std::string("pcl_data");
    std::string pub_topicname_lidar = std::string("pcl_data");
    std::string sub_topicname_lidar = std::string("theta_msg");
    int min_r = 512;
    int max_r = 827;
    double height = 0.;

    Gl* gl;

    // Local variables
    int cnt = 0;
    int max_idx = max_r - min_r;
};

Scanner::Scanner()
{
    // ros init
    InitROS();

    // GL Init
    gl = new Gl(serial_port_name, 921600);
    std::cout << "Serial Num : " << gl->GetSerialNum() << std::endl;
    gl->SetFrameDataEnable(true);
}

void Scanner::InitROS(void)
{
    nh_priv.param("serial_port_name", serial_port_name, serial_port_name);
    nh_priv.param("frame_id", frame_id, frame_id);
    nh_priv.param("pub_topicname_lidar", pub_topicname_lidar, pub_topicname_lidar);
    nh_priv.param("sub_topicname_lidar", sub_topicname_lidar, sub_topicname_lidar);
    nh_priv.param("min_r", min_r, min_r);
    nh_priv.param("max_r", max_r, max_r);
    nh_priv.param("height", height, height);


    data_pub = nh.advertise<sensor_msgs::PointCloud>(pub_topicname_lidar, 10);
    data_sub = nh.subscribe<std_msgs::Int16>(sub_topicname_lidar, 10, &Scanner::msgCallback, this);
    pcl_pub = nh.advertise<sensor_msgs::PointCloud>("pcl_total", 10);
    range_pub = nh.advertise<std_msgs::Float64MultiArray>("lidar_range", 10);
    mode_pub = nh.advertise<std_msgs::Int16MultiArray>("mode_msg", 10);

    scan_msg.header.frame_id = frame_id;
    channels.name = "rgb";
    max_idx = max_r - min_r;
}

void Scanner::msgCallback(const std_msgs::Int16::ConstPtr & msg)
{
    if (msg->data == WAIT) {
        std_msgs::Int16MultiArray start_msg;
        start_msg.data.push_back(SCAN);
        start_msg.data.push_back(min_r);
        start_msg.data.push_back(max_r);
        mode_pub.publish(start_msg);
    } else {
        double theta = ((msg->data - 512) * 0.29) * M_PI / 180 + M_PI_2;
        PubPointCloud(theta);
    }
}

double Scanner::GetDistance(geometry_msgs::Point32 p)
{
    double d = sqrt((p.x * p.x) + (p.y * p.y) + (p.z * p.z));

    return d;
}


geometry_msgs::Point32 Scanner::CalPoint(double r, double theta, double yaw)
{
    geometry_msgs::Point32 p;
    double x = r * cos(yaw);
    double y = r * sin(yaw) * sin(theta) + 0.135 * sin(theta) + 0.055 * cos(theta);
    double z = r * sin(yaw) * cos(theta) + 0.135 * cos(theta) - 0.055 * sin(theta) + height;

    p.x = x;
    p.y = y;
    p.z = z;

    return p;
}

float Scanner::CalColor(geometry_msgs::Point32 p)
{
    double d = GetDistance(p);
    double cm = d * 100;
    int ratio = (int)(cm * 511 / 1000);
    if (ratio > 511) ratio = 511;
    if (ratio < 0) ratio = 0;
    unsigned int rgb;
    if (ratio <= 255) {
        rgb = 0xff0000 + ((256 - ratio) * 256);
    } else {
        rgb = 0xff0000 - ((ratio - 256) * 65536);
    }
    
    float float_rgb = *reinterpret_cast<float*>(&rgb);

    return float_rgb;
}

void Scanner::PubPointCloud(double theta)
{
    if (cnt == 0) ROS_INFO("Scan Start");

    Gl::framedata_t frame_data;
    gl->ReadFrameData(frame_data);

    int num_data = frame_data.distance.size();
    if(num_data>0)
    {
        scan_msg.header.stamp = ros::Time::now();

        for (int i = 0; i < num_data; i++)
        {
            double yaw = frame_data.angle[i];
            double r = frame_data.distance[i];
            if (r > 25.0) r = 25.0;

            geometry_msgs::Point32 p = CalPoint(r, theta, yaw);
            float float_rgb = CalColor(p);

            scan_msg.points.push_back(p);
            range.data.push_back(r);
            channels.values.push_back(float_rgb);
        }
        scan_msg.channels.clear();
        scan_msg.channels.push_back(channels);
        data_pub.publish(scan_msg);
    }
    cnt++;
    if (cnt == max_idx){
        data_sub.shutdown();
        std_msgs::Int16MultiArray stop_msg;
        stop_msg.data.push_back(STOP);
        mode_pub.publish(stop_msg);
        pcl_pub.publish(scan_msg);
        range_pub.publish(range);
        ROS_INFO("Scan Finish");
    }
}

int main(int argc,char** argv)
{
    ros::init(argc, argv, "lidar_scan");

    Scanner scanner;

    ros::spin();

    return 0;
}
