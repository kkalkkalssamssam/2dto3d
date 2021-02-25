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

class ChangeFinder
{
public:
	ChangeFinder();

    void InitROS(void);
    void PclCallBack(const sensor_msgs::PointCloud::ConstPtr &msg);
    void RangeCallBack(const std_msgs::Float64MultiArray::ConstPtr & msg);
    void PubPointCloud(double theta, int idx);
    void msgCallback(const std_msgs::Int16::ConstPtr & msg);
    geometry_msgs::Point32 CalPoint(double r, double theta, double yaw);
    float CalDetectColor(geometry_msgs::Point32 p);
    double CompareDistance(double d, double d_orig);
    double GetDistance(geometry_msgs::Point32 p);
    bool isSaved(void);

private:
    // ros
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv{"~"};

    ros::Publisher data_pub;
    ros::Subscriber pcl_sub;
    ros::Subscriber range_sub;
    ros::Subscriber theta_sub;
    ros::Publisher mode_pub;

    sensor_msgs::PointCloud scan_msg;
    sensor_msgs::PointCloud change_msg;
    std::vector<double> range_orig;

    // Launch variables
    std::string serial_port_name = std::string("/dev/ttyUSB0");
    std::string frame_id = std::string("pcl_data");
    std::string pub_topicname_lidar = std::string("pcl_rviz");
    std::string sub_topicname_lidar = std::string("theta_msg");
    int min_r = 512;
    int max_r = 827;
    double height = 0.;
    
    bool pcl_saved = false;
    bool range_saved = false;

    // Local variables
    Gl* gl;

    int max_idx = 607;
    int detect_cnt = 0;
    int detect_idx;
};

ChangeFinder::ChangeFinder()
{
    // ros init
    InitROS();

    // GL Init
    gl = new Gl(serial_port_name, 921600);
    std::cout << "Serial Num : " << gl->GetSerialNum() << std::endl;
    gl->SetFrameDataEnable(true);
}

void ChangeFinder::InitROS(void)
{
    nh_priv.param("serial_port_name", serial_port_name, serial_port_name);
    nh_priv.param("frame_id", frame_id, frame_id);
    nh_priv.param("pub_topicname_lidar", pub_topicname_lidar, pub_topicname_lidar);
    nh_priv.param("sub_topicname_lidar", sub_topicname_lidar, sub_topicname_lidar);
    nh_priv.param("min_r", min_r, min_r);
    nh_priv.param("max_r", max_r, max_r);
    nh_priv.param("height", height, height);

    
    data_pub = nh.advertise<sensor_msgs::PointCloud>(pub_topicname_lidar, 10);
    pcl_sub = nh.subscribe<sensor_msgs::PointCloud>("pcl_total", 10, &ChangeFinder::PclCallBack, this);
    range_sub = nh.subscribe<std_msgs::Float64MultiArray>("lidar_range", 10, &ChangeFinder::RangeCallBack, this);
    theta_sub = nh.subscribe<std_msgs::Int16>(sub_topicname_lidar, 10, &ChangeFinder::msgCallback, this);        
    mode_pub = nh.advertise<std_msgs::Int16MultiArray>("mode_msg", 10);

    detect_idx = max_r - min_r + 1;
}

void ChangeFinder::PclCallBack(const sensor_msgs::PointCloud::ConstPtr &msg)
{
    scan_msg = *msg;
    change_msg = scan_msg;
    max_idx = scan_msg.points.size() / 1000;
    pcl_saved = true;
    ROS_INFO("pcl save success");
}

void ChangeFinder::RangeCallBack(const std_msgs::Float64MultiArray::ConstPtr &msg)
{
    range_orig = msg->data;
    range_saved = true;
    ROS_INFO("range save success");
}

bool ChangeFinder::isSaved(void)
{
    if (pcl_saved && range_saved) return true;
    else return false;
}

void ChangeFinder::msgCallback(const std_msgs::Int16::ConstPtr & msg)
{
    if (isSaved()){
        if (msg->data == WAIT) {
            std_msgs::Int16MultiArray start_msg;
            start_msg.data.push_back(SCAN);
            start_msg.data.push_back(min_r);
            start_msg.data.push_back(max_r);
            mode_pub.publish(start_msg);
        } else {
            double theta = ((msg->data - 512) * 0.29) * M_PI / 180 + M_PI_2;
            int idx = msg->data - 512;
            PubPointCloud(theta, idx);
        }
    }
}

double ChangeFinder::GetDistance(geometry_msgs::Point32 p)
{
    double d = sqrt((p.x * p.x) + (p.y * p.y) + (p.z * p.z));

    return d;
}


geometry_msgs::Point32 ChangeFinder::CalPoint(double r, double theta, double yaw)
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


float ChangeFinder::CalDetectColor(geometry_msgs::Point32 p)
{
    double d = GetDistance(p);
    double cm = d * 100;
    int ratio = (int)(cm * 255 / 1000);
    if (ratio > 255) ratio = 255;
    if (ratio < 0) ratio = 0;
    unsigned int rgb = 0x00ff00 - (ratio * 256);
        
    float float_rgb = *reinterpret_cast<float*>(&rgb);

    return float_rgb;
}


double ChangeFinder::CompareDistance(double d, double d_orig)
{
    double res = d - d_orig;

    if (abs(res) < 0.1) res = 0;

    return res;
}

void ChangeFinder::PubPointCloud(double theta, int idx)
{
    if (detect_cnt == 0) ROS_INFO("Detect Start");

    Gl::framedata_t frame_data;
    gl->ReadFrameData(frame_data);

    int num_data = frame_data.distance.size();
    if(num_data>0)
    {
        change_msg.header.stamp = ros::Time::now();

        if (detect_cnt < detect_idx){
            for (int i = 0; i < num_data; i++)
            {
                double yaw = frame_data.angle[i];
                double r = frame_data.distance[i];
                if (r > 10.0) r = 10.0;
                
                double d_orig = range_orig[idx * 1000 + i];
                double res = CompareDistance(r, d_orig);

                if (res < 0) {
                    geometry_msgs::Point32 p = CalPoint(r, theta, yaw);
                    float float_rgb = CalDetectColor(p);
                    change_msg.points.push_back(p);
                    change_msg.channels[0].values.push_back(float_rgb);
                } else if (res > 0) {
                    geometry_msgs::Point32 p = CalPoint(r, theta, yaw);
                    float float_rgb = CalDetectColor(p);
                    change_msg.points.push_back(p);
                    change_msg.channels[0].values.push_back(float_rgb);
                    change_msg.points[idx * 1000 + i] = p;
                    change_msg.channels[0].values[idx * 1000 + i] = float_rgb;
                } else {
                    change_msg.points.push_back(scan_msg.points[idx * 1000 + i]);
                    change_msg.channels[0].values.push_back(scan_msg.channels[0].values[idx * 1000 + i]);
                }
            }
            detect_cnt++;
        } else {
            theta_sub.shutdown();
            std_msgs::Int16MultiArray stop_msg;
            stop_msg.data.push_back(STOP);
            mode_pub.publish(stop_msg);
            ROS_INFO("Detect Finish");
        }
    } 
    data_pub.publish(change_msg);
}

int main(int argc,char** argv)
{
    ros::init(argc, argv, "change_finder");

    ChangeFinder cf;
    
    ros::spin();

    return 0;
}
