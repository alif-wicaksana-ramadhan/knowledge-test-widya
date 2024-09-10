#include <iostream>
#include <thread>
#include <chrono>
#include <cmath>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <Eigen/Dense>
#include "pcd_reader/Rotate.h"
#include "pcd_reader/Save.h"

using namespace std;

class PointCloudReader
{
private:
    float theta = 0;
    ros::NodeHandle nh;
    ros::Publisher pub;
    ros::ServiceServer save_ser;
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;

public:
    PointCloudReader();
    ~PointCloudReader();
    void run();
    void main_loop();
    void user_loop();
    void rotate(double theta);
    void fromPCD(string filename);
    void publishTransformedPCD();
    double degreesToRadians(double degrees);
    bool save(pcd_reader::Save::Request &req, pcd_reader::Save::Response &res);
};

PointCloudReader::PointCloudReader()
{
    pub = nh.advertise<sensor_msgs::PointCloud2>("/transformed_cloud", 1);
    save_ser = nh.advertiseService("save", &PointCloudReader::save, this);

    transformed_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
}

PointCloudReader::~PointCloudReader()
{
}

void PointCloudReader::run()
{
    thread thread1(&PointCloudReader::main_loop, this);
    thread thread2(&PointCloudReader::user_loop, this);

    thread1.join();
    thread2.join();
}

void PointCloudReader::main_loop()
{
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        publishTransformedPCD();
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void PointCloudReader::user_loop()
{
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        float theta;

        cout << "Enter a theta value: ";
        cin >> theta;

        if (cin.fail())
        {
            cout << "Invalid theta. Please enter a float value." << endl;
            cin.clear();
            cin.ignore(numeric_limits<streamsize>::max(), '\n');
        }
        else
        {
            cout << "Rotating object with theta of: " << theta << endl;
            this->rotate(theta);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
}

void PointCloudReader::rotate(double theta)
{
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

    double thetaRad = degreesToRadians(theta);
    transform(0, 0) = cos(thetaRad);
    transform(0, 1) = -sin(thetaRad);
    transform(1, 0) = sin(thetaRad);
    transform(1, 1) = cos(thetaRad);
    pcl::transformPointCloud(*cloud, *transformed_cloud, transform);
}

bool PointCloudReader::save(pcd_reader::Save::Request &req, pcd_reader::Save::Response &res)
{
    string savepath = req.savepath.data;
    ROS_INFO("Saving the point cloud data of the object to: %s", savepath.c_str());

    pcl::io::savePCDFileASCII(savepath, *transformed_cloud);

    return true;
}

void PointCloudReader::fromPCD(string filename)
{

    if (pcl::io::loadPCDFile<pcl::PointXYZ>(filename, *cloud) == -1) //* load the file
    {
        PCL_ERROR("Couldn't read file test_pcd.pcd \n");
    }

    transformed_cloud = cloud;
}

void PointCloudReader::publishTransformedPCD()
{
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*transformed_cloud, output);
    output.header.frame_id = "map";

    pub.publish(output);
}

double PointCloudReader::degreesToRadians(double degrees)
{
    return degrees * (M_PI / 180.0);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pointcloud_reader");

    if (argc < 2)
    {
        cerr << "Please provide a rotation angle in degrees!" << endl;
        return -1;
    }

    string filepath = argv[1];

    PointCloudReader pcd_reader = PointCloudReader();
    pcd_reader.fromPCD(filepath);
    pcd_reader.run();

    return (0);
}