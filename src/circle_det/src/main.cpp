#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <geometry_msgs/PoseStamped.h>
#include "circle_det/circles.h"
#include <deque>
#include "std_msgs/Int8.h"

ros::Publisher pub, point_all_pub;
std::deque<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_all_points;
circle_det::circles circles_msg;
std_msgs::Int8 state;

//静态圆范围
double xmax1, ymax1, zmax1, xmin1, ymin1, zmin1;
//动态圆范围
double xmax2, ymax2, zmax2, xmin2, ymin2, zmin2;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{  
    if (state.data==3||state.data==5)
    {
        double xmax, ymax, zmax, xmin, ymin, zmin;
        if(state.data == 3)
        {
            xmax=xmax1; ymax=ymax1; zmax=zmax1;
            xmin=xmin1; ymin=ymin1; zmin=zmin1;
        }
        else if(state.data == 5)
        {
            xmax=xmax2; ymax=ymax2; zmax=zmax2;
            xmin=xmin2; ymin=ymin2; zmin=zmin2;       
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_single_points(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*cloud_msg, *cloud_single_points);

        // 在指定范围内过滤点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto& point : cloud_single_points->points)
        {
            if (point.x >= xmin && point.x <= xmax &&
                point.y >= ymin && point.y <= ymax &&
                point.z >= zmin && point.z <= zmax)
            {
                filtered_cloud->points.push_back(point);
            }
        }
        filtered_cloud->width = filtered_cloud->points.size();
        filtered_cloud->height = 1;
        filtered_cloud->is_dense = true;

        // 将过滤后的点云添加到双端队列中
        cloud_all_points.push_back(filtered_cloud);

        // 维持双端队列的大小，保留最后的 15 帧点云
        if (cloud_all_points.size() > 15)
        {
            cloud_all_points.pop_front(); // 移除最旧的点云
        }

        // 处理最新的累积点云数据
        pcl::PointCloud<pcl::PointXYZ>::Ptr accumulated_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto& cloud : cloud_all_points)
        {
            *accumulated_cloud += *cloud;
        }

        // 发布点云
        sensor_msgs::PointCloud2 cloud_filtered_msg;
        pcl::toROSMsg(*accumulated_cloud, cloud_filtered_msg);
        cloud_filtered_msg.header.frame_id = "camera_init";
        point_all_pub.publish(cloud_filtered_msg);

        // 计算圆心并发布
        if (cloud_all_points.size() == 15)
        {
            geometry_msgs::Point circle_center;
            for (const auto& point : accumulated_cloud->points)
            {
                circle_center.x += point.x;
                circle_center.y += point.y;
                circle_center.z += point.z;
            }

            circle_center.x /= accumulated_cloud->points.size();
            circle_center.y /= accumulated_cloud->points.size();
            circle_center.z /= accumulated_cloud->points.size();

            circles_msg.pos.push_back(circle_center);
            pub.publish(circles_msg);
            std::cout<<circles_msg<<state.data<<std::endl;
            circles_msg.pos.clear();
        }
    }
}

void stateMachineCallback(const std_msgs::Int8::ConstPtr& msg)
{
    state = *msg;
    //ROS_INFO("Received state machine state: [%d]", state.data);  
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "circle_det_node");
    ros::NodeHandle nh;

    // 静态圆范围区域
    nh.param<double>("/circle_det_node/xmax1", xmax1, 0.0);
    nh.param<double>("/circle_det_node/ymax1", ymax1, 0.0);
    nh.param<double>("/circle_det_node/zmax1", zmax1, 0.0);
    nh.param<double>("/circle_det_node/xmin1", xmin1, 0.0);
    nh.param<double>("/circle_det_node/ymin1", ymin1, 0.0);
    nh.param<double>("/circle_det_node/zmin1", zmin1, 0.0);
    ROS_INFO("xmax1: %.2f, ymax1: %.2f, zmax1: %.2f", xmax1, ymax1, zmax1);
    ROS_INFO("xmin1: %.2f, ymin1: %.2f, zmin1: %.2f", xmin1, ymin1, zmin1);

    // 动态圆范围区域
    nh.param<double>("/circle_det_node/xmax2", xmax2, 0.0);
    nh.param<double>("/circle_det_node/ymax2", ymax2, 0.0);
    nh.param<double>("/circle_det_node/zmax2", zmax2, 0.0);
    nh.param<double>("/circle_det_node/xmin2", xmin2, 0.0);
    nh.param<double>("/circle_det_node/ymin2", ymin2, 0.0);
    nh.param<double>("/circle_det_node/zmin2", zmin2, 0.0);
    ROS_INFO("xmax2: %.2f, ymax2: %.2f, zmax2: %.2f", xmax2, ymax2, zmax2);
    ROS_INFO("xmin2: %.2f, ymin2: %.2f, zmin2: %.2f", xmin2, ymin2, zmin2);

    pub = nh.advertise<circle_det::circles>("/circle", 1);
    point_all_pub = nh.advertise<sensor_msgs::PointCloud2>("/all_points", 1);

    ros::Subscriber cloub_sub = nh.subscribe("/cloud_registered", 1, cloud_cb);
    ros::Subscriber state_sub = nh.subscribe<std_msgs::Int8>("/state_machine", 10, stateMachineCallback);

    ros::spin();
    return 0;
}
