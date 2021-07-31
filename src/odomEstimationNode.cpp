// Author of FLOAM: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro

// Author of Lilo: Edison Velasco 
// Email evs25@alu.ua.es

//c++ lib
#include <cmath>
#include <vector>
#include <mutex>
#include <queue>
#include <thread>
#include <chrono>

//ros lib
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

//pcl lib
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

//local lib
#include "lidar.h"
#include "odomEstimationClass.h"

OdomEstimationClass odomEstimation;
std::mutex mutex_lock;
std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudEdgeBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudSurfBuf;
lidar::Lidar lidar_param;

ros::Publisher pubLaserOdometry;

ros::Publisher time_average;


void velodyneSurfHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
    mutex_lock.lock();
    pointCloudSurfBuf.push(laserCloudMsg);
    mutex_lock.unlock();
}
void velodyneEdgeHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
    mutex_lock.lock();
    pointCloudEdgeBuf.push(laserCloudMsg);
    mutex_lock.unlock();
}

bool is_odom_inited = false;
double total_time =0, edge_limit, surf_limit;
int total_frame=0;
bool clear_map;
void odom_estimation(){

    // previous odometry
    double x_orient_prev = 0;
    double y_orient_prev = 0;
    double z_orient_prev = 0;

    double x_pos_prev = 0;
    double y_pos_prev = 0;
    double z_pos_prev = 0;
    float time_delay  = 0;


    while(1){
        if(!pointCloudEdgeBuf.empty() && !pointCloudSurfBuf.empty()){

            mutex_lock.lock();

            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_surf_in(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_edge_in(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::fromROSMsg(*pointCloudEdgeBuf.front(), *pointcloud_edge_in);
            pcl::fromROSMsg(*pointCloudSurfBuf.front(), *pointcloud_surf_in);
            ros::Time pointcloud_time = (pointCloudSurfBuf.front())->header.stamp;
            pointCloudEdgeBuf.pop();
            pointCloudSurfBuf.pop();
            mutex_lock.unlock();

            if(is_odom_inited == false){
                odomEstimation.initMapWithPoints(pointcloud_edge_in, pointcloud_surf_in);
                is_odom_inited = true;
                ROS_INFO("odom inited");
            }else{
                std::chrono::time_point<std::chrono::system_clock> start, end;
                start = std::chrono::system_clock::now();
                odomEstimation.updatePointsToMap(pointcloud_edge_in, pointcloud_surf_in, clear_map, edge_limit , surf_limit);
                end = std::chrono::system_clock::now();
                std::chrono::duration<float> elapsed_seconds = end - start;
                total_frame++;
                float time_temp = elapsed_seconds.count() * 1000.0;
                total_time+=time_temp;
                time_delay = total_time/total_frame;
                ROS_INFO("average odom estimation time %f ms \n \n", time_delay);
                time_delay = time_delay/1000.0;
            }




            Eigen::Quaterniond q_current(odomEstimation.odom.rotation());
            Eigen::Vector3d t_current = odomEstimation.odom.translation();

            static tf::TransformBroadcaster br;
            tf::Transform transform;
            transform.setOrigin( tf::Vector3(t_current.x(), t_current.y(), t_current.z()) );
            tf::Quaternion q(q_current.x(),q_current.y(),q_current.z(),q_current.w());
            transform.setRotation(q);
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom_lidar", "base_link"));


            // Velocity
            double u_x = (t_current.x() - x_pos_prev)/time_delay;
            double u_y = (t_current.y() - y_pos_prev)/time_delay;
            double u_z = (t_current.z() - z_pos_prev)/time_delay;

            double w_x = (q_current.x() - x_orient_prev)/time_delay;
            double w_y = (q_current.y() - y_orient_prev)/time_delay;
            double w_z = (q_current.z() - z_orient_prev)/time_delay;

            // previous odometry

            x_pos_prev = t_current.x();
            y_pos_prev = t_current.y();
            z_pos_prev = t_current.z();

            x_orient_prev = q_current.x() ;
            y_orient_prev = q_current.y() ;
            z_orient_prev = q_current.z() ;


            // publish odometry
            nav_msgs::Odometry laserOdometry;
            laserOdometry.header.frame_id = "odom_lidar";
            laserOdometry.child_frame_id = "base_link";
            laserOdometry.header.stamp = pointcloud_time;
            laserOdometry.pose.pose.orientation.x = q_current.x();
            laserOdometry.pose.pose.orientation.y = q_current.y();
            laserOdometry.pose.pose.orientation.z = q_current.z();
            laserOdometry.pose.pose.orientation.w = q_current.w();
            laserOdometry.pose.pose.position.x = t_current.x();
            laserOdometry.pose.pose.position.y = t_current.y();
            laserOdometry.pose.pose.position.z = t_current.z();
            laserOdometry.twist.twist.linear.x = u_x;
            laserOdometry.twist.twist.linear.y = u_y;
            laserOdometry.twist.twist.linear.z = u_z;
            laserOdometry.twist.twist.angular.x = w_x;
            laserOdometry.twist.twist.angular.y = w_y;
            laserOdometry.twist.twist.angular.z = w_z;

            for(int i = 0; i<36; i++) {
              if(i == 0 || i == 7 || i == 14) {
                laserOdometry.pose.covariance[i] = .01;
               }
               else if (i == 21 || i == 28 || i== 35) {
                 laserOdometry.pose.covariance[i] += 0.1;
               }
               else {
                 laserOdometry.pose.covariance[i] = 0;
               }
            }

            pubLaserOdometry.publish(laserOdometry);

            //publish time

            std_msgs::Float64 time_msg;
            time_msg.data = time_delay*1000.0;
            time_average.publish(time_msg);

         }

        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "main");
    ros::NodeHandle nh;

    int scan_line = 16;
    double vertical_angle = 0.0;
    double scan_period= 0.1;
    double max_dis = 60.0;
    double min_dis =0;
    double edge_resolution = 0.3;
    double surf_resolution = 0.6;
    bool validation_angle = false;

    clear_map = true;
    edge_limit = 10000;
    surf_limit = 10000;

    nh.getParam("/scan_period", scan_period); 
    nh.getParam("/vertical_angle", vertical_angle); 
    nh.getParam("/max_dis", max_dis);
    nh.getParam("/min_dis", min_dis);
    nh.getParam("/scan_line", scan_line);
    nh.getParam("/edge_resolution", edge_resolution);
    nh.getParam("/surf_resolution", surf_resolution);
    nh.getParam("/clear_map", clear_map);

    nh.getParam("/edge_limit", edge_limit);
    nh.getParam("/surf_limit", surf_limit);

    lidar_param.setScanPeriod(scan_period);
    lidar_param.setValidationAngle(validation_angle);
    lidar_param.setVerticalAngle(vertical_angle);
    lidar_param.setLines(scan_line);
    lidar_param.setMaxDistance(max_dis);
    lidar_param.setMinDistance(min_dis);

    odomEstimation.init(lidar_param, edge_resolution, surf_resolution);
    ros::Subscriber subEdgeLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_edge", 100, velodyneEdgeHandler);
    ros::Subscriber subSurfLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_surf", 100, velodyneSurfHandler);

    pubLaserOdometry = nh.advertise<nav_msgs::Odometry>("/odom", 100);
    time_average = nh.advertise<std_msgs::Float64>("/time_average", 100);


    std::thread odom_estimation_process{odom_estimation};

    ros::spin();

    return 0;
}

