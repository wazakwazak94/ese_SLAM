#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sstream>
#include <math.h>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#define PI 3.14159265

typedef pcl::PointXYZI PointI;
typedef pcl::PointCloud<PointI> Pointcloud;
typedef sensor_msgs::PointCloud2 msgs_Point;
typedef sensor_msgs::Imu msgs_Imu;
typedef Eigen::Vector3f Vector3;
typedef Eigen::Matrix3f Matrix3;
typedef Eigen::Matrix4f Matrix4;

Vector3 original_accel, original_ang_vel;
Vector3 current_accel, current_ang_vel;         // 0->x 1->y 2->z 
Vector3 past_accel, last_ang_vel;
Vector3 Position, last_Position, Velocity; 

Pointcloud::Ptr IMU_odom (new Pointcloud);

double time_interval = 0;

Matrix3 Roll, Pitch, Yaw, IMU_rotation_matrix;

ros::Time imu_current_time, imu_last_time;

void TimeUpdate (void)
{
    imu_last_time = imu_current_time;
    imu_current_time = ros::Time::now();
    time_interval = (imu_current_time - imu_last_time).toSec();
}

void initRotationMatrix  
(const float imu_roll, const float imu_pitch, const float imu_yaw)
{
    float roll_  = imu_roll  * PI/180;
    float pitch_ = imu_pitch * PI/180;
    float yaw_   = imu_yaw   * PI/180;

    Roll  << 1,0,0,
             0,cos(roll_),-1*sin(roll_),
             0,sin(roll_),cos(roll_);

    Pitch << cos(pitch_),0,sin(pitch_),
             0,1,0,
             -1*sin(pitch_),0,cos(pitch_);

    Yaw   << cos(yaw_),-1*sin(yaw_),0,
             sin(yaw_),cos(yaw_),0,
             0,0,1; 

    IMU_rotation_matrix = Roll * Pitch * Yaw;

    return ;
}

void IMUrotation (void)
{
    current_accel = IMU_rotation_matrix * original_accel;
    current_ang_vel = IMU_rotation_matrix * original_ang_vel;
}

void estimateOdomFromImu (void)
{
    Position += pow(time_interval,2)*current_accel;
    Velocity += time_interval*current_accel;

    Pointcloud::Ptr tempPose (new Pointcloud);
    tempPose->points.resize(1);
    tempPose->points[0].x = Position(0);
    tempPose->points[0].y = Position(1);
    tempPose->points[0].z = 0;
    tempPose->points[0].intensity = 0;

   std::cout << "Position x :" << Velocity(0) << "\n"
             << "Position y :" << Velocity(1) << "\n"
             << "Position z :" << Velocity(2) << "\n" 
             << "time interval :" << time_interval << std::endl;

    *IMU_odom += *tempPose;
}