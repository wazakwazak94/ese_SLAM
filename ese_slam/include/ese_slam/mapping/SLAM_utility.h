#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sstream>
#include <memory.h>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <math.h>

#include <Eigen/Eigen>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/ndt.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>

#include <iostream>
#include <pcl/io/pcd_io.h>

#include <ese_icp/LocalMap.h>

#include <gtsam/geometry/Rot3.h>


#define PI 3.14159265

using namespace std;

//convenient typedef
typedef pcl::PointXYZI PointI;
typedef pcl::PointCloud<PointI> Pointcloud;
typedef pcl::FPFHSignature33 FPFHS;
typedef pcl::PointCloud<FPFHS> FPFHS33;
typedef pcl::PointNormal Normal;
typedef pcl::PointCloud<Normal> PointNormal;
typedef sensor_msgs::PointCloud2 msgs_Point;
typedef Eigen::Matrix4f Matrix4;
typedef Eigen::Matrix3f Matrix3;
typedef Eigen::Vector3f Vector3;

typedef struct LocalMap
{
    uint16_t map_index;
    Pointcloud::Ptr map_data;
    float x, y, z, roll, pitch, yaw;
    struct Localmap *prev, *next;
}LocalMap;

LocalMap *head, *tail;

Pointcloud::Ptr current_localmap (new Pointcloud);
Pointcloud::Ptr next_localmap (new Pointcloud);

int current_index = 0;
int next_index = 0;
int numberOfLocalmap = 0;

Pointcloud::Ptr original_map_data (new Pointcloud);
Pointcloud::Ptr original_based_data (new Pointcloud);
Pointcloud::Ptr original_scan_data (new Pointcloud);

Pointcloud::Ptr voxeled_map_data (new Pointcloud);
Pointcloud::Ptr voxeled_based_data (new Pointcloud);
Pointcloud::Ptr voxeled_scan_data (new Pointcloud);

Pointcloud::Ptr init_aligned_scan (new Pointcloud);
Pointcloud::Ptr final_aligned_scan (new Pointcloud);

Pointcloud::Ptr lidarPose (new Pointcloud);

FPFHS33::Ptr target_feature (new FPFHS33);
FPFHS33::Ptr input_feature (new FPFHS33);

Matrix4 total_transformation;
Matrix4 align_transformation;

Vector3 targetViewPoint;
Vector3 inputViewPoint;

size_t original_totalSize = 0;
size_t voxeled_totalSize = 0;
float transScore = 0;
