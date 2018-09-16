#include <ese_icp/ese_slam.h>

void MatrixInit (void)
{
    total_transformation << 1,0,0,0,
                            0,1,0,0,
                            0,0,1,0,
                            0,0,0,1;

	targetViewPoint << 0,0,0;
	inputViewPoint << 0,0,0;

	lidarPose->points.resize(1);
	lidarPose->points[0].x = 0;
	lidarPose->points[0].y = 0;
	lidarPose->points[0].z = 0;
	lidarPose->points[0].intensity = 0;
}

void getTotalTransformation
(Matrix4 initial_transformation, Matrix4 icp_transformation)
{
	total_transformation = icp_transformation * initial_transformation * total_transformation;
	align_transformation = icp_transformation * initial_transformation;  

	Pointcloud::Ptr tempPose (new Pointcloud);
	tempPose->points.resize(1);
	tempPose->points[0].x = total_transformation(0,3);
	tempPose->points[0].y = total_transformation(1,3);
	tempPose->points[0].z = total_transformation(2,3);
	tempPose->points[0].intensity = 0;

	*lidarPose += *tempPose;                   
}

void ReadPCD 
(Pointcloud::Ptr &target_cloud, int file_num)
{
	std::string file_adrs = "/home/mbek/Desktop/bag_file/ouster_pcd2/";
	std::stringstream file_read;
	file_read << file_adrs << file_num << ".pcd";

	if(pcl::io::loadPCDFile<PointI> (file_read.str() , *target_cloud) == -1)
	{
		PCL_ERROR ("Could not read file of %d.pcd",file_num);
	}
}

void poseTransformation (void)
{
    pcl::transformPointCloud(*voxeled_scan_data, *voxeled_scan_data, total_transformation);

    pcl::transformPointCloud(*original_scan_data, *original_scan_data, total_transformation);
}

void updateMapData(void)
{
    *original_map_data += *original_scan_data;
    *voxeled_map_data += *final_aligned_scan;

    original_totalSize = original_map_data->points.size();
	voxeled_totalSize = voxeled_map_data->points.size();
}

void updateLocalMapData(void)
{
	if (numberOfLocalmap == 0 && current_index < 5)
	{
		if (current_index == 0)
			*current_localmap = *original_map_data;
		else
			*current_localmap += *original_scan_data;
		current_index++;
	}
	else
	{
		*current_localmap += *original_scan_data;
		*next_localmap += *original_scan_data;
		current_index++;
		next_index++;

		/*if (current_index == 10 && next_index == 5)
		{
			//write pcd file current local map
			std::string file_adrs = "/home/mbek/Desktop/bag_file/localmap_";
			std::stringstream ss;
			ss << file_adrs << numberOfLocalmap << ".pcd";

			current_localmap->height = 1;
			current_localmap->width = current_localmap->points.size();
			pcl::io::savePCDFileASCII (ss.str(), *current_localmap);
			ROS_INFO("write %d.pcd file", numberOfLocalmap);

			//next -> current
			current_localmap->clear();
			*current_localmap = *next_localmap;
			next_localmap->clear();

			current_index = 5;
			next_index = 0;
			numberOfLocalmap++;
		}*/
	}
}

void voxelFiltering 
(Pointcloud::Ptr &input_cloud, Pointcloud::Ptr &target_cloud, float leafSize, float ground_threshold)
{
	size_t point_index = 0, ground_index_ = 0;
	pcl::PointIndices::Ptr ground_index (new pcl::PointIndices());

	while(point_index < input_cloud->points.size())
	{
		if (input_cloud->points[point_index].z < targetViewPoint(2) - ground_threshold )
		{
			ground_index->indices.push_back(point_index);
		}

		point_index++;
	}

	//extract
	pcl::ExtractIndices<PointI> extract;
	Pointcloud::Ptr extracted_cloud (new Pointcloud);

	extract.setInputCloud (input_cloud);
	extract.setIndices (ground_index);
	extract.setNegative (true);
	extract.filter (*extracted_cloud);

	//voxel
	pcl::VoxelGrid<PointI> voxel;
	voxel.setLeafSize (leafSize, leafSize, leafSize);
	voxel.setInputCloud (extracted_cloud);
	voxel.filter (*target_cloud);
}

void EstimateFeature 
(Pointcloud::Ptr &input_cloud, Vector3 viewPoint, FPFHS33::Ptr &input_feature,
 float normal_radius, float fpfh_radius)
{
	PointNormal::Ptr input_normal (new PointNormal);
	pcl::search::KdTree<PointI>::Ptr input_tree (new pcl::search::KdTree<PointI>);
	pcl::NormalEstimation<PointI, Normal> normal;

	normal.setInputCloud (input_cloud);
	normal.setSearchMethod (input_tree);
	normal.setRadiusSearch (normal_radius);
	normal.setViewPoint (viewPoint(0), viewPoint(1), viewPoint(2));
	normal.compute(*input_normal);

	pcl::FPFHEstimation <PointI, Normal, FPFHS> fpfh;
	
	fpfh.setInputCloud (input_cloud);
	fpfh.setInputNormals (input_normal);
	fpfh.setSearchMethod (input_tree);
	fpfh.setRadiusSearch (fpfh_radius);
	fpfh.compute (*input_feature);
} 

Matrix4 InitailAlign 
(const Pointcloud::Ptr &input_cloud, const FPFHS33::Ptr &input_feature,
 const Pointcloud::Ptr &target_cloud, const FPFHS33::Ptr &target_feature,
 const int numberOfSamples, const int initial_iteration)
{
	pcl::SampleConsensusInitialAlignment<PointI, PointI, FPFHS> sac;
    sac.setMinSampleDistance (0.001);
    sac.setMaxCorrespondenceDistance (5);
    sac.setMaximumIterations (initial_iteration);
    sac.setNumberOfSamples (numberOfSamples);

	sac.setInputCloud (input_cloud);
    sac.setSourceFeatures (input_feature);
    sac.setInputTarget (target_cloud);
    sac.setTargetFeatures (target_feature);

    sac.align (*init_aligned_scan);

    return (sac.getFinalTransformation());
}

Matrix4 ICPAlign
(const Pointcloud::Ptr &input_cloud, const Pointcloud::Ptr &target_cloud,
 const int icp_iteration)
{
    pcl::IterativeClosestPoint<PointI, PointI> icp;

    icp.setInputCloud (input_cloud);
    icp.setInputTarget (target_cloud);

    icp.setMaximumIterations (icp_iteration);
    icp.setMaxCorrespondenceDistance (5);
    icp.align (*final_aligned_scan);
    
    transScore = icp.getFitnessScore();
    ROS_INFO ("%f ",icp.getFitnessScore());
    return (icp.getFinalTransformation());
}

int main (int argc, char** argv)
{
	ros::init(argc, argv, "read_pcd");
	ros::NodeHandle nh;
	ros::NodeHandle ns("~");

	int start_file, end_file, numberOfSamples, initial_iteration, icp_iteration;
	float leaf_size, ground_threshold, normal_radius, fpfh_radius, score_threshold;

	ns.param ("start_file",start_file, 250);
	ns.param ("end_file", end_file, 351);

	ns.param ("leaf_size", leaf_size, 0.25f);
	ns.param ("ground_threshold",ground_threshold, -1.8f);
	ns.param ("normal_radius",normal_radius, 5.0f);
	ns.param ("fpfh_radius",fpfh_radius, 5.0f);
	ns.param ("numberOfSamples",numberOfSamples, 50);
	ns.param ("initial_iteration",initial_iteration, 50);
	ns.param ("icp_iteration",icp_iteration, 50);
	ns.param ("score_threshold", score_threshold, 2.0f);

	ros::Publisher map_pub = nh.advertise<msgs_Point>("map_cloud",1000);
    ros::Publisher scan_pub = nh.advertise<msgs_Point>("icp_cloud",1000);
	ros::Publisher odom_pub = nh.advertise<msgs_Point>("lidar_odom",1000);

	MatrixInit();
	int file_num = start_file + 1;

	ReadPCD (original_based_data, start_file);
	ReadPCD (original_map_data, start_file);
	voxelFiltering (original_map_data, voxeled_map_data, leaf_size, ground_threshold);
	voxeled_totalSize += voxeled_map_data->points.size();
	original_totalSize += original_map_data->points.size();
	updateLocalMapData();

	while(ros::ok())
	{
		if (file_num == end_file)
			break;

		voxelFiltering (original_based_data, voxeled_based_data, leaf_size, ground_threshold);
		EstimateFeature (voxeled_based_data, targetViewPoint, target_feature, normal_radius, fpfh_radius);

		ReadPCD (original_scan_data, file_num);
		voxelFiltering (original_scan_data, voxeled_scan_data, leaf_size, ground_threshold);
		poseTransformation();

		EstimateFeature (voxeled_scan_data, inputViewPoint, input_feature, normal_radius, fpfh_radius);
		Matrix4 initial_transformation = InitailAlign (voxeled_scan_data, input_feature, voxeled_based_data, target_feature,
													   numberOfSamples, initial_iteration);
		Matrix4 icp_transformation = ICPAlign (init_aligned_scan, voxeled_based_data, icp_iteration);

		if (transScore < score_threshold)
		{
			getTotalTransformation(initial_transformation, icp_transformation);
			pcl::transformPointCloud(*original_scan_data, *original_scan_data, align_transformation);
			updateMapData();
			updateLocalMapData();
			*original_based_data = *original_scan_data;
			targetViewPoint = inputViewPoint;
			inputViewPoint(0) = total_transformation(0,3);
			inputViewPoint(1) = total_transformation(1,3);
			inputViewPoint(2) = total_transformation(2,3);
			ROS_INFO("%d done!",file_num);
		}

		msgs_Point aligned_data_;
        msgs_Point map_data_;
		msgs_Point odom_data_;
        pcl::toROSMsg (*final_aligned_scan, aligned_data_);
        pcl::toROSMsg (*voxeled_based_data, map_data_);
		pcl::toROSMsg (*lidarPose, odom_data_);
        aligned_data_.header.frame_id = "base_link";
        aligned_data_.header.stamp = ros::Time::now();
        map_data_.header.frame_id = "base_link";
        map_data_.header.stamp = ros::Time::now();
		odom_data_.header.frame_id = "base_link";
		odom_data_.header.stamp = ros::Time::now();
        scan_pub.publish(aligned_data_);
		map_pub.publish(map_data_);
		odom_pub.publish(odom_data_);

		file_num++;
	}

	original_map_data->height = 1;
	original_map_data->width = original_totalSize;
	pcl::io::savePCDFileASCII ("/home/mbek/Desktop/bag_file/os1_map.pcd", *original_map_data);
	ROS_INFO("write os1_map.pcd file");

	voxeled_map_data->height = 1;
	voxeled_map_data->width = voxeled_totalSize;
	pcl::io::savePCDFileASCII ("/home/mbek/Desktop/bag_file/os1_voxel_map.pcd", *voxeled_map_data);
	ROS_INFO("write os1_voxel_map.pcd file");

	return (0);
}