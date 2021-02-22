/******************************************************************************** 
** @Copyright(c) $year$ $registered organization$ All Rights Reserved.
** @auth： taify
** @date： 2021/01/12
** @desc： myicp源文件
** @Ver : V1.0.0
*********************************************************************************/

#include "myicp.h"
#include "myicp_helpers.h"

MyICP::MyICP()
{

}

MyICP::~MyICP()
{

}

void MyICP::setSourceCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) 
{
	source_cloud = cloud;
}

void MyICP::setTargetCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	target_cloud = cloud;
}

void MyICP::setLeafSize(float size)
{
	leaf_size = size;
}

void MyICP::setMinError(float error)
{
	min_error = error;
}

void MyICP::setMaxIters(int iters)
{
	max_iters = iters;
}

void MyICP::setEpsilon(float eps)
{
	epsilon = eps;
}

void MyICP::downsample()
{
	pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
	voxel_grid.setLeafSize(leaf_size, leaf_size, leaf_size);
	voxel_grid.setInputCloud(source_cloud);
	source_cloud_downsampled.reset(new pcl::PointCloud<pcl::PointXYZ>);
	voxel_grid.filter(*source_cloud_downsampled);
	voxel_grid.setInputCloud(target_cloud);
	target_cloud_downsampled.reset(new pcl::PointCloud<pcl::PointXYZ>);
	voxel_grid.filter(*target_cloud_downsampled);
	std::cout << "down size *cloud_src_o from " << source_cloud->size() << " to " << source_cloud_downsampled->size() << endl;
	std::cout << "down size *cloud_tgt_o from " << target_cloud->size() << " to " << target_cloud_downsampled->size() << endl;
}

void MyICP::registration()
{
	std::cout << "icp registration start..." << std::endl;

	Eigen::Matrix3f R_12 = Eigen::Matrix3f::Identity();
	Eigen::Vector3f T_12 = Eigen::Vector3f::Zero();
	Eigen::Matrix4f H_12 = Eigen::Matrix4f::Identity();

	pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud_mid(new pcl::PointCloud<pcl::PointXYZ>());

	//建立kd树
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
	kdtree->setInputCloud(target_cloud_downsampled);

	double error = INT_MAX, score = INT_MAX;
	Eigen::Matrix4f H_final = H_12;
	int iters = 0;

	//开始迭代，直到满足条件
	while (error > min_error && iters < max_iters)
	{
		iters++;
		double last_error = error;

		//计算最邻近点对
		calNearestPointPairs(H_12, source_cloud_downsampled, target_cloud_downsampled, target_cloud_mid, kdtree, error);

		if (last_error - error < epsilon)
			break;

		//计算点云中心坐标
		Eigen::Vector4f source_centroid, target_centroid_mid;
		pcl::compute3DCentroid(*source_cloud_downsampled, source_centroid);
		pcl::compute3DCentroid(*target_cloud_mid, target_centroid_mid);

		//去中心化
		Eigen::MatrixXf souce_cloud_demean, target_cloud_demean;
		pcl::demeanPointCloud(*source_cloud_downsampled, source_centroid, souce_cloud_demean);
		pcl::demeanPointCloud(*target_cloud_mid, target_centroid_mid, target_cloud_demean);

		//计算W=q1*q2^T
		Eigen::Matrix3f W = (souce_cloud_demean*target_cloud_demean.transpose()).topLeftCorner(3, 3);

		//SVD分解得到新的旋转矩阵和平移矩阵
		Eigen::JacobiSVD<Eigen::Matrix3f> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
		Eigen::Matrix3f U = svd.matrixU();
		Eigen::Matrix3f V = svd.matrixV();

		if (U.determinant()*V.determinant() < 0)
		{
			for (int x = 0; x < 3; ++x)
				V(x, 2) *= -1;
		}

		R_12 = V* U.transpose();
		T_12 = target_centroid_mid.head(3) - R_12*source_centroid.head(3);
		H_12 << R_12, T_12, 0, 0, 0, 1;
		H_final = H_12*H_final; //更新变换矩阵

		std::cout << "iters:"  << iters << "  "<< "error:" << error << std::endl;
	}
	transformation_matrix << H_final;
}

void MyICP::saveICPCloud(const std::string filename)
{
	icp_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::transformPointCloud(*source_cloud, *icp_cloud, transformation_matrix); //点云变换
	pcl::io::savePCDFileBinary(filename, *icp_cloud);
}

void MyICP::getTransformationMatrix()
{
	std::cout << "transformation_matrix:" << std::endl << transformation_matrix << std::endl;
}

void MyICP::getScore()
{
	double fitness_score = 0.0;
	pcl::KdTreeFLANN <pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(target_cloud);

#pragma omp parallel for reduction(+:fitness_score) //采用openmmp加速
	for (int i = 0; i < icp_cloud->points.size(); ++i)
	{
		std::vector<int> nn_indices(1);
		std::vector<float> nn_dists(1);
		kdtree.nearestKSearch(icp_cloud->points[i], 1, nn_indices, nn_dists);
		fitness_score += nn_dists[0];
	}

	std::cout << "score:" << std::endl << fitness_score / icp_cloud->points.size() << std::endl;
}

void MyICP::visualize()
{
	pcl::visualization::PCLVisualizer viewer("registration Viewer");
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> src_h(source_cloud, 0, 255, 0); 	//原始点云绿色
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> tgt_h(target_cloud, 255, 0, 0); 	//目标点云红色
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> final_h(icp_cloud, 0, 0, 255); 	//匹配好的点云蓝色

	viewer.setBackgroundColor(255, 255, 255);
	viewer.addPointCloud(source_cloud, src_h, "source cloud");
	viewer.addPointCloud(target_cloud, tgt_h, "target cloud");
	viewer.addPointCloud(icp_cloud, final_h, "result cloud");

	while (!viewer.wasStopped())
	{
		viewer.spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
}

