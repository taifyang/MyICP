/******************************************************************************** 
** @Copyright(c) $year$ $registered organization$ All Rights Reserved.
** @auth： taify
** @date： 2021/01/12
** @desc： myicp_helpers源文件
** @Ver : V1.0.0
*********************************************************************************/

#include "myicp_helpers.h"

void calNearestPointPairs(Eigen::Matrix4f H, pcl::PointCloud<pcl::PointXYZ>::Ptr& source_cloud, const pcl::PointCloud<pcl::PointXYZ>::Ptr& target_cloud,
	pcl::PointCloud<pcl::PointXYZ>::Ptr& target_cloud_mid, pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree, double &error)
{
	double err = 0.0;
	pcl::transformPointCloud(*source_cloud, *source_cloud, H);
	std::vector<int>indexs(source_cloud->size());

#pragma omp parallel for reduction(+:err) //采用openmmp加速
	for (int i = 0; i < source_cloud->size(); ++i)
	{
		std::vector<int>index(1);
		std::vector<float>distance(1);
		kdtree->nearestKSearch(source_cloud->points[i], 1, index, distance);
		err = err + sqrt(distance[0]);
		indexs[i] = index[0];
	}

	pcl::copyPointCloud(*target_cloud, indexs, *target_cloud_mid);
	error = err / source_cloud->size();
}