/********************************************************************************
** @Copyright(c) $year$ $registered organization$ All Rights Reserved.
** @auth： taify
** @date： 2021/01/12
** @desc： myicp_helpers头文件
** @Ver : V1.0.0
*********************************************************************************/

#ifndef MYICP_HELPERS_H_
#define MYICP_HELPERS_H_

#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/search/kdtree.h> 

/**
 * @brief calNearestPointPairs 计算最邻近点对
 * @param H 变换矩阵
 * @param source_cloud 源点云
 * @param target_cloud 目标点云
 * @param target_cloud_mid 中间点云
 * @param kdtree kd树
 * @param error 误差
 */
void calNearestPointPairs(Eigen::Matrix4f H, pcl::PointCloud<pcl::PointXYZ>::Ptr& source_cloud, const pcl::PointCloud<pcl::PointXYZ>::Ptr& target_cloud,
	pcl::PointCloud<pcl::PointXYZ>::Ptr& target_cloud_mid, pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree, double &error);

#endif // MYICP_HELPERS_H_
