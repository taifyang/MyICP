/********************************************************************************
** @Copyright(c) $year$ $registered organization$ All Rights Reserved.
** @auth�� taify
** @date�� 2021/01/12
** @desc�� myicp_helpersͷ�ļ�
** @Ver : V1.0.0
*********************************************************************************/

#ifndef MYICP_HELPERS_H_
#define MYICP_HELPERS_H_

#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/search/kdtree.h> 

/**
 * @brief calNearestPointPairs �������ڽ����
 * @param H �任����
 * @param source_cloud Դ����
 * @param target_cloud Ŀ�����
 * @param target_cloud_mid �м����
 * @param kdtree kd��
 * @param error ���
 */
void calNearestPointPairs(Eigen::Matrix4f H, pcl::PointCloud<pcl::PointXYZ>::Ptr& source_cloud, const pcl::PointCloud<pcl::PointXYZ>::Ptr& target_cloud,
	pcl::PointCloud<pcl::PointXYZ>::Ptr& target_cloud_mid, pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree, double &error);

#endif // MYICP_HELPERS_H_
