/******************************************************************************** 
** @Copyright(c) $year$ $registered organization$ All Rights Reserved.
** @auth�� taify
** @date�� 2021/01/12
** @desc�� myicpͷ�ļ�
** @Ver : V1.0.0
*********************************************************************************/

#ifndef MYICP_H_
#define MYICP_H_

#include <iostream>
#include <fstream>
#include <ctime>
#include <cmath>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/search/kdtree.h> 
#include <pcl/correspondence.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <omp.h>

/**
 * @brief The MyICP class
 */
class MyICP
{
public:
        /**
         * @brief MyICP ���캯��
         */
    MyICP();

	~MyICP();

        /**
         * @brief setSourceCloud �����������
         * @param cloud �������
         */
	void setSourceCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

        /**
         * @brief setTargetCloud ����Ŀ�����
         * @param cloud Ŀ�����
         */
	void setTargetCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

        /**
         * @brief setLeafSize ���������˲�����ߴ�
         * @param size �����˲�����ߴ�
         */
	void setLeafSize(float size);

        /**
         * @brief setMinError ������С���
         * @param error ��С���
         */
	void setMinError(float error);

        /**
         * @brief setMaxIters ��������������
         * @param iters ����������
         */
	void setMaxIters(int iters);

        /**
         * @brief setEpsilon ������׼���
         * @param eps ��׼���
         */
	void setEpsilon(float eps);

        /**
         * @brief downsample �²���
         */
	void downsample();

        /**
         * @brief registration ��׼
         */
	void registration();

        /**
         * @brief saveICPCloud ������׼�õ��ĵ���
         * @param filename �����ļ���
         */
	void saveICPCloud(const std::string filename);

        /**
         * @brief getTransformationMatrix �õ��任����
         */
	void getTransformationMatrix();

        /**
         * @brief getScore �õ���׼�÷�
         */
	void getScore();

        /**
         * @brief visualize ��׼������ӻ����������Ϊ��ɫ��Ŀ�����Ϊ��ɫ����׼����Ϊ��ɫ��
         */
	void visualize();

private:
        /**
         * @brief source_cloud �������
         */
	pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud;

        /**
         * @brief target_cloud Ŀ�����
         */
	pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud;

        /**
         * @brief source_cloud_downsampled ��������²����õ��ĵ���
         */
	pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud_downsampled;

        /**
         * @brief target_cloud_downsampled Ŀ������²����õ��ĵ���
         */
	pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud_downsampled;

        /**
         * @brief icp_cloud ��׼�õ��ĵ���
         */
	pcl::PointCloud<pcl::PointXYZ>::Ptr icp_cloud;

        /**
         * @brief leaf_size ��������ߴ�
         */
	float leaf_size;

        /**
         * @brief min_error ��С���
         */
	float min_error;

        /**
         * @brief max_iters ����������
         */
	int max_iters;

        /**
         * @brief epsilon ��׼���
         */
	float epsilon;

        /**
         * @brief transformation_matrix �任����
         */
	Eigen::Matrix4f transformation_matrix = Eigen::Matrix4f::Identity();
};

#endif // MYICP_H
