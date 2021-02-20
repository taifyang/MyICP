/******************************************************************************** 
** @Copyright(c) $year$ $registered organization$ All Rights Reserved.
** @auth： taify
** @date： 2021/01/12
** @desc： myicp头文件
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
         * @brief MyICP 构造函数
         */
    MyICP();

	~MyICP();

        /**
         * @brief setSourceCloud 设置输入点云
         * @param cloud 输入点云
         */
	void setSourceCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

        /**
         * @brief setTargetCloud 设置目标点云
         * @param cloud 目标点云
         */
	void setTargetCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

        /**
         * @brief setLeafSize 设置体素滤波网格尺寸
         * @param size 体素滤波网格尺寸
         */
	void setLeafSize(float size);

        /**
         * @brief setMinError 设置最小误差
         * @param error 最小误差
         */
	void setMinError(float error);

        /**
         * @brief setMaxIters 设置最大迭代次数
         * @param iters 最大迭代次数
         */
	void setMaxIters(int iters);

        /**
         * @brief setEpsilon 设置配准误差
         * @param eps 配准误差
         */
	void setEpsilon(float eps);

        /**
         * @brief downsample 下采样
         */
	void downsample();

        /**
         * @brief registration 配准
         */
	void registration();

        /**
         * @brief saveICPCloud 保存配准得到的点云
         * @param filename 点云文件名
         */
	void saveICPCloud(const std::string filename);

        /**
         * @brief getTransformationMatrix 得到变换矩阵
         */
	void getTransformationMatrix();

        /**
         * @brief getScore 得到配准得分
         */
	void getScore();

        /**
         * @brief visualize 配准结果可视化（输入点云为绿色，目标点云为红色，配准点云为蓝色）
         */
	void visualize();

private:
        /**
         * @brief source_cloud 输入点云
         */
	pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud;

        /**
         * @brief target_cloud 目标点云
         */
	pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud;

        /**
         * @brief source_cloud_downsampled 输入点云下采样得到的点云
         */
	pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud_downsampled;

        /**
         * @brief target_cloud_downsampled 目标点云下采样得到的点云
         */
	pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud_downsampled;

        /**
         * @brief icp_cloud 配准得到的点云
         */
	pcl::PointCloud<pcl::PointXYZ>::Ptr icp_cloud;

        /**
         * @brief leaf_size 体素网格尺寸
         */
	float leaf_size;

        /**
         * @brief min_error 最小误差
         */
	float min_error;

        /**
         * @brief max_iters 最大迭代次数
         */
	int max_iters;

        /**
         * @brief epsilon 配准误差
         */
	float epsilon;

        /**
         * @brief transformation_matrix 变换矩阵
         */
	Eigen::Matrix4f transformation_matrix = Eigen::Matrix4f::Identity();
};

#endif // MYICP_H
