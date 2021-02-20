/******************************************************************************** 
** @Copyright(c) $year$ $registered organization$ All Rights Reserved.
** @auth： taify
** @date： 2021/01/13
** @desc： MyICP调用demo
** @Ver : V1.0.0
*********************************************************************************/

#include "myicp.h"
#include "myicp_helpers.h"

int main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	
	//加载点云
	pcl::io::loadPCDFile(argv[1], *source_cloud);
	pcl::io::loadPCDFile(argv[2], *target_cloud);

	//运行算法
	MyICP icp;
	icp.setSourceCloud(source_cloud);
	icp.setTargetCloud(target_cloud);
	icp.setLeafSize(3);
	icp.downsample();
	icp.setMinError(0.01);
	icp.setMaxIters(50);
	icp.registration();
	icp.saveICPCloud("icp_cloud.pcd");
	icp.getTransformationMatrix();
	icp.getScore();
	icp.visualize();

	system("pause");
	return 0;
}