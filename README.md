#自己实现的ICP点云配准算法
#使用了第三方库PCL，但未使用其配准模块的API
#算法比PCL库的API快速，对于大数据集速度最快可至PCL库的API的1/3；小数据集不太稳定，有时结果和速度都较快，也有时速度略慢
#本工程文件采用CMake管理，经测试在windows10系统的vs2015 professional和qt5.9.1平台以及Ubutnu16.04系统上均可正常工作。

data文件夹下为数据集，skeleton.pcd/skeleton_model.pcd为默认数据集。若使用bunny1.pcd/bunny2.pcd测试，建议修改setLeafSize参数为0.01，setMinError参数为0.0001。
include文件夹下为头文件。
Release文件夹下为windows下的可执行程序和所依赖的动态库。
src文件夹下为源文件。

windows下运行程序：
MyICP.exe <source.pcd> <target.pcd>

Linux下运行程序：
mkdir build
cd build
cmake ..
make
./MyICP <source.pcd> <target.pcd>
