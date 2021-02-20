# CMake 最低版本号要求
cmake_minimum_required (VERSION 2.8)

# 项目信息
project (MyICP)

# 查找PCL库
find_package(PCL 1.2 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# 查找目录下的所有源文件和头文件
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src)
AUX_SOURCE_DIRECTORY(${PROJECT_SOURCE_DIR}/src SRCS)
FILE(GLOB INC_PATH ${PROJECT_SOURCE_DIR}/include/*.h)
LIST(APPEND INCS ${INC_PATH} )

# 指定生成目标
add_executable(MyICP main.cpp ${INCS} ${SRCS} )
target_link_libraries (MyICP ${PCL_LIBRARIES})
