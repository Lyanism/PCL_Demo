cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(pcl_tuturial)
add_definitions ("-w") #Suppres warning message 
find_package(PCL 1.8 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} inc)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

SET(UTILS_SRC_LIST src/pcl_utils_data.cpp src/pcl_utils_filter.cpp src/pcl_utils_ransac.cpp src/pcl_utils_viewer.cpp src/pcl_utils_segment.cpp)

add_library (pcl_utils_ts SHARED ${UTILS_SRC_LIST})
target_link_libraries (pcl_utils_ts ${PCL_LIBRARIES})

add_executable (pcl_demo src/pcl_demo.cpp)
target_link_libraries (pcl_demo ${PCL_LIBRARIES} pcl_utils_ts)

add_executable (pcl_test src/pcl_test.cpp)
target_link_libraries (pcl_test ${PCL_LIBRARIES})
