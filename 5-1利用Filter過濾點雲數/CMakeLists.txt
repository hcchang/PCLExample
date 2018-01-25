cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(5-1_VoxelFilterForPointCloud)
find_package(PCL 1.7 REQUIRED COMPONENTS common io visualization)


include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(5-1_VoxelFilterForPointCloud VoxelFilterForPointCloud.cpp)
target_include_directories(5-1_VoxelFilterForPointCloud PUBLIC ${PCL_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS})
target_link_libraries(5-1_VoxelFilterForPointCloud ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_VISUALIZATION_LIBRARIES})



