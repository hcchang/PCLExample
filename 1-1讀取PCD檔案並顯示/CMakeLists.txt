cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(1-1_LoadPCDFileAndShow)
find_package(PCL 1.7 REQUIRED COMPONENTS common io visualization)


include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(1-1_LoadPCDFileAndShow LoadPCDFileAndShow.cpp)
target_include_directories(1-1_LoadPCDFileAndShow PUBLIC ${PCL_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS})
target_link_libraries(1-1_LoadPCDFileAndShow ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_VISUALIZATION_LIBRARIES})



