cmake_minimum_required(VERSION 3.30)
project(src1)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(CMAKE_CXX_STANDARD 20)

add_executable(src1 main.cpp
        Model/PointCloudShapes.cpp
        Model/PointCloudShapes.h
        Controller/Controller.cpp
        Controller/Controller.h
        Model/Model.cpp
        Model/Model.h
)

target_link_libraries(src1 ${PCL_LIBRARIES})
#include_directories(/home/linuxbrew/.linuxbrew/Cellar)

#link_directories(/home/linuxbrew/.linuxbrew/Cellar/pcl/1.14.1_3/include)
#link_directories(/home/linuxbrew/.linuxbrew/Cellar/pcl/1.14.1_3/include/pcl-1.14/pcl)

#include_directories("/home/linuxbrew/.linuxbrew/Cellar/pcl/1.14.1_3/include/pcl-1.14/pcl")
#target_link_libraries(src1 "/home/linuxbrew/.linuxbrew/Cellar/pcl/1.14.1_3/include/pcl-1.14/pcl")
