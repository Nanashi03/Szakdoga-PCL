cmake_minimum_required(VERSION 3.30)
project(src1)

set(CMAKE_AUTOMOC ON) # For meta object compiler
set(CMAKE_AUTORCC ON) # Resource files
set(CMAKE_AUTOUIC ON) # UI files
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(Qt5 COMPONENTS Concurrent OpenGL Widgets REQUIRED)
set(QTX Qt5)

set(CMAKE_CXX_STANDARD 20)

add_executable(src1 main.cpp
        Model/PointCloudShapes.h
        Controller/Controller.cpp
        Controller/Controller.h
        Model/Model.cpp
        Model/Model.h
        View/Viewer.cpp
        View/Viewer.h
)

target_link_libraries(src1 ${PCL_LIBRARIES} ${QTX}::Widgets)
