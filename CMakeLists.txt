cmake_minimum_required(VERSION 3.30)
project(src1)

set(CMAKE_AUTOMOC ON) # For meta object compiler
set(CMAKE_AUTORCC ON) # Resource files
set(CMAKE_AUTOUIC ON) # UI files
set(CMAKE_CXX_STANDARD 20)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(Qt5 COMPONENTS Concurrent OpenGL Widgets REQUIRED)
set(QTX Qt5)

include_directories(${CMAKE_BINARY_DIR})

set(UIS
        Forms/InputDialogBox.ui
        View/mainwindow.ui
)

set(SOURCES
        main.cpp
        View/Viewer.cpp
        View/mainwindow.cpp
        Forms/InputDialogBox.cpp
        Controller/Controller.cpp
        Model/Model.cpp
        Model/PointCloudShapes.cpp
)

set(HEADERS
        View/Viewer.h
        View/mainwindow.h
        Forms/InputDialogBox.h
        Controller/Controller.h
        Model/Model.h
        Model/PointCloudShapes.h
        DataStructures/InputFormData.h
        DataStructures/EditCloudData.h
        DataStructures/BoundingBoxData.h
)

add_executable(src1 ${SOURCES} ${HEADERS} ${UIS})

target_link_libraries(src1 ${PCL_LIBRARIES} ${QTX}::Widgets ${QTX}::OpenGL)
