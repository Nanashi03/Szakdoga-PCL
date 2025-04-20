cmake_minimum_required(VERSION 3.30)
project(src1)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

find_package(PCL REQUIRED)
find_package(Qt5 COMPONENTS Concurrent OpenGL Widgets REQUIRED)
find_package(SQLite3 REQUIRED)

add_executable(src1
        main.cpp
        View/mainwindow.cpp
        View/Viewer.cpp
        Forms/InputDialogBox.cpp
        Forms/HelpDialogBox.cpp
        Controller/Controller.cpp
        Model/Model.cpp
        Model/PointCloudShapes.cpp
        Model/Database.cpp

        View/mainwindow.ui
        Forms/InputDialogBox.ui
        Forms/HelpDialogBox.ui

        View/mainwindow.h
        View/Viewer.h
        Forms/InputDialogBox.h
        Forms/HelpDialogBox.h
        Controller/Controller.h
        Model/Model.h
        Model/PointCloudShapes.h
        DataStructures/InputFormData.h
        DataStructures/EditCloudData.h
        DataStructures/BoundingBoxData.h
        Model/Database.h
)

target_include_directories(src1 PRIVATE
        ${PCL_INCLUDE_DIRS}
        ${CMAKE_BINARY_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/Forms
        ${CMAKE_CURRENT_SOURCE_DIR}/View
        ${CMAKE_CURRENT_SOURCE_DIR}/Model
        ${CMAKE_CURRENT_SOURCE_DIR}/Controller
        ${CMAKE_CURRENT_SOURCE_DIR}/DataStructures
)

target_link_libraries(src1
        ${PCL_LIBRARIES}
        Qt5::Widgets
        Qt5::OpenGL
        Qt5::Concurrent
        SQLite::SQLite3
)

add_definitions(${PCL_DEFINITIONS})