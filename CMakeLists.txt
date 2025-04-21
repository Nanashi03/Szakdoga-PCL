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
find_package(GTest REQUIRED)

#CORE
add_library(core STATIC
        Model/Model.cpp
        Model/PointCloudShapes.cpp
        Model/Database.cpp

        Model/Model.h
        Model/PointCloudShapes.h
        Model/Database.h
        DataStructures/InputFormData.h
        DataStructures/EditCloudData.h
        DataStructures/BoundingBoxData.h
)

target_include_directories(core PUBLIC
        ${PCL_INCLUDE_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR}/Model
        ${CMAKE_CURRENT_SOURCE_DIR}/DataStructures
)

target_link_libraries(core
        ${PCL_LIBRARIES}
        SQLite::SQLite3
)

#APP

add_executable(PointCloudEditor
        main.cpp
        View/mainwindow.cpp
        View/Viewer.cpp
        Forms/InputDialogBox.cpp
        Forms/HelpDialogBox.cpp
        Controller/Controller.cpp

        View/mainwindow.ui
        Forms/InputDialogBox.ui
        Forms/HelpDialogBox.ui

        View/mainwindow.h
        View/Viewer.h
        Forms/InputDialogBox.h
        Forms/HelpDialogBox.h
        Controller/Controller.h
)

target_include_directories(PointCloudEditor PRIVATE
        ${CMAKE_BINARY_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/Forms
        ${CMAKE_CURRENT_SOURCE_DIR}/View
        ${CMAKE_CURRENT_SOURCE_DIR}/Controller
)

target_link_libraries(PointCloudEditor
        core
        Qt5::Widgets
        Qt5::OpenGL
        Qt5::Concurrent
)

add_definitions(${PCL_DEFINITIONS})

#TESTSUITE
add_executable(PointCloudEditorTestSuite
        Tests/PointCloudShapesTest.cc
        Tests/ModelTest.cc
)

target_include_directories(PointCloudEditorTestSuite PRIVATE
        core
        ${CMAKE_CURRENT_SOURCE_DIR}/Tests/Data
        ${GTEST_INCLUDE_DIRS}
)

target_link_libraries(PointCloudEditorTestSuite
        core
        GTest::GTest
        GTest::Main
)

file(GLOB PCD_FILES ${CMAKE_CURRENT_SOURCE_DIR}/Tests/Data/*.pcd)
foreach(file ${PCD_FILES})
    file(COPY ${file} DESTINATION ${CMAKE_BINARY_DIR}/TestData)
endforeach()

enable_testing()
add_test(NAME TestSrc1 COMMAND test_src1)