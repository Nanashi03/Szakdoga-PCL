//asdasd
// Created by kristof on 2025.03.17..
//

#ifndef VIEWER_H
#define VIEWER_H

#include <pcl/visualization/pcl_visualizer.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <thread>
#include <memory>
#include <functional>

#include "BoundingBoxData.h"


using PointType = pcl::PointXYZRGBNormal;
using PointCloudT = pcl::PointCloud<PointType>;
using PointCloudNormals = pcl::PointCloud<pcl::Normal>;

using EventClickListener = std::function<void(const std::string&)>;
using EventButtonListener = std::function<void(int,int,int)>;

using namespace std::chrono_literals;

class Viewer {
private:
    Eigen::Affine3f boundingBoxTransform;
    pcl::visualization::PCLVisualizer::Ptr viewer;
    static void pointPickingEventOccurred (const pcl::visualization::PointPickingEvent&, void*);
    static void keyboardPressingEventOccurred (const pcl::visualization::KeyboardEvent&, void*);
public:
    static EventClickListener cloudSelectedEventListener;
    static EventButtonListener selectedCloudTranslateLeftEventListener;

    Viewer();
    void run();

    void addCloud(const std::string&, bool, PointCloudT::ConstPtr);
    void addNormals(const std::string&, PointCloudT::ConstPtr);
    void updateCloud(const std::string&, bool, PointCloudT::ConstPtr);
    void removeCloud(const std::string&, bool);
    void removeNormals(const std::string&);

    void addBoundingBoxCube(const BoundingBoxData&);
    void removeBoundingBoxCube();
    void translateBoundingBoxCube(const Eigen::Affine3f&);
    void rotateBoundingBoxCube(const Eigen::Affine3f&);

    void init(vtkRenderer*, vtkGenericOpenGLRenderWindow*);
    vtkSmartPointer<vtkRenderWindow> getRender();
    void setupInteractor(vtkRenderWindowInteractor*, vtkRenderWindow*);
};

#endif //VIEWER_H
