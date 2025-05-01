#ifndef VIEWER_H
#define VIEWER_H

#include <thread>
#include <memory>
#include <functional>
#include <vtkSmartPointer.h>
#include <vtkAxesActor.h>
#include <vtkOrientationMarkerWidget.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "BoundingBoxData.h"

using PointType = pcl::PointXYZRGBNormal;
using PointCloudT = pcl::PointCloud<PointType>;
using PointCloudNormals = pcl::PointCloud<pcl::Normal>;

using EventClickListener = std::function<void(const std::string&)>;
using EventButtonListener = std::function<void(int,int,int)>;

using namespace std::chrono_literals;

class Viewer {
    Eigen::Affine3f boundingBoxTransform;
    pcl::visualization::PCLVisualizer::Ptr viewer;
    vtkSmartPointer<vtkOrientationMarkerWidget>  axesWidget;
    static void pointPickingEventOccurred (const pcl::visualization::PointPickingEvent&, void*);
    static void keyboardPressingEventOccurred (const pcl::visualization::KeyboardEvent&, void*);
public:
    inline static EventClickListener cloudSelectedEventListener = nullptr;
    inline static EventButtonListener selectedCloudTranslateLeftEventListener = nullptr;

    Viewer();

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