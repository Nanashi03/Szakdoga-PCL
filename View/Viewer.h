//asdasd
// Created by kristof on 2025.03.17..
//

#ifndef VIEWER_H
#define VIEWER_H

#include <pcl/visualization/pcl_visualizer.h>
#include <thread>
#include <memory>
#include <functional>


using PointType = pcl::PointXYZRGBNormal;
using PointCloudT = pcl::PointCloud<PointType>;
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

    void addCloud(const std::string&, PointCloudT::ConstPtr);
    void updateCloud(const std::string&, PointCloudT::ConstPtr);
    void removeCloud(const std::string&);

    void addBoundingBoxCube(const Eigen::Vector3f&, const Eigen::Quaternionf&, double, double, double);
    void removeBoundingBoxCube();
    void translateBoundingBoxCube(float,float,float);
};

#endif //VIEWER_H
