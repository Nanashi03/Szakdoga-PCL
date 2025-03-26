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
using EventListener = std::function<void(const std::string&)>;

using namespace std::chrono_literals;

class Viewer {
private:
    pcl::visualization::PCLVisualizer::Ptr viewer;
    static void pointPickingEventOccurred (const pcl::visualization::PointPickingEvent&, void*);
public:
    static EventListener cloudSelectedEventListener;

    Viewer();
    void run();
    void addCloud(const std::string&, PointCloudT::ConstPtr);
    void updateCloud(const std::string&, PointCloudT::ConstPtr);
    void removeCloud(const std::string&);
};

#endif //VIEWER_H
