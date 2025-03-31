//asd
// Created by kristof on 2025.03.16..
//

#ifndef MODEL_H
#define MODEL_H

#include <vector>
#include <memory>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/common/pca.h>

#include "PointCloudShapes.h"

struct BoundingBoxData
{
    Eigen::Quaternionf bboxQuaternion;
    Eigen::Vector3f bboxTransform;
    double width, height, depth;
    const string NAME = "BBOX";
};

class Model {
    private:
        std::pmr::vector<IPointCloudShape> clouds;
        int selectedCloud;
        //void createBoundingBoxAround(int);
    public:
        void createBoundingBoxAround(int, BoundingBoxData&);
        Model();
        void addCloud(const IPointCloudShape&);
        void updateCloud(const IPointCloudShape&);
        void removeCloud(const IPointCloudShape&);
        void colorCloud(pcl::RGB, int);

        void selectCloud(const string&, BoundingBoxData&);
        void deSelectCloud();
        PointCloudT::ConstPtr translateSelectedCloud(float,float,float);
        PointCloudT::ConstPtr rotateSelectedCloud(float,char);

        string getSelectedCloudName();
        bool isCloudSelected();
};

#endif //MODEL_H
