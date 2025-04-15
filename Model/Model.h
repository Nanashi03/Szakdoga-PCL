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

#include "../DataStructures/EditCloudData.h"
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
        vector<shared_ptr<IPointCloudShape>> clouds;
        int selectedCloud;

        void createBoundingBoxAround(int, BoundingBoxData&);
    public:
        Model();
        void addCloud(const shared_ptr<IPointCloudShape>&);
        void updateSelectedCloudDimensions(float,float,float);
        void removeCloud(const shared_ptr<IPointCloudShape>&);
        void colorSelectedCloud(pcl::RGB);

        void selectCloud(const string&, BoundingBoxData&);
        void deSelectCloud();
        PointCloudT::ConstPtr translateSelectedCloud(float,float,float);
        PointCloudT::ConstPtr rotateSelectedCloud(float,char);

        EditCloudData getEditCloudData();
        PointCloudT::ConstPtr getSelectedCloudShape();
        string getSelectedCloudName();
        bool getSelectedCloudAreNormalsPresent();
        bool isCloudSelected();
};

#endif //MODEL_H
