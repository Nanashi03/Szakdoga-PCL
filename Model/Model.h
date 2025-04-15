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
#include "../DataStructures/BoundingBoxData.h"
#include "PointCloudShapes.h"

class Model {
    private:
        vector<shared_ptr<IPointCloudShape>> clouds;
        int selectedCloud;
    public:
        Model();
        void addCloud(const shared_ptr<IPointCloudShape>&);
        void updateSelectedCloudDimensions(float,float,float);
        void removeCloud(const shared_ptr<IPointCloudShape>&);
        void colorSelectedCloud(pcl::RGB);

        void selectCloud(const string&);
        void deSelectCloud();
        PointCloudT::ConstPtr translateSelectedCloud(float,float,float);
        PointCloudT::ConstPtr rotateSelectedCloud(float,char);

        EditCloudData getEditCloudData();
        BoundingBoxData getBoundingBoxDataAroundSelectedCloud();

        PointCloudT::ConstPtr getSelectedCloudShape();
        string getSelectedCloudName();
        bool getSelectedCloudAreNormalsPresent();
        bool isCloudSelected();
};

#endif //MODEL_H
