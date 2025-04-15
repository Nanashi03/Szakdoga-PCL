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
        void removeSelectedCloud();
        void generateNormalsForSelectedCloud();

        void selectCloud(const string&);
        void deSelectCloud();

        void colorSelectedCloud(pcl::RGB);
        void updateSelectedCloudDimensions(float,float,float);
        void updateSelectedCloudDensity(int);
        void updateSelectedCloudIsFilled(bool);
        void updateSelectedCloudAreNormalsPresent(bool);
        PointCloudT::ConstPtr translateSelectedCloud(float,float,float);
        PointCloudT::ConstPtr rotateSelectedCloud(float,char);

        EditCloudData getEditCloudData();
        BoundingBoxData getBoundingBoxDataAroundSelectedCloud();

        PointCloudT::ConstPtr getSelectedCloudShape();
        string getSelectedCloudName();
        string getSelectedCloudNormalsName();
        bool getSelectedCloudAreNormalsPresent();
        bool isCloudSelected();
};

#endif //MODEL_H
