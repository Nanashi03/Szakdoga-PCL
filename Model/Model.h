//asd
// Created by kristof on 2025.03.16..
//

#ifndef MODEL_H
#define MODEL_H

#include <vector>
#include <memory>
#include <pcl/common/common.h>
#include <pcl/common/pca.h>

#include "EditCloudData.h"
#include "BoundingBoxData.h"
#include "PointCloudShapes.h"

class Model {
    private:
        vector<shared_ptr<IPointCloudShape>> clouds;
        int selectedCloud;
    public:
        Model();
        void addCloud(const shared_ptr<IPointCloudShape>&);
        void exportClouds(const string&);

        void selectCloud(const string&);
        void deSelectCloud();

        void removeSelectedCloud();
        void colorSelectedCloud(pcl::RGB);
        void updateSelectedCloudDimensions(float,float,float);
        void updateSelectedCloudDensity(int);
        void updateSelectedCloudIsFilled(bool);
        void updateSelectedCloudAreNormalsShown(bool);
        void translateSelectedCloud(float,float,float,Eigen::Affine3f&);
        void rotateSelectedCloud(int,char,Eigen::Affine3f&);

        EditCloudData getEditCloudData();
        BoundingBoxData getBoundingBoxDataAroundSelectedCloud();

        PointCloudT::ConstPtr getSelectedCloudShape();
        string getSelectedCloudName();
        string getSelectedCloudNormalsName();
        bool getSelectedCloudAreNormalsShown();
        bool isCloudSelected();
};

#endif //MODEL_H
