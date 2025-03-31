//asd
// Created by kristof on 2025.03.16..
//

#ifndef MODEL_H
#define MODEL_H

#include <vector>
#include <memory>
#include <pcl/common/transforms.h>
#include <algorithm>

#include "PointCloudShapes.h"

class Model {
    private:
        std::pmr::vector<IPointCloudShape> clouds;
        int selectedCloud;

    public:
        Model();
        void addCloud(const IPointCloudShape&);
        void updateCloud(const IPointCloudShape&);
        void removeCloud(const IPointCloudShape&);
        void colorCloud(pcl::RGB, int);

        PointCloudT::ConstPtr selectCloud(const string&);
        PointCloudT::ConstPtr deSelectCloud();
        PointCloudT::ConstPtr translateSelectedCloud(float,float,float);
        PointCloudT::ConstPtr rotateSelectedCloud(float,char);

        string getSelectedCloudName();
        bool isCloudSelected();
};

#endif //MODEL_H
