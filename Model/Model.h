#ifndef MODEL_H
#define MODEL_H

#include <vector>
#include <memory>

#include "EditCloudData.h"
#include "BoundingBoxData.h"
#include "Database.h"
#include "PointCloudShapes.h"

class Model {
    vector<shared_ptr<IPointCloudShape>> clouds;
    int selectedCloud;
    shared_ptr<IPointCloudShape> createPointCloudShape(const string&, const string&, bool, float, float, float, float);
public:
    Model();
    void addCloud(const shared_ptr<IPointCloudShape>&);
    void importProject(const string&);
    void exportClouds(const string&);
    void exportProject(const string&);

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

    EditCloudData getEditCloudData(const string&);
    BoundingBoxData getBoundingBoxDataAroundSelectedCloud() const;

    vector<string> getCloudNames() const;
    vector<shared_ptr<IPointCloudShape>> getClouds();
    PointCloudT::ConstPtr getSelectedCloudShape() const;
    string getSelectedCloudName() const;
    string getSelectedCloudNormalsName() const;
    bool getSelectedCloudAreNormalsShown() const;
    bool isCloudSelected() const;
};

#endif //MODEL_H