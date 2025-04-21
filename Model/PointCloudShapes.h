#ifndef POINTCLOUDSHAPES_H
#define POINTCLOUDSHAPES_H

#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/transforms.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/common/pca.h>
#include <cmath>

#include "BoundingBoxData.h"

using PointType = pcl::PointXYZRGBNormal;
using PointCloudT = pcl::PointCloud<PointType>;

using namespace std;

class IPointCloudShape {
protected:
    string id;
    PointCloudT::Ptr shapePtr;
    BoundingBoxData boundingBoxData;
    Eigen::Vector3f translationValues;
    Eigen::Affine3f currentRotation;
    vector<float> dimensions;
    Eigen::Vector3i rotationValues;
    pcl::RGB color;
    float density;

    bool isFilled, areNormalsShown, isBoundingBoxDataCalculated;

    bool isColorable, isFillable, isDensitable;
    vector<string> labels;
    vector<bool> showLabels;

    IPointCloudShape(const string&,bool,float);
    void transformPointCloudToCenter();
    void transformPointCloudBackToOriginal();
    void calculateNormals();
    void calculateBoundingBoxData();
public:
    virtual void generateShape();
    virtual void scale(float,float,float);

    void setShape(PointCloudT::Ptr);
    void addToRotationMatrix(const Eigen::Affine3f&);
    void addToTranslationValues(const Eigen::Vector3f&);
    void setRotationAt(int,int);
    void setColor(pcl::RGB);
    void setDensity(int);
    void setIsFilled(bool);
    void setAreNormalsShown(bool);

    const string& getId() const;
    string getNormalId() const;
    PointCloudT::Ptr getShape() const;
    const BoundingBoxData& getBoundingBoxData();
    const Eigen::Vector3f& getTranslationValues() const;
    const Eigen::Affine3f& getCurrentRotation() const;
    vector<float> getDimensions() const;
    int getRotationAt(int) const;
    pcl::RGB getColor() const;
    float getDensity() const;
    bool getIsFilled() const;
    bool getAreNormalsShown() const;
    bool getIsColorable() const;
    bool getIsFillable() const;
    bool getIsDensitable() const;
    vector<string> getLabels() const;
    vector<bool> getShowLabels() const;

    virtual ~IPointCloudShape() = default;
};

class ImportedPointCloudShape : public IPointCloudShape {
private:
    string filePath;
public:
    ImportedPointCloudShape(const string&, string);
    void generateShape() override;
    void scale(float,float,float) override;
};

class RectanglePointCloudShape : public IPointCloudShape {
private:
    float width, height;
public:
    RectanglePointCloudShape(const string&, bool,float,float,float);
    void generateShape() override;
    void scale(float,float,float) override;
};

class CuboidPointCloudShape : public IPointCloudShape {
private:
    float width, height, length;
public:
    CuboidPointCloudShape(const string&, bool,float,float,float,float);
    void generateShape() override;
    void scale(float,float,float) override;
};

class CirclePointCloudShape : public IPointCloudShape {
private:
    float radius;
public:
    CirclePointCloudShape(const string&, bool,float,float);
    void generateShape() override;
    void scale(float,float,float) override;
};

class SpherePointCloudShape : public IPointCloudShape {
private:
    float radius;
public:
    SpherePointCloudShape(const string&, bool,float,float);
    void generateShape() override;
    void scale(float,float,float) override;
};

class CylinderPointCloudShape : public IPointCloudShape {
private:
    float radius, height;
public:
    CylinderPointCloudShape(const string&, bool,float,float,float);
    void generateShape() override;
    void scale(float,float,float) override;
};

class ConePointCloudShape : public IPointCloudShape {
private:
    float radius, height;
public:
    ConePointCloudShape(const string&, bool,float,float,float);
    void generateShape() override;
    void scale(float,float,float) override;
};

#endif //POINTCLOUDSHAPES_H