//
// Created by kristof on 2025.03.23..
//

#include "PointCloudShapes.h"

IPointCloudShape::IPointCloudShape(const string& id_) : id{id_} {
    intensity = 1;
}

IPointCloudShape::IPointCloudShape(const string& id_, float i) :
    id{id_},
    intensity{1/i}
{}

string IPointCloudShape::getId() { return id; }

PointCloudT IPointCloudShape::getShape() { return shape; }

void IPointCloudShape::generateShape() { }
/*********************************************IMPORTED_CLOUD***********************************************************/
ImportedPointCloudShape::ImportedPointCloudShape(const string& id, const string& filePath) :
    IPointCloudShape(id),
    filePath {filePath}
{}

void ImportedPointCloudShape::generateShape() {
    pcl::PointCloud<pcl::PointXYZ> tmp;
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (filePath, tmp) == -1) //* load the file
    {
        PCL_ERROR("Couldn't read file test_pcd.pcd \n");
    }
    pcl::copyPointCloud(tmp, shape);
}
/******************************************RECTANGLE*******************************************************************/
RectanglePointCloudShape::RectanglePointCloudShape(const string& id, float w, float h, float i) :
    IPointCloudShape(id,i),
    width {w},
    height {h}
{}

void RectanglePointCloudShape::generateShape() {
    for (float i = 0; i <= width; i += intensity) {
        for (float j = 0; j <= height; j += intensity) {
            PointType point {i, j, 0};
            shape.push_back(point);
        }
    }
}
/*********************************************CUBOID*******************************************************************/
CuboidPointCloudShape::CuboidPointCloudShape(const string& id, float w, float h, float l, float i) :
    IPointCloudShape(id,i),
    width {w},
    height {h},
    length {l}
{}

void CuboidPointCloudShape::generateShape() {
    for (float i = 0; i <= width; i += intensity) {
        for (float j = 0; j <= height; j += intensity) {
            for (float k = 0; k <= length; k += intensity) {
                PointType point {i, j, k};
                shape.push_back(point);
            }
        }
    }
}
/*********************************************CIRCLE*******************************************************************/
CirclePointCloudShape::CirclePointCloudShape(const string& id, float r, float i) :
    IPointCloudShape(id,i),
    radius {r}
{}

void CirclePointCloudShape::generateShape() {
    float degreeIntensity = 360*intensity / 40;
    for (float i = 0; i <= 360.0f; i += degreeIntensity) {
        for (float j = 0; j <= radius; j += intensity) {
            float rad = i * (M_PI / 180.0f);
            PointType point {j*cos(rad), j*sin(rad), 0};
            shape.push_back(point);
        }
    }
}
/*********************************************SPHERE*******************************************************************/
SpherePointCloudShape::SpherePointCloudShape(const string& id, float r, float i) :
    IPointCloudShape(id,i),
    radius {r}
{}

void SpherePointCloudShape::generateShape() {
    float degreeIntensity = 360*intensity / 40;

    for (float i = 0; i <= 180.0f; i += degreeIntensity) {
        for (float j = 0; j < 360.0f; j += degreeIntensity) {
            float radI = i * (M_PI / 180.0f), radJ = j * (M_PI / 180.0f);
            PointType point {radius*sin(radI)*cos(radJ), radius*sin(radI)*sin(radJ), radius*cos(radI)};
            shape.push_back(point);
        }
    }
}
