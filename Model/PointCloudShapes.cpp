//asd
// Created by kristof on 2025.03.23..
//

#include "PointCloudShapes.h"

IPointCloudShape::IPointCloudShape(const string& id_, bool iF = false, float d = 1.0f) :
    id{id_},
    shapePtr { make_shared<PointCloudT>() },
    isFilled { iF },
    density { d }
{ }

string IPointCloudShape::getId() { return id; }

PointCloudT::Ptr IPointCloudShape::getShape() { return shapePtr; }

void IPointCloudShape::generateShape() { }

void IPointCloudShape::setColor(pcl::RGB color) {
    for (int i=0; i<shapePtr->size(); i++) {
        shapePtr->points[i].r = color.r;
        shapePtr->points[i].g = color.g;
        shapePtr->points[i].b = color.b;
    }
}
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
    pcl::copyPointCloud(tmp, *shapePtr);
}
/******************************************RECTANGLE*******************************************************************/
RectanglePointCloudShape::RectanglePointCloudShape(const string& id, bool iF, float w, float h, float d) :
    IPointCloudShape(id, iF, 1/d),
    width {w},
    height {h}
{}

void RectanglePointCloudShape::generateShape() {
    for (float i = 0; i <= width; i += density) {
        for (float j = 0; j <= height; j += density) {
            if (!isFilled && (i==0 || i>width-density || j==0 || j>height-density)) {
                PointType point {i, j, 0};
                shapePtr->points.push_back(point);
            } else if (isFilled) {
                PointType point {i, j, 0};
                shapePtr->points.push_back(point);
            }
        }
    }
}
/*********************************************CUBOID*******************************************************************/
CuboidPointCloudShape::CuboidPointCloudShape(const string& id, bool iF, float w, float h, float l, float d) :
    IPointCloudShape(id, iF, 1/d),
    width {w},
    height {h},
    length {l}
{}

void CuboidPointCloudShape::generateShape() {
    for (float i = 0; i <= width; i += density) {
        for (float j = 0; j <= height; j += density) {
            for (float k = 0; k <= length; k += density) {
                if (i==0 || i>width-density || j==0 || j>height-density || k==0 || k>length-density) {
                    PointType point {i, j, k};
                    shapePtr->points.push_back(point);
                } else if (isFilled) {
                    PointType point {i, j, k};
                    shapePtr->points.push_back(point);
                }
            }
        }
    }
}
/*********************************************CIRCLE*******************************************************************/
CirclePointCloudShape::CirclePointCloudShape(const string& id, bool iF, float r, float d) :
    IPointCloudShape(id, iF, 1/d),
    radius {r}
{}

void CirclePointCloudShape::generateShape() {
    float degreeIntensity = 360*density / 20;

    if (fmodf(radius, density) > 0.0001)
        radius = density*static_cast<int>(radius / density);

    for (float i = 0; i <= 360.0f; i += degreeIntensity) {
        float rad = i * (M_PI / 180.0f);
        PointType point {radius*cos(rad), radius*sin(rad), 0};
        shapePtr->points.push_back(point);
        if (isFilled) {
            for (float j = 0; j < radius; j += density) {
                PointType point {j*cos(rad), j*sin(rad), 0};
                shapePtr->points.push_back(point);
            }
        }
    }
}
/*********************************************SPHERE*******************************************************************/
SpherePointCloudShape::SpherePointCloudShape(const string& id, bool iF, float r, float d) :
    IPointCloudShape(id,iF,1/d),
    radius {r}
{}

void SpherePointCloudShape::generateShape() {
    float degreeIntensity = 360*density / 20;
    for (float i = 0; i <= 180.0f; i += degreeIntensity) {
        for (float j = 0; j < 360.0f; j += degreeIntensity) {
            float radI = i * (M_PI / 180.0f), radJ = j * (M_PI / 180.0f);
            PointType point {radius*sin(radI)*cos(radJ), radius*sin(radI)*sin(radJ), radius*cos(radI)};
            shapePtr->points.push_back(point);
            if (isFilled) {
                for (float k = 0; k < radius; k += density) {
                    PointType point {k*sin(radI)*cos(radJ), k*sin(radI)*sin(radJ), k*cos(radI)};
                    shapePtr->points.push_back(point);
                }
            }
        }
    }
}
/*********************************************CYLINDER*****************************************************************/
CylinderPointCloudShape::CylinderPointCloudShape(const string& id, bool iF, float r, float h, float d) :
    IPointCloudShape(id,iF,1/d),
    radius {r},
    height {h}
{}

void CylinderPointCloudShape::generateShape() {
    float degreeIntensity = 360*density / 20;

    if (fmodf(radius, density) > 0.0001)
        radius = density*static_cast<int>(radius / density);

    for (float i = 0; i <= 360.0f; i += degreeIntensity) {
        float rad = i * (M_PI / 180.0f);
        for (float j = 0; j <= radius; j += density) {
            PointType point = {j*cos(rad), j*sin(rad), 0};
            shapePtr->points.push_back(point);
            point.z = height;
            shapePtr->points.push_back(point);
        }
    }

    for (float i = 0; i <= 360.0f; i += degreeIntensity) {
        for (float j = density; j < height; j += density) {
            float rad = i * (M_PI / 180.0f);
            PointType point {radius*cos(rad), radius*sin(rad), j};
            shapePtr->points.push_back(point);
            if (isFilled) {
                for (float k = 0; k < radius; k += density) {
                    PointType point {k*cos(rad), k*sin(rad), j};
                    shapePtr->points.push_back(point);
                }
            }
        }
    }
}
/***********************************************CONE*******************************************************************/
ConePointCloudShape::ConePointCloudShape(const string& id, bool iF, float r, float h, float d) :
    IPointCloudShape(id,iF,1/d),
    radius {r},
    height {h}
{}

void ConePointCloudShape::generateShape() {
    float degreeIntensity = 360*density / 20;

    if (fmodf(radius, density) > 0.0001)
        radius = density*static_cast<int>(radius / density);

    if (fmodf(height, density) > 0.0001)
        height = density*static_cast<int>(height / density);

    for (float i = 0; i <= 360.0f; i += degreeIntensity) {
        float rad = i * (M_PI / 180.0f);
        for (float j = 0; j <= radius; j += density) {
            PointType point = {j*cos(rad), j*sin(rad), 0};
            shapePtr->points.push_back(point);
        }
    }

    for (float i = 0; i < 360.0f; i += degreeIntensity)
    {
        float radI = i * (M_PI / 180.0f);
        for (float u = density; u <= height; u += density)
        {
            float x = (height - u) / height * radius * cos(radI);
            float y = (height - u) / height * radius * sin(radI);
            PointType point {x, y, u};
            shapePtr->points.push_back(point);
            if (isFilled) {
                for (float k = 0; k < radius; k += density) {
                    x = (height - u) / height * k * cos(radI);
                    y = (height - u) / height * k * sin(radI);
                    PointType point {x, y, u};
                    shapePtr->points.push_back(point);
                }
            }
        }
    }
}
