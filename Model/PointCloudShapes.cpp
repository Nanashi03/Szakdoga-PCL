//asd
// Created by kristof on 2025.03.23..
//

#include "PointCloudShapes.h"

#include <oneapi/tbb/task_arena.h>

IPointCloudShape::IPointCloudShape(const string& id_, bool iF = false, float d = 1.0f) :
    id{id_},
    shapePtr { make_shared<PointCloudT>() },
    isFilled { iF },
    areNormalsPresent { false },
    density { d }
{ }

void IPointCloudShape::calculateNormals()
{
    areNormalsPresent = true;

    for (int i=0; i<shapePtr->points.size(); i++)
    {
        cout << shapePtr->points[i].x << " " << shapePtr->points[i].y << " " << shapePtr->points[i].z << endl;
    }

    cout << endl;

    pcl::NormalEstimation<PointType, PointType> ne;
    ne.setInputCloud (shapePtr);
    pcl::search::KdTree<PointType>::Ptr tree { new pcl::search::KdTree<PointType> () };
    ne.setSearchMethod (tree);
    ne.setRadiusSearch (0.05);
    ne.compute (*shapePtr);

    for (int i=0; i<shapePtr->size(); ++i)
    {
        cout << shapePtr->points[i].normal_x << " " << shapePtr->points[i].normal_y << " " << shapePtr->points[i].normal_z << endl;
    }
}

bool IPointCloudShape::getAreNormalsPresent() { return areNormalsPresent; }

string IPointCloudShape::getId() { return id; }

string IPointCloudShape::getNormalId() { return id + "_normals"; }

PointCloudT::Ptr IPointCloudShape::getShape() { return shapePtr; }

void IPointCloudShape::generateShape() { }

void IPointCloudShape::setColor(pcl::RGB color) {
    for (int i=0; i<shapePtr->size(); i++) {
        shapePtr->points[i].r = color.r;
        shapePtr->points[i].g = color.g;
        shapePtr->points[i].b = color.b;
    }
}

void IPointCloudShape::setShape(PointCloudT::Ptr shape) {
    this->shapePtr = shape;
}
/*********************************************IMPORTED_CLOUD***********************************************************/
ImportedPointCloudShape::ImportedPointCloudShape(const string& id, const string& filePath) :
    IPointCloudShape(id),
    filePath {filePath}
{}

void ImportedPointCloudShape::generateShape() {
    pcl::PCLPointCloud2 cloud2;
    pcl::PCDReader reader;
    if (reader.readHeader(filePath, cloud2) != 0) {
        PCL_ERROR("Couldn't read file test_pcd.pcd \n");
    }
    bool constainRGB { false };
    bool constainNormals {false};
    for (auto field : cloud2.fields) {
        if (field.name == "rgb") constainRGB = true;
        if (field.name == "normal_x") constainNormals = true;
    }
    
    if (constainRGB && constainNormals) {
        pcl::io::loadPCDFile<PointType>(filePath, *shapePtr);
    } else if (constainRGB) {
        pcl::PointCloud<pcl::PointXYZRGB> tmp;
        pcl::io::loadPCDFile<pcl::PointXYZRGB>(filePath, tmp);
        pcl::copyPointCloud(tmp, *shapePtr);
    } else {
        cout << "IM HERE" << endl;
        pcl::PointCloud<pcl::PointXYZ> tmp;
        pcl::io::loadPCDFile<pcl::PointXYZ> (filePath, tmp);
        pcl::copyPointCloud(tmp, *shapePtr);
        setColor({ 255, 255, 255 });
    }
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
                PointType point {i, j, 0, 255, 255, 255};
                shapePtr->points.push_back(point);
            } else if (isFilled) {
                PointType point {i, j, 0, 255, 255, 255};
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
                    PointType point {i, j, k, 255, 255, 255};
                    shapePtr->points.push_back(point);
                } else if (isFilled) {
                    PointType point {i, j, k, 255, 255, 255};
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
        PointType point {radius*cos(rad), radius*sin(rad), 0, 255, 255, 255};
        shapePtr->points.push_back(point);
        if (isFilled) {
            for (float j = 0; j < radius; j += density) {
                PointType point {j*cos(rad), j*sin(rad), 0, 255, 255, 255};
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
            PointType point {radius*sin(radI)*cos(radJ), radius*sin(radI)*sin(radJ), radius*cos(radI), 255, 255, 255};
            shapePtr->points.push_back(point);
            if (isFilled) {
                for (float k = 0; k < radius; k += density) {
                    PointType point {k*sin(radI)*cos(radJ), k*sin(radI)*sin(radJ), k*cos(radI), 255, 255, 255};
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
            PointType point = {j*cos(rad), j*sin(rad), 0, 255, 255, 255};
            shapePtr->points.push_back(point);
            point.z = height;
            shapePtr->points.push_back(point);
        }
    }

    for (float i = 0; i <= 360.0f; i += degreeIntensity) {
        for (float j = density; j < height; j += density) {
            float rad = i * (M_PI / 180.0f);
            PointType point {radius*cos(rad), radius*sin(rad), j, 255, 255, 255};
            shapePtr->points.push_back(point);
            if (isFilled) {
                for (float k = 0; k < radius; k += density) {
                    PointType point {k*cos(rad), k*sin(rad), j, 255, 255, 255};
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
            PointType point = {j*cos(rad), j*sin(rad), 0, 255, 255, 255};
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
                    PointType point {x, y, u, 255, 255, 255};
                    shapePtr->points.push_back(point);
                }
            }
        }
    }
}
