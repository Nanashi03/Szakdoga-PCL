//
// Created by kristof on 2025.03.16..
//

#include "Controller.h"

Controller::Controller() {}

void Controller::start() {
    viewer.run();
}

void Controller::importCloud(const std::string& filePath) {
    ImportedPointCloudShape cloud{"id1", filePath};
    cloud.generateShape();

    model.addCloud(cloud);
    viewer.addCloud(cloud.getId(), cloud.getShape());
}

void Controller::generateRectangle(const std::string& id, float width, float height, float intensity) {
    RectanglePointCloudShape cloud {id, width, height, intensity};
    cloud.generateShape();

    model.addCloud(cloud);
    viewer.addCloud(cloud.getId(), cloud.getShape());
}

void Controller::generateCircle(const std::string& id, float radius, float intensity) {
    CirclePointCloudShape cloud {id, radius, intensity};
    cloud.generateShape();

    model.addCloud(cloud);
    viewer.addCloud(cloud.getId(), cloud.getShape());
}

void Controller::generateCube(const std::string & id, float width, float height, float length, float intensity) {
    CuboidPointCloudShape cloud {id, width, height, length, intensity};
    cloud.generateShape();

    model.addCloud(cloud);
    viewer.addCloud(cloud.getId(), cloud.getShape());
}

void Controller::generateSphere(const std::string& id, float radius, float intensity) {
    SpherePointCloudShape cloud {id, radius, intensity};
    cloud.generateShape();

    model.addCloud(cloud);
    viewer.addCloud(cloud.getId(), cloud.getShape());
}


