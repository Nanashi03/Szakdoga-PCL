//asdasd
// Created by kristof on 2025.03.16..
//

#include "Controller.h"

Controller::Controller() {
    Viewer::cloudSelectedEventListener = [this](const string& name) { this->selectCloud(name); };
}

void Controller::start() {
    viewer.run();
}

void Controller::tmp() {
    return;
}

void Controller::selectCloud(const string& cloudName) {
    cout << "I SELECTED A CLOUD: " << cloudName << endl;

    viewer.updateCloud(cloudName, model.selectCloud(cloudName));
}

void Controller::importCloud(const std::string& filePath) {
    ImportedPointCloudShape cloud{"id1", filePath};
    cloud.generateShape();

    model.addCloud(cloud);
    viewer.addCloud(cloud.getId(), cloud.getShape());
}

void Controller::generateRectangle(const std::string& id, bool isFilled, float width, float height, float intensity) {
    RectanglePointCloudShape cloud {id, isFilled, width, height, intensity};
    cloud.generateShape();

    model.addCloud(cloud);
    viewer.addCloud(cloud.getId(), cloud.getShape());
}

void Controller::generateCircle(const std::string& id, bool isFilled, float radius, float intensity) {
    CirclePointCloudShape cloud {id, isFilled, radius, intensity};
    cloud.generateShape();

    model.addCloud(cloud);
    viewer.addCloud(cloud.getId(), cloud.getShape());
}

void Controller::generateCube(const std::string & id, bool isFilled, float width, float height, float length, float intensity) {
    CuboidPointCloudShape cloud {id, isFilled, width, height, length, intensity};
    cloud.generateShape();

    model.addCloud(cloud);
    viewer.addCloud(cloud.getId(), cloud.getShape());
}

void Controller::generateSphere(const std::string& id, bool isFilled, float radius, float intensity) {
    SpherePointCloudShape cloud {id, isFilled, radius, intensity};
    cloud.generateShape();

    model.addCloud(cloud);
    viewer.addCloud(cloud.getId(), cloud.getShape());
}

void Controller::generateCylinder(const std::string& id, bool isFilled, float radius, float height, float intensity) {
    CylinderPointCloudShape cloud {id, isFilled, radius, height, intensity};
    cloud.generateShape();

    model.addCloud(cloud);
    viewer.addCloud(cloud.getId(), cloud.getShape());
}

void Controller::generateCone(const string& id, bool isFilled, float radius, float height, float intensity) {
    ConePointCloudShape cloud {id, isFilled, radius, height, intensity};
    cloud.generateShape();

    model.addCloud(cloud);
    viewer.addCloud(cloud.getId(), cloud.getShape());
}



