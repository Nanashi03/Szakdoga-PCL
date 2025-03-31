//asdasd
// Created by kristof on 2025.03.16..
//

#include "Controller.h"

Controller::Controller() {
    Viewer::cloudSelectedEventListener = [this](const string& name) { this->selectCloud(name); };
    Viewer::selectedCloudTranslateLeftEventListener = [this](int x, int y, int z) { this->translate(x,y,z); };
}

void Controller::start() {
    viewer.run();
}

void Controller::tmp() {
    return;
}

void Controller::selectCloud(const string& cloudName) {
    cout << "I SELECTED A CLOUD: " << cloudName << endl;

    if (model.isCloudSelected())
        viewer.updateCloud(model.getSelectedCloudName(), model.deSelectCloud());

    viewer.updateCloud(cloudName, model.selectCloud(cloudName));
}

void Controller::importCloud(const string& id, const string& filePath) {
    ImportedPointCloudShape cloud{id, filePath};
    cloud.generateShape();

    model.addCloud(cloud);
    viewer.addCloud(cloud.getId(), cloud.getShape());
}

void Controller::generateRectangle(const string& id, bool isFilled, float width, float height, float density) {
    RectanglePointCloudShape cloud {id, isFilled, width, height, density};
    cloud.generateShape();

    model.addCloud(cloud);
    viewer.addCloud(cloud.getId(), cloud.getShape());
}

void Controller::generateCircle(const string& id, bool isFilled, float radius, float density) {
    CirclePointCloudShape cloud {id, isFilled, radius, density};
    cloud.generateShape();

    model.addCloud(cloud);
    viewer.addCloud(cloud.getId(), cloud.getShape());
}

void Controller::generateCube(const string & id, bool isFilled, float width, float height, float length, float density) {
    CuboidPointCloudShape cloud {id, isFilled, width, height, length, density};
    cloud.generateShape();

    model.addCloud(cloud);
    viewer.addCloud(cloud.getId(), cloud.getShape());
}

void Controller::generateSphere(const string& id, bool isFilled, float radius, float density) {
    SpherePointCloudShape cloud {id, isFilled, radius, density};
    cloud.generateShape();

    model.addCloud(cloud);
    viewer.addCloud(cloud.getId(), cloud.getShape());
}

void Controller::generateCylinder(const string& id, bool isFilled, float radius, float height, float density) {
    CylinderPointCloudShape cloud {id, isFilled, radius, height, density};
    cloud.generateShape();

    model.addCloud(cloud);
    viewer.addCloud(cloud.getId(), cloud.getShape());
}

void Controller::generateCone(const string& id, bool isFilled, float radius, float height, float density) {
    ConePointCloudShape cloud {id, isFilled, radius, height, density};
    cloud.generateShape();

    model.addCloud(cloud);
    viewer.addCloud(cloud.getId(), cloud.getShape());
}

void Controller::translate(int x, int y, int z) {
    if (model.isCloudSelected())
        viewer.updateCloud(model.getSelectedCloudName(), model.translateSelectedCloud(x,y,z));
}

void Controller::rotate(float angle, char axis) {
    if (model.isCloudSelected())
        viewer.updateCloud(model.getSelectedCloudName(), model.rotateSelectedCloud(angle, axis));
}



