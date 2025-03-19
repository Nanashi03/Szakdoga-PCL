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

