//
// Created by kristof on 2025.03.16..
//

#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "../Model/Model.h"
#include "../Model/PointCloudShapes.h"
#include "../View/Viewer.h"

class Controller {
private:
    Model model;
    Viewer viewer;
public:
    Controller();
    void start();
    void importCloud(const std::string&);
};

#endif //CONTROLLER_H
