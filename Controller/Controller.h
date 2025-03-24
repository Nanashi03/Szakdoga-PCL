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
    void generateRectangle(const std::string&, float, float, float);
    void generateCircle(const std::string&, float, float);
    void generateCube(const std::string&, float, float, float, float);
    void generateSphere(const std::string&, float, float);
};

#endif //CONTROLLER_H
