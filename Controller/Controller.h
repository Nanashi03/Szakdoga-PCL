//asdasd
// Created by kristof on 2025.03.16..
//

#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "../Model/Model.h"
#include "../View/Viewer.h"

class Controller {
private:
    Model model;
    Viewer viewer;
public:
    Controller();
    void start();
    void selectCloud(const string&);

    void importCloud(const string&);
    void generateRectangle(const string&, float, float, float);
    void generateCircle(const string&, float, float);
    void generateCube(const string&, float, float, float, float);
    void generateSphere(const string&, float, float);

    void tmp();
};

#endif //CONTROLLER_H
