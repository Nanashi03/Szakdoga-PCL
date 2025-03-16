//
// Created by kristof on 2025.03.16..
//

#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "../Model/Model.h"

template<typename PointT>
class Controller {
    private:
        Model<PointT> model;
    public:
        Controller();
        void importCloud(const std::string&);
};

#endif //CONTROLLER_H
