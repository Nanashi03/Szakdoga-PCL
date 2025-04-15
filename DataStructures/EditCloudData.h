//
// Created by kristof on 4/14/25.
//

#ifndef EDITCLOUDDATA_H
#define EDITCLOUDDATA_H

#include <string>

struct EditCloudData
{
    std::string name;
    bool isFilled, areNormalsPresent;
    std::vector<int> rgb, rotation;

    std::vector<float> dim;
    float density;
    std::vector<std::string> labels;

    std::vector<bool> showLabels;
    bool showColorEdit;
    bool showFilledEdit;
    bool showDensityEdit;
};

#endif //EDITCLOUDDATA_H
