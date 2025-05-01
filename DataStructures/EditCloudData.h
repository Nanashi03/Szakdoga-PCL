#ifndef EDITCLOUDDATA_H
#define EDITCLOUDDATA_H

#include <string>

struct EditCloudData
{
    std::string name;

    bool isFilled, areNormalsShown, showFilledEdit, showColorEdit, showDensityEdit;
    float density;
    std::vector<int> rgb, rotation;
    std::vector<float> dim;

    std::vector<std::string> labels;
    std::vector<bool> showLabels;
};

#endif //EDITCLOUDDATA_H