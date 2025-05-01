#ifndef BOUNDINGBOXDATA_H
#define BOUNDINGBOXDATA_H

struct BoundingBoxData
{
    std::string NAME = "BBOX";
    float minX, minY, minZ;
    float maxX, maxY, maxZ;
};

#endif //BOUNDINGBOXDATA_H