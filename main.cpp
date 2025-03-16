#include <iostream>
#include <pcl/point_types.h>
#include  "Model/PointCloudShapes.h"

int main (int argc, char** argv) {
  std::string s = "imported_1";
  ImportedPointCloudShape<pcl::PointXYZ> imported{s};
  imported.generateShape("test.pcd");

  return 0;
}