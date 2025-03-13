#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace std;

int main() {
    cout << "Hello World!" << endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
fskgnf
dfnjgkjnfdg

    if (pcl::io::loadPCDFile<pcl::PointXYZ> ("test.pcd", *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
        return (-1);
    }
    cout << "Loaded "
              << cloud->width * cloud->height
              << " data points from test.pcd with the following fields: "
              << endl;
    for (const auto& point: *cloud)
        cout << "    " << point.x
                  << " "    << point.y
                  << " "    << point.z << endl;

    return (0);
}
/* \author Geoffrey Biggs */
