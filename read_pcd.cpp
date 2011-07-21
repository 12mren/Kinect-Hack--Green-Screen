#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

using namespace pcl;

int
main (int argc, char** argv)
{
    PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ>);
    std::string file_name ="../data/Vangogh.pcd";
      if (io::loadPCDFile<PointXYZ> (file_name, *cloud) == -1) //* load the file
          {
                PCL_ERROR ("Couldn't read file \n");
                    return (-1);
                      }
        std::cerr << "Loaded ";
      visualization::CloudViewer viewer("Simple Cloud Viewer");
      viewer.showCloud(cloud);
      while (!viewer.wasStopped())
      {
      }
      return (0);
}
