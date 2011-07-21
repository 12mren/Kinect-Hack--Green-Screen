#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

using namespace pcl;

int
main (int argc, char** argv)
{
    PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ>);
    PointCloud<PointXYZ>::Ptr cloud2 (new PointCloud<PointXYZ>);
    std::string file_name ="../data/table_scene_lms400.pcd";
      if (io::loadPCDFile<PointXYZ> (file_name, *cloud) == -1) //* load the file
          {
                PCL_ERROR ("Couldn't read file \n");
                    return (-1);
                      }
        std::cerr << "Loaded ";
       /* for (size_t i=0; i<cloud->points.size(); i++)
        {
          cloud->points[i].z=cloud->points[i].z+10;
        }*/
      std::string file_name2 ="../data/table_scene_mug_stereo_textured_cylinder.pcd";
      if (io::loadPCDFile<PointXYZ> (file_name2, *cloud2) == -1) //* load the file
          {
                PCL_ERROR ("Couldn't read file \n");
                    return (-1);
                      }
        std::cerr << "Loaded ";
      visualization::CloudViewer viewer("Simple Cloud Viewer");
      while (!viewer.wasStopped())
      {
      viewer.showCloud(cloud,"cloud");
  
     // sleep (200);
    
      viewer.showCloud(cloud2,"cloud2");
      }
      return (0);
}
