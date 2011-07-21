#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>


#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
using namespace pcl;
typedef pcl::PointXYZ PointT;

   
  void estimateNormals (const pcl::PointCloud<PointT>::ConstPtr& cloud){
       NormalEstimation<PointXYZ, Normal> ne;
   ne.setInputCloud (cloud);
   KdTreeFLANN<PointXYZ>::Ptr tree (new KdTreeFLANN<PointXYZ> ());
   ne.setSearchMethod (tree);
   PointCloud<Normal>::Ptr cloud_normals (new PointCloud<Normal>);
   ne.setRadiusSearch (0.03);
   ne.compute (*cloud_normals);
 pcl::PCDWriter writer;

writer.write ("../data/table_normals.pcd", *cloud_normals,false);
                    }
  int main (int argc, char**argv)
  {
    pcl::PCDReader reader;
   
   
    pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
    reader.read("./table_scene_mug_stereo_textured_cylinder.pcd", *cloud);
    
    estimateNormals(cloud);
   
  }
