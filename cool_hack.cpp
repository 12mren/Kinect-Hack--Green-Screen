#include <iostream>
#include <pcl/io/openni_grabber.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <cv.h>
#include <highgui.h> 
#include <cvaux.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <pcl/visualization/cloud_viewer.h>
// For grabbing the point clouds
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
//For voxel grid.
#include <pcl/filters/voxel_grid.h>

using namespace cv;
using namespace pcl;

typedef pcl::PointXYZRGB PointT;

PointCloud<PointT>::ConstPtr cylSeg( const PointCloud<PointT>::ConstPtr& cl ) {

  // All the objects needed
  PCDReader reader;
  PassThrough<PointT> pass;
  NormalEstimation<PointT, Normal> ne;
  SACSegmentationFromNormals<PointT, Normal> seg; 
  PCDWriter writer;
  ExtractIndices<PointT> extract;
  ExtractIndices<pcl::Normal> extract_normals;
  KdTreeFLANN<PointT>::Ptr tree (new KdTreeFLANN<PointT> ());

  // Datasets
  PointCloud<PointT>::Ptr cloud_filtered (new PointCloud<PointT>);
  PointCloud<pcl::Normal>::Ptr cloud_normals (new PointCloud<pcl::Normal>);
  ModelCoefficients::Ptr coefficients_plane (new ModelCoefficients), coefficients_cylinder (new ModelCoefficients);
  PointIndices::Ptr inliers_plane (new PointIndices), inliers_cylinder (new PointIndices);

  // Build a passthrough filter to remove spurious NaNs
  pass.setInputCloud (cl);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (1, 2);
  pass.filter (*cloud_filtered);

  // Estimate point normals
  ne.setSearchMethod (tree);
  ne.setInputCloud (cloud_filtered);
  ne.setKSearch (50);
  ne.compute (*cloud_normals);

  // Create the segmentation object for cylinder segmentation and set all the parameters
  seg.setOptimizeCoefficients (true);
  seg.setModelType (SACMODEL_CYLINDER);
  seg.setMethodType (SAC_RANSAC);
  seg.setNormalDistanceWeight (0.1);
  seg.setMaxIterations (1000);
  seg.setDistanceThreshold (0.3);
  seg.setRadiusLimits (0, .1);//The larger the more fuzzy.
  seg.setInputCloud (cloud_filtered);
  seg.setInputNormals (cloud_normals);

  // Obtain the cylinder inliers and coefficients
  seg.segment (*inliers_cylinder, *coefficients_cylinder);

  extract.setInputCloud (cloud_filtered);
  extract.setIndices (inliers_cylinder);
  extract.setNegative (false);
  PointCloud<PointT>::Ptr cloud_cylinder (new PointCloud<PointT> ());
  extract.filter (*cloud_cylinder);

  return cloud_cylinder;
}

class SimpleOpenNIViewer
{
  public:
  SimpleOpenNIViewer () : viewer ("PCL OpenNI Viewer") {}

  //Displays both image point cloud and kinect point cloud.
  void cloud_cb_ (const PointCloud<PointT>::ConstPtr &cloud ) {

   //Computes the maximum Z  distance captured by the kinect.
   double max =0;
   for (size_t j=0; j<cloud->size(); j++){
      if (cloud->points[j].z>max){
      max = cloud->points[j].z;
    }
  }
 
   //Sets the Z value of the image to the maximum Z distance captured by the Kinect.
   for (size_t j=0; j<cloud_filtered->size();j++){
     cloud_filtered->points[j].z=max;
   }

   //Visualizes both point clouds.
   if ( !viewer.wasStopped() ) {
      viewer.showCloud (cylSeg(cloud), "cloud");
      viewer.showCloud (cloud_filtered, "cloud_filtered");
  }
 }

  //Loads the image, edits image cloud, and runs visualization.
  void run ( ) {

    initCV();
    Grabber* interface = new OpenNIGrabber();

    boost::function<void (const PointCloud<PointT>::ConstPtr&)> f =
    boost::bind (&SimpleOpenNIViewer::cloud_cb_, this, _1);

    interface->registerCallback (f);

    interface->start ();

    while (!viewer.wasStopped())
    {
      sleep (1);
    }
    interface->stop ();
  }

  visualization::CloudViewer viewer;


  private:

//Loads image, edits point cloud for image (cloud 2), downsamples data, and shrinks image. Stores new point cloud in cloud_filtered.
  void initCV() {
    //Loads image using OpenCV.
    img = imread("../data/vangogh.jpg");
    if(img.empty()) {
      printf("Could not load image file: %s\n", "vangogh.jpg");
      exit(0);
    } 

    //Initializes 2 point clouds. An unfiltered image and a filtered image.
    cloud2 = PointCloud<PointT>::Ptr(new PointCloud<PointT>);
    cloud_filtered= PointCloud<PointT>::Ptr(new PointCloud<PointT>);   
   
    //Sets the size of the point cloud to be that of the image.
    cloud2->width = img.cols;
    cloud2->height = img.rows;
    cloud2->points.resize(cloud2->width*cloud2->height);

    //Assigns the points in the image (intensity and color) to that of the point cloud).
    for (size_t j=0; j < img.rows*img.cols; j++)
    {
       int row = j/img.cols;
       int col = j%img.cols;
       Vec3b color = img.at<Vec3b>(row,col);
       cloud2->points[j].x=col-1711.1/2;
       cloud2->points[j].y=row-750;
       cloud2->points[j].z=50;
       uint8_t r = color[2], g = color[1], b = color[0];
       uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
       cloud2->points[j].rgb = *reinterpret_cast<float*>(&rgb);
    }
  //Downsamples the image.
   pcl::VoxelGrid<PointT> sor;
   sor.setInputCloud (cloud2);
   sor.setLeafSize (5, 5, 5);
   sor.filter (*cloud_filtered);

   //Resizes the image.
     for (size_t j=0; j<cloud_filtered->size(); j++){
      cloud_filtered->points[j].x/=img.rows*.3;
      cloud_filtered->points[j].y/=img.rows*.3;
  }
  }

  //Private variables: imported image, unfiltered point cloud for image, filtered point cloud for image.
  Mat img;
  PointCloud<PointT>::Ptr cloud2;
  PointCloud<PointT>::Ptr cloud_filtered;

};




//Main program.
int  main (int argc, char** argv)
{

  SimpleOpenNIViewer v;
  v.run( );
  return(0);
}
                     
