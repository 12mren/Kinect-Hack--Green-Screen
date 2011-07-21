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

// For the segmentation
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include "timer_eb.h"
using namespace cv;
using namespace pcl;

typedef pcl::PointXYZRGB PointT;
/* EB::TimerList tm;
 tm["cylseg"] = new EB::Timer("cylseg function"); 
  tm["cloudcb"] = new EB::Timer("cloudcb function");
 tm["run"]=new EB::Timer("run function");
tm["initcv"]=new EB::Timer("initCV function"); 
*/
   
//Segments the person in Point cloud "cloud." Returns "cloud_cylinder."
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
  seg.setDistanceThreshold (0.8);
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
//tm["cylseg"]->stop();
  return cloud_cylinder;

}

//Visualizes both point clouds.
class SimpleOpenNIViewer
{

  public:
  SimpleOpenNIViewer () : viewer ("PCL OpenNI Viewer") {}

  //Call back function. Displays both image point cloud and kinect point cloud.
  void cloud_cb_ (const PointCloud<PointT>::ConstPtr &cloud ) {
// tm["cloudcb"]->start();
    //Checks to see if this is the first instance of the call back function. If so it sets z values of image to that of the farthest depth fromt the Kinect.
     if (!firstcb){  
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
     firstcb=true;
  }
  
   //Visualizes both point clouds.
   if ( !viewer.wasStopped() ) {
      viewer.showCloud (cylSeg(cloud), "cloud");
      viewer.showCloud (cloud_filtered, "cloud_filtered");
   }
// tm["cloudcb"]->stop();

 }


  //Loads the image, edits image cloud, and runs visualization.
  void run ( ) {
// tm["run"]->start();
   //"firstcb" holds a boolean to determine if it is the first instance of the call back function.
    firstcb=false;
   
    initCV();
    
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
       row = j/img.cols;
       col = j%img.cols;
       Vec3b color = img.at<Vec3b>(row,col);
       cloud2->points[j].x=col-img.cols/2;
       cloud2->points[j].y=row-img.rows/2;
       cloud2->points[j].z=50;
       r = color[2];
       g = color[1];
       b = color[0];
       rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
       cloud2->points[j].rgb = *reinterpret_cast<float*>(&rgb);
    }
   //Downsamples the image.
     pcl::VoxelGrid<PointT> sor;
     sor.setInputCloud (cloud2);
     leafsize= (img.rows*img.cols)/(563700.);
     sor.setLeafSize (leafsize,leafsize,leafsize);
     sor.filter (*cloud_filtered);

   //Resizes the image.
     for (size_t j=0; j<cloud_filtered->size(); j++){
      cloud_filtered->points[j].x/=img.rows*.3;
      cloud_filtered->points[j].y/=img.rows*.3;
  }

  //Creates cloud and visualizes it.
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
 //tm["run"]->stop();

  }

  visualization::CloudViewer viewer;


  private:
  bool firstcb;
  int row;
  int col;
  uint8_t r,g,b;
  uint32_t rgb;
  double leafsize;


  //Loads image using OpenCV. Asks user for image path.
   void initCV() {
 //tm["initcv"]->start();

    string imgpath ="";
    cout<<"Please enter an image path."<<endl;
    getline(cin, imgpath);
    img = imread(imgpath);
    if(img.empty()) {
      printf("Could not load image file.");
      exit(0);
    } 
 //tm["initcv"]->stop();

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
  //tm.writeToFile("alt_timer_log");
  return(0);
}
                     

