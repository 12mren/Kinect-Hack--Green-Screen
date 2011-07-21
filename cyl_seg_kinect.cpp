// For grabbing the point clouds
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>

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

// For visualizing the point clouds
#include <pcl/visualization/cloud_viewer.h>

typedef pcl::PointXYZ PointT;

using namespace pcl;
void PointCloud<Normal>::ConstPtr estimateNormals (const pcl::PointCloud<PointT>::ConstPtr& cloud){
   
    // Create the normal estimation class, and pass the input dataset to it
     NormalEstimation<PointXYZ, Normal> ne;
     ne.setInputCloud (cloud);
    
    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
     KdTreeFLANN<PointXYZ>::Ptr tree (new KdTreeFLANN<PointXYZ> ());
     ne.setSearchMethod (tree);
    
    // Output datasets
    PointCloud<Normal>::Ptr cloud_normals (new PointCloud<Normal>);
    
    // Use all neighbors in a sphere of radius 3cm
    ne.setRadiusSearch (0.03);
    
   // Compute the features
      ne.compute (*cloud_normals);
    
    // cloud_normals->points.size () should have the same size as the input cloud->points.size ()*
         }

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


  void cloud_cb_ (const PointCloud<PointT>::ConstPtr &cloud)
  {
    if ( !viewer.wasStopped() ) {
     // viewer.showCloud (estimateNormals(cloud)); 
     // viewer.showCloud ( cylSeg( cloud ), "Segmented Cylinder" );
    }
  }

  void run ()
  {
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

};

    
int main()
{
  SimpleOpenNIViewer v;
  v.run ();
  

  return 0;
}
