//Vizualization libraries.
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/common/time.h>
#include <pcl/visualization/cloud_viewer.h>

//Cylinder segmentation libraries.
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

class SimpleOpenNIViewer
{
  public:
  SimpleOpenNIViewer () : viewer ("PCL OpenNI Viewer") {}

  void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud)
  {
    if ( !viewer.wasStopped() ) {
      viewer.showCloud (cloud);
    }
  }

  void run ()
  {
    pcl::Grabber* interface = new pcl::OpenNIGrabber();

    boost::function<void (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr&)> f =
    boost::bind (&SimpleOpenNIViewer::cloud_cb_, this, _1);

    interface->registerCallback (f);

    interface->start ();

    while (!viewer.wasStopped())
    {
      sleep (1);
    }

    interface->stop ();
  }

  pcl::visualization::CloudViewer viewer;

};

int main ()
{
  SimpleOpenNIViewer v;
  v.run ();
  return 0;
}
