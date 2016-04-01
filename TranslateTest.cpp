#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/histogram_visualizer.h>
#include <pcl/point_types.h>
#include <pcl/features/pfh.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/vfh.h>
#include <pcl/visualization/pcl_plotter.h>
#include <pcl/visualization/image_viewer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/point_picking_event.h>
#include <vector>
#include <pcl/features/pfh.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/area_picking_event.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
 #include <pcl/common/common.h>
//#include "labeled_data_class.h"
bool histo=false;
using namespace std;

void compute_surface_normals(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud,pcl::PointCloud<pcl::Normal>::Ptr &normals_out)
{
		pcl::NormalEstimation<pcl::PointXYZRGBA, pcl::Normal> ne;
		pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZRGBA>);
		ne.setSearchMethod(kdtree);
		ne.setRadiusSearch(0.02); 
	    ne.setInputCloud(cloud); 

    Eigen::Vector4f centroid;
    pcl::compute3DCentroid (*cloud, centroid);
    ne.setViewPoint (centroid[0], centroid[1], centroid[2]);
	ne.compute(*normals_out);

}

void visualize_normals(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,const pcl::PointCloud<pcl::Normal>::Ptr normals,pcl::PointCloud<pcl::FPFHSignature33>::Ptr &fpfhs_out)
 {

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->initCameraParameters ();

  int v1(0); 
		viewer->createViewPort (0.0, 0.0, 0.5, 1.0, v1);
		viewer->setBackgroundColor (0, 0, 0, v1);
	viewer->addPointCloud(cloud,"cloudID",v1); 
		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloudID");
		viewer->resetCameraViewpoint("cloudID");
		viewer->addText ("Visualize PointCloud ", 10, 10, "v1 text", v1);
        int v2(0); 

		viewer->createViewPort (0.5, 0.0, 1.0, 1.0, v2);
       viewer->setBackgroundColor (0.3, 0.3, 0.3, v2);
		viewer->addPointCloudNormals<pcl::PointXYZRGBA, pcl::Normal> (cloud, normals,10,0.03,"normalsID2", v2);
		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "normalsID2");
		viewer->resetCameraViewpoint("normalsID2");
		viewer->addText ("Visualize The Normals2 ", 10, 10, "v2 text", v2);

		viewer->spin (); 

}

int main(int argc, char** argv)
{ 
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs_out (new pcl::PointCloud<pcl::FPFHSignature33> ());

if (pcl::io::loadPCDFile<pcl::PointXYZRGBA>(argv[1], *cloud) != 0) 
                      return -1;
     
compute_surface_normals(cloud,normals);

std::cout << "point1 size " << cloud->points.size() << std::endl;

visualize_normals(cloud,normals,fpfhs_out);
return (0);
 
	}