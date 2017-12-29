

#include "pcl_utils_viewer.h"

using namespace std;


pcl::visualization::PCLVisualizer::Ptr simpleVis (const point_cloud_holder_t& holder)
{
	/*Create a viewer*/
	pcl::visualization::PCLVisualizer::Ptr  viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));

	/*Set background to pure black*/
	viewer->setBackgroundColor (0, 0, 0);
	viewer->addCoordinateSystem (0.1, 0, 0, 0);


	if(NULL != holder.pXYZ){
		viewer->addPointCloud<pcl::PointXYZ> (holder.pXYZ, "sample cloud");
	}
	else if(NULL != holder.pXYZRGB){
		pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(holder.pXYZRGB);
		viewer->addPointCloud<pcl::PointXYZRGB> (holder.pXYZRGB, rgb, "sample cloud");
	}

	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");

	/* We look from [0,0,-0.5], to [0,0,0] ,
	 * The center of rotating will be 0,0,-0.5, and user is able to enlarge/decrease fov with zoom*/
	viewer->setCameraPosition(0,0,-0.5,0,0,0,0,0,0);

	return (viewer);
}
