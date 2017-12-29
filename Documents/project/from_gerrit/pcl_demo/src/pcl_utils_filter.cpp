/*
 * pcl_filter.cpp
 *
 *  Created on: 2017年12月15日
 *      Author: tangzm
 */

#include "pcl_utils_filter.h"

using namespace std;


void applyFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, std::vector<filter_proc_t>& filters){
	// Create the filtering object
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

	for (vector<filter_proc_t>::iterator it=filters.begin(); it!=filters.end(); it++){
		filter_type_t current_type = it->type;
		filter_param_t current_param = it->param;

		if (linear_filter == current_type){
			pcl::PassThrough<pcl::PointXYZ> pass;
			pass.setInputCloud (cloud);

			linear_filter_param_t param = current_param.linear_filter;

			double actual_min = (param.min < param.max)?param.min:param.max;
			double actual_max = (param.min > param.max)?param.min:param.max;

			pass.setFilterFieldName (param.field_name);
			pass.setFilterLimits (actual_min, actual_max);

			/* If setFilterLimitsNegative is set true, the pixels out side of the range would pass */
			pass.setFilterLimitsNegative (param.min > param.max);

			pass.filter (*cloud_filtered);
			cloud_filtered.swap(cloud);
		}
		else if(down_sample == current_type){
			pcl::VoxelGrid<pcl::PointXYZ> sor;
			sor.setInputCloud (cloud);

			down_sample_param_t param = current_param.down_sample;

			/* Each voxel as a "cube", which x=y=z*/
			sor.setLeafSize (param.voxel_size, param.voxel_size, param.voxel_size);

			sor.filter (*cloud_filtered);
			cloud_filtered.swap(cloud);
		}
		else if(stats_denoise == current_type){
			pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
			sor.setInputCloud (cloud);

			denoise_param_t param = current_param.denoise;

			sor.setMeanK (param.neighbours);
			sor.setStddevMulThresh (param.std_err);

			sor.filter (*cloud_filtered);
			cloud_filtered.swap(cloud);
		}
		else if(projection == current_type){
			  // We project it onto a z=100 surface (defined by ax+by+cz+d=0)
			  pcl::ProjectInliers<pcl::PointXYZ> proj;
			  proj.setInputCloud (cloud);

			  projection_param_t param = current_param.projection;

			  /* Defined by ax+by+cz+w = 0 */
			  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
			  coefficients->values.resize (4);
			  coefficients->values[0] = param.a;
			  coefficients->values[1] = param.b;
			  coefficients->values[2] = param.c;
			  coefficients->values[3] = param.w;

			  // Create the filtering object
			  proj.setModelType (pcl::SACMODEL_PLANE);
			  proj.setModelCoefficients (coefficients);

			  proj.filter (*cloud_filtered);
			  cloud_filtered.swap(cloud);
		}
		else if(radius_filter == current_type)
		{
			pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
			outrem.setInputCloud(cloud);

			radius_param_t param = current_param.radius_filter;
    		outrem.setRadiusSearch(param.radius);
    		outrem.setMinNeighborsInRadius (param.minNeighbours);
    		// apply filter
    		outrem.filter (*cloud_filtered);
			cloud_filtered.swap(cloud);
		}
	}
}
