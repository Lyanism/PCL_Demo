/*
 * pcl_utils_segment.h
 *
 *  Created on: 2017年12月18日
 *      Author: tangzm
 */

#include <iostream>
#include <fstream>
#include <stdio.h>

#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/conditional_removal.h>

#include <pcl/ModelCoefficients.h>

#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/min_cut_segmentation.h>
#include <pcl/features/don.h>






#ifndef PCL_UTILS_SEGMENT_H_
#define PCL_UTILS_SEGMENT_H_

typedef enum {
	euclidien,
	region_growth,
	min_cut,
	difference_of_normals,
}segment_type_t;

typedef struct {
	double distance;
	int  min_points;
} euclidien_param_t;

typedef struct {
	int norm_neighbour;
	int min_points;
	int neighbour;
	double theta_thres;
	double curvature_thres;
} region_param_t;

typedef struct {
	double x;
	double y;
	double z;
	double sigma;
	double radius;
	int neighbour;
	double weight;
} mincut_param_t;

typedef struct {
	double scale1;
	double scale2;
	double threshold;
} difference_param_t;



typedef union {
	euclidien_param_t euclidien;
	region_param_t region;
	mincut_param_t mincut;
	difference_param_t difference;
}segment_param_t;

extern std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& applySegment(
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, segment_type_t type, segment_param_t& param);

extern std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& applySegment();

#endif /* PCL_UTILS_SEGMENT_H_ */
