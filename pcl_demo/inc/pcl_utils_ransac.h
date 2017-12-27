/*
 * pcl_utils_ransac.h
 *
 *  Created on: 2017年12月17日
 *      Author: tangzm
 */


#include <iostream>
#include <fstream>
#include <stdio.h>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/filters/extract_indices.h>

#include <pcl/ModelCoefficients.h>


#ifndef PCL_UTILS_RANSAC_H_
#define PCL_UTILS_RANSAC_H_

typedef struct {
	int max_plane;

	int max_iter;
	double dist_threshold;
	double probability;
} ransac_param_t;

extern std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& applyRansac(
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const ransac_param_t& param);


#endif /* PCL_UTILS_RANSAC_H_ */
