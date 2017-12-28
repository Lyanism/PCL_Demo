/*
 * pcl_utils_filter.h
 *
 *  Created on: 2017年12月17日
 *      Author: tangzm
 */

#include <iostream>
#include <fstream>
#include <stdio.h>

#include <pcl/common/common_headers.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/project_inliers.h>

#include <pcl/ModelCoefficients.h>

#ifndef PCL_UTILS_FILTER_H_
#define PCL_UTILS_FILTER_H_


typedef enum {
	linear_filter,
	down_sample,
	stats_denoise,
	projection,
	radius_filter,
} filter_type_t;

typedef struct {
	/* If min < max, the range is [min, max]
	 * If min > max, the range is [min, positive-limit] U [negative-limit, max] */
	double min;
	double max;
	const char* field_name;
} linear_filter_param_t;

typedef struct {
	double voxel_size;
} down_sample_param_t;

typedef struct {
	int neighbours;
	double std_err;
} denoise_param_t;

typedef struct {
	double a; /*factor of X*/
	double b; /*factor of Y*/
	double c; /*factor of Z*/
	double w;
} projection_param_t;

typedef struct {
	double radius;
	int minNeighbours;
} radius_param_t;

typedef union{
	linear_filter_param_t linear_filter;
	down_sample_param_t down_sample;
	denoise_param_t denoise;
	projection_param_t projection;
	radius_param_t radius_filter;
} filter_param_t;

typedef struct {
	filter_type_t type;
	filter_param_t param;
} filter_proc_t;

extern void applyFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, std::vector<filter_proc_t>& filters);

#endif /* PCL_UTILS_FILTER_H_ */
