/*
 * pcl_utils_viewer.h
 *
 *  Created on: 2017年12月17日
 *      Author: tangzm
 */

#include <iostream>
#include <fstream>
#include <stdio.h>

#include <pcl/common/common_headers.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#ifndef PCL_UTILS_VIEWER_H_
#define PCL_UTILS_VIEWER_H_

typedef enum {
	plane,
	segment
}visual_mode_t;


typedef struct {
	pcl::PointCloud<pcl::PointXYZ>::Ptr pXYZ;
	pcl::PointCloud<pcl::PointXYZRGB >::Ptr pXYZRGB;
} point_cloud_holder_t;

extern pcl::visualization::PCLVisualizer::Ptr simpleVis (const point_cloud_holder_t& holder);


#endif /* PCL_UTILS_VIEWER_H_ */
