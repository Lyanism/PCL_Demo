/*
 * pcl_utils_oem_raw.h
 *
 *  Created on: 2017年12月17日
 *      Author: tangzm
 */

#include <fstream>
#include <iostream>
#include <pcl/common/common_headers.h>

#ifndef PCL_UTILS_OEM_RAW_H_
#define PCL_UTILS_OEM_RAW_H_

extern void readPanasonicPointCloudRaw(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud , const std::string& file_str);

extern void combinePointClouds(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clouds, pcl::PointCloud<pcl::PointXYZ>::Ptr target);
extern void combinePointClouds(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clouds, pcl::PointCloud<pcl::PointXYZRGB>::Ptr target);

#endif /* PCL_UTILS_OEM_RAW_H_ */
