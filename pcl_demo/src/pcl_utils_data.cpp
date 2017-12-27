/*
 * proc_oem_raw.cpp
 *
 *  Created on: 2017年12月15日
 *      Author: tangzm
 */

#include "pcl_utils_data.h"

using namespace std;

typedef struct {
	uint8_t r;
	uint8_t g;
	uint8_t b;
} color_t;

static color_t render_colors[] = {
		{0xFF, 0, 0},  /*1. Red*/
		{0, 0xFF, 0},   /*2. Green*/
		{0, 0, 0xFF},   /*3. Blue*/
		{0xFF, 0xFF, 0},  /*4. Yellow*/
		{0xFF, 0x7F, 0},		/*5. Orange*/
		{0x7F, 0, 0x7F}, 	/*6. Purple*/
		{0x99, 0x66, 0x33} /*7. Brown*/
};

static string render_color_names[] = {
		"red",
		"green",
		"blue",
		"yellow",
		"orange",
		"purple",
		"brown"
};

void readPanasonicPointCloudRaw(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud , const string& file_str) {

	ifstream infile(file_str.c_str());

	if(!infile.is_open()) {
		cout<<"Can not open:"<<file_str<<"\n";
	}
	else {
     	pcl::PointXYZ basic_point;
        char cellXYZ[6];

        while(infile){
        		infile.read(cellXYZ, 6);

        		/*Calculated in little endian, the result unit will be 1 meter*/
        		basic_point.x = double(cellXYZ[1]<<8 | (cellXYZ[0] & 0xFF))/1000;
        		basic_point.y = double(cellXYZ[3]<<8 | (cellXYZ[2] & 0xFF))/1000;
        		basic_point.z = double(cellXYZ[5]<<8 | (cellXYZ[4] & 0xFF))/1000;

        		cloud->points.push_back(basic_point);
        }
	}
}


void generateCylinderPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {

	/*Draw a cylinder*/
	for (float z(-1.0); z <= 1.0; z += 0.05)
  	{
	    for (float angle(0.0); angle <= 360.0; angle += 5.0)
	    {
	      pcl::PointXYZ basic_point;
	      basic_point.x = cosf (pcl::deg2rad(angle));
	      basic_point.y = sinf (pcl::deg2rad(angle));
	      basic_point.z = z;
	      cloud->points.push_back(basic_point);
	    }
  	}
}

void combinePointClouds(const vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clouds, pcl::PointCloud<pcl::PointXYZ>::Ptr target){
	for (int i=0; i<clouds.size(); i++){
		(*target) += *(clouds[i]);
	}
}

void combinePointClouds(const vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clouds, pcl::PointCloud<pcl::PointXYZRGB>::Ptr target){

	int render_color_len = sizeof(render_colors)/sizeof(render_colors[0]);

	for (int i=0; i<clouds.size(); i++){
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = clouds[i];
		uint8_t r = render_colors[i%render_color_len].r;
		uint8_t g = render_colors[i%render_color_len].g;
		uint8_t b = render_colors[i%render_color_len].b;


		for (int j=0; j<clouds[i]->points.size(); j++){
			pcl::PointXYZRGB basic_point;
			basic_point.x = cloud->points[j].x;
			basic_point.y = cloud->points[j].y;
			basic_point.z = cloud->points[j].z;

			basic_point.r = r;
			basic_point.g = g;
			basic_point.b = b;

			target->points.push_back(basic_point);
		}
	}
}

