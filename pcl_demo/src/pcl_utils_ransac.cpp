
#include "pcl_utils_ransac.h"

using namespace std;

vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& applyRansac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const ransac_param_t& param){
	vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>* cloud_vector = new vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>();

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

	int nb_total = cloud->points.size();

	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
	pcl::ExtractIndices<pcl::PointXYZ> extract;

	cout << "Totally points: " << nb_total << endl;

	for(int i=0; i<param.max_plane; i++){

		// Create the segmentation object
		pcl::SACSegmentation<pcl::PointXYZ> seg;

		// Mandatory
		seg.setModelType (pcl::SACMODEL_PLANE);
		seg.setMethodType (pcl::SAC_RANSAC);
		seg.setMaxIterations (param.max_iter);
		seg.setProbability(param.probability);
		seg.setDistanceThreshold (param.dist_threshold);

		seg.setInputCloud (cloud);
		seg.segment (*inliers, *coefficients);

		pcl::PointCloud<pcl::PointXYZ>::Ptr m_cloud(new pcl::PointCloud<pcl::PointXYZ>());

		// Extract the inliers
		extract.setInputCloud (cloud);
		extract.setIndices (inliers);
		extract.setNegative (false);
		extract.filter (*m_cloud);

		cloud_vector->push_back(m_cloud);

		/* Remain point clouds*/
		extract.setNegative (true);
		extract.filter(*cloud_filtered);
		cloud_filtered.swap(cloud);
	}

	cloud_vector->push_back(cloud);
	return *cloud_vector;
}





