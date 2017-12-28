#include "pcl_utils_segment.h"


using namespace std;
using namespace pcl;



vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& applySegment(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, segment_type_t type, segment_param_t& param){
	vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>* cloud_vector = new vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>();

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

	int nb_total = cloud->points.size();



	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud (cloud);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::ExtractIndices<pcl::PointXYZ> extract;


	if (euclidien == type) {
		pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
		ec.setClusterTolerance (param.euclidien.distance);
		ec.setMinClusterSize (param.euclidien.min_points);
		ec.setSearchMethod (tree);
		ec.setInputCloud (cloud);
		ec.extract (cluster_indices);
	}
	else if (region_growth == type) {
		pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
		normal_estimator.setSearchMethod (tree);
		normal_estimator.setInputCloud (cloud);
		normal_estimator.setKSearch (param.region.norm_neighbour);
		normal_estimator.compute (*normals);

		pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
		reg.setMinClusterSize (param.region.min_points);
		reg.setSearchMethod (tree);
		reg.setNumberOfNeighbours (param.region.neighbour);
		reg.setInputCloud (cloud);
		//reg.setIndices (indices);
		reg.setInputNormals (normals);
		reg.setSmoothnessThreshold (param.region.theta_thres / 180.0 * M_PI);
		reg.setCurvatureThreshold (param.region.curvature_thres);

		reg.extract (cluster_indices);
	}
	else if(min_cut == type){
		pcl::MinCutSegmentation<pcl::PointXYZ> seg;
		  seg.setInputCloud (cloud);

		  pcl::PointCloud<pcl::PointXYZ>::Ptr foreground_points(new pcl::PointCloud<pcl::PointXYZ> ());
		  pcl::PointXYZ point;
		  point.x = param.mincut.x;
		  point.y = param.mincut.y;
		  point.z = param.mincut.z;
		  foreground_points->points.push_back(point);
		  seg.setForegroundPoints (foreground_points);

		  seg.setSigma (param.mincut.sigma);
		  seg.setRadius (param.mincut.radius);
		  seg.setNumberOfNeighbours (param.mincut.neighbour);
		  seg.setSourceWeight (param.mincut.weight);

		  seg.extract (cluster_indices);
	}

	else if(difference_of_normals == type){
		pcl::NormalEstimation<PointXYZ, PointNormal> ne;
		  ne.setInputCloud (cloud);
  		  ne.setSearchMethod (tree);
          cout << "?????" << endl;
		  ne.setViewPoint (std::numeric_limits<float>::max (), std::numeric_limits<float>::max (), std::numeric_limits<float>::max ());
		  
		  cout << "Calculating normals for scale1..." << param.difference.scale1 << endl;
		  pcl::PointCloud<PointNormal>::Ptr normals_small_scale (new pcl::PointCloud<PointNormal>);
		  cout << "HERE" << endl;
		  ne.setRadiusSearch (param.difference.scale1);
		  cout << "HERE" << endl;
  		  ne.compute (*normals_small_scale);
		  cout << "Calculating normals for scale2..." << param.difference.scale2 << endl;	
		  pcl::PointCloud<PointNormal>::Ptr normals_large_scale (new pcl::PointCloud<PointNormal>);
		  ne.setRadiusSearch (param.difference.scale2);
  		  ne.compute (*normals_large_scale);
		  
		  PointCloud<PointNormal>::Ptr doncloud (new pcl::PointCloud<PointNormal>);
          copyPointCloud<PointXYZ, PointNormal>(*cloud, *doncloud);
		  pcl::DifferenceOfNormalsEstimation<PointXYZ, PointNormal, PointNormal> don;
          don.setInputCloud (cloud);
  		  don.setNormalScaleLarge (normals_large_scale);
  		  don.setNormalScaleSmall (normals_small_scale);
		  don.computeFeature (*doncloud);
		  
		pcl::ConditionOr<PointNormal>::Ptr range_cond (new pcl::ConditionOr<PointNormal> ());
		  range_cond->addComparison (pcl::FieldComparison<PointNormal>::ConstPtr (new pcl::FieldComparison<PointNormal> ("curvature", pcl::ComparisonOps::GT, param.difference.threshold)));

		pcl::ConditionalRemoval<PointNormal> condrem (false);
		  condrem.setCondition(range_cond);
  		  condrem.setInputCloud (doncloud);

  		pcl::PointCloud<PointNormal>::Ptr doncloud_filtered (new pcl::PointCloud<PointNormal>);
		  doncloud = doncloud_filtered;
		std::cout << "Filtered Pointcloud: " << doncloud->points.size () << " data points." << std::endl;
	}	  


	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);

		// Extract the inliers
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices(*it));

		cout<<"segment contains "<<inliers->indices.size()<<endl;

		extract.setInputCloud (cloud);
		extract.setIndices (inliers);
		extract.setNegative (false);
		extract.filter (*cloud_cluster);

		cloud_vector->push_back(cloud_cluster);
	}

	return *cloud_vector;
}
