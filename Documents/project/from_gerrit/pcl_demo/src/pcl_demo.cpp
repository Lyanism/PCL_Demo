/*
 * pcl_demo.cpp
 *
 *  Created on: 2017年12月17日
 *      Author: tangzm
 */


#include "pcl_utils_ts.h"

#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>

#include <boost/algorithm/string.hpp>
#include <boost/thread/thread.hpp>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <time.h>
#include <string>

using namespace std;
using namespace boost::property_tree;
using namespace boost::property_tree::json_parser;

void
printUsage (const char* progName)
{
  cout << "\n\nUsage: "<<progName<<" [options]\n\n"
            << "Options:\n"
            << "-------------------------------------------\n"
            << "-h           this help\n"
			<< "-cfg		     Use Configuration file, could be (-cfg test.json), then it will ignore all other commands\n";
}


typedef enum {
	mono,
	color
}visual_type_t;

class DelayTime{
	public:
		clock_t start,finish;
		double totaltime;
		string methodName;

		void countingBegin()
		{	
			cout<<"counting begin"<<endl;
			start = clock();
		}

		string getMethodName(string methodName)
		{	
			cout<<"the method name is :"<< methodName << endl;
			return methodName;
		}

		void printout()
		{	
			finish = clock();
			totaltime = double(finish-start)/CLOCKS_PER_SEC;
			cout<< "time of delay is : " << totaltime << endl;
		}

};

void
processConfiguration (string& file_path) {

  	pcl::visualization::PCLVisualizer::Ptr  viewer;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudMono (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudRGB(new pcl::PointCloud<pcl::PointXYZRGB>);

	// clock_t start,finish;
	// double totaltime;

	visual_type_t vis_type = mono;
	vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> seperated_clouds;

	point_cloud_holder_t pc_holder;

	ifstream infile(file_path.c_str());

	ptree pt;

	/*Read Config file*/
	try{
		read_json(infile, pt);
	}
	catch(ptree_error & e) {
		cout<<"error on parsing :"<<file_path<<endl<<e.what()<<endl;
		return;
	}

	/*Read Input file(generate point cloud)*/
    if (0 == pt.get<string>("input.type").compare("panasonic")){
    		readPanasonicPointCloudRaw(cloudMono,  pt.get<string>("input.file"));
    		vis_type = mono;
    }
    else if((0 == pt.get<string>("input.type").compare("pcd"))){
    		pcl::io::loadPCDFile<pcl::PointXYZ> (pt.get<string>("input.file"), *cloudMono);
    		vis_type = mono;
    }
    else if((0 == pt.get<string>("input.type").compare("pcd-rgb"))){
    		pcl::io::loadPCDFile<pcl::PointXYZRGB> (pt.get<string>("input.file"), *cloudRGB);
    		vis_type = color;
    }
    else {	
    		cout<<"Unsupported input type: "<<pt.get<string>("input.type")<<endl;
    		cout<<"We only support panasonic|pcd for the moment"<<endl;
    }

	/*Process with pipeline*/

    BOOST_FOREACH(boost::property_tree::ptree::value_type &v, pt.get_child("proc")){


		DelayTime delay;
		delay.countingBegin();
		
		
		// cout<<"start counting"<<endl;
		// start = clock();

		if(0 == v.second.get<string>("type").compare("linear-filter")){
			filter_proc_t item;
			vector<filter_proc_t> filter_params;

			item.type = linear_filter;

			item.param.linear_filter.field_name = v.second.get<string>("field-name").c_str();
			item.param.linear_filter.min = v.second.get<float>("min");
			item.param.linear_filter.max = v.second.get<float>("max");

			filter_params.push_back(item);

			applyFilter(cloudMono, filter_params);
			vis_type = mono;
		}
		else if (0 == v.second.get<string>("type").compare("downsample")){
			filter_proc_t item;
			vector<filter_proc_t> filter_params;
			delay.getMethodName(v.second.get<string>("type"));
			item.type = down_sample;


			item.param.down_sample.voxel_size = v.second.get<float>("size");
			filter_params.push_back(item);

			applyFilter(cloudMono, filter_params);
			vis_type = mono;
			delay.printout();
			
		}
		else if (0 == v.second.get<string>("type").compare("denoise")){
			filter_proc_t item;
			vector<filter_proc_t> filter_params;
			delay.getMethodName(v.second.get<string>("type"));
			item.type = stats_denoise;

			item.param.denoise.neighbours = v.second.get<int>("neighbour");
			item.param.denoise.std_err = v.second.get<float>("std_err");
			filter_params.push_back(item);

			applyFilter(cloudMono, filter_params);
			vis_type = mono;
			delay.printout();
			
			
		}
		else if (0 == v.second.get<string>("type").compare("project")){
			filter_proc_t item;
			vector<filter_proc_t> filter_params;

			item.type = projection;

			item.param.projection.a = v.second.get<float>("a");
			item.param.projection.b = v.second.get<float>("b");
			item.param.projection.c = v.second.get<float>("c");
			item.param.projection.w = v.second.get<float>("w");

			filter_params.push_back(item);

			applyFilter(cloudMono, filter_params);
			vis_type = mono;
		}
		else if(0 ==v.second.get<string>("type").compare("radius")){
			filter_proc_t item;
			vector<filter_proc_t> filter_params;

			item.type = radius_filter;
			item.param.radius_filter.radius = v.second.get<float>("radius");
			item.param.radius_filter.minNeighbours = v.second.get<float>("minNeighbours");

			filter_params.push_back(item);

			applyFilter(cloudMono, filter_params);
			vis_type = mono;
		}
		else if (0 == v.second.get<string>("type").compare("ransac")){
			ransac_param_t param;
			param.max_plane = v.second.get<int>("max-plane");
			param.max_iter = v.second.get<int>("iteration");
			param.probability = v.second.get<float>("probability");
			param.dist_threshold = v.second.get<float>("threshold");

			seperated_clouds = applyRansac(cloudMono, param);

			cloudRGB->clear();
			combinePointClouds(seperated_clouds, cloudRGB);
			vis_type = color;
		}
		else if (0 == v.second.get<string>("type").compare("euclidien")){
			segment_param_t param;
			delay.getMethodName(v.second.get<string>("type"));

			param.euclidien.distance = v.second.get<float>("distance");
			param.euclidien.min_points = v.second.get<int>("points");

			seperated_clouds = applySegment(cloudMono, euclidien, param);

			cloudRGB->clear();
			combinePointClouds(seperated_clouds, cloudRGB);
			vis_type = color;
			delay.printout();
			
		}
		else if (0 == v.second.get<string>("type").compare("region-growth")){
			segment_param_t param;

			param.region.norm_neighbour = v.second.get<int>("norm-neighbour");

			param.region.min_points = v.second.get<int>("min-points");
			param.region.neighbour = v.second.get<int>("neighbour");

			param.region.theta_thres = v.second.get<float>("theta-threshold");
			param.region.curvature_thres = v.second.get<float>("curvature-threshold");

			seperated_clouds = applySegment(cloudMono, region_growth, param);

			cloudRGB->clear();
			combinePointClouds(seperated_clouds, cloudRGB);
			vis_type = color;
		}
		else if (0 == v.second.get<string>("type").compare("mincut")){
			segment_param_t param;

			param.mincut.x = v.second.get<float>("x");
			param.mincut.y = v.second.get<float>("y");
			param.mincut.z = v.second.get<float>("z");

			param.mincut.neighbour = v.second.get<int>("neighbour");
			param.mincut.radius = v.second.get<float>("radius");
			param.mincut.sigma = v.second.get<float>("sigma");
			param.mincut.weight = v.second.get<float>("weight");

			seperated_clouds = applySegment(cloudMono, min_cut, param);
			cloudRGB->clear();
			combinePointClouds(seperated_clouds, cloudRGB);
			vis_type = color;
		}
		else if (0 == v.second.get<string>("type").compare("difference-of-normals")){
			segment_param_t param;

			param.difference.scale1 = v.second.get<float>("scale1");
			param.difference.scale2 = v.second.get<float>("scale2");
			param.difference.threshold = v.second.get<float>("threshold");

			seperated_clouds = applySegment(cloudMono, difference_of_normals, param);
			cloudRGB->clear();
			combinePointClouds(seperated_clouds, cloudRGB);
			vis_type = color;
		}

		
		
    }

    /*Display*/

    if (0 == pt.get<string>("output.file").compare("display")){
        if(mono == vis_type){
        		pc_holder.pXYZ = cloudMono;
        		viewer = simpleVis(pc_holder);
        }
        else if(color == vis_type){
        	    	pc_holder.pXYZRGB = cloudRGB;
        	    	viewer = simpleVis(pc_holder);
        }

        else {
    		cout<<"Unsupported output type: "<<pt.get<string>("output.type")<<endl;
    		cout<<"We only support simple|extracted-planes|segmentations for the moment"<<endl;
        }

    		/*Main looper*/

		
		

      	while (!viewer->wasStopped ())
        {
      	    /*Render it*/
            viewer->spinOnce (100);
			

            /*Sleep for 100ms*/
            boost::this_thread::sleep (boost::posix_time::microseconds (100000));
        }
    }
    else {
    		string output_file =  pt.get<string>("output.file");
        if(mono == vis_type){
        		cloudMono->width = 1;
        		cloudMono->height = cloudMono->points.size();
        		pcl::io::savePCDFileASCII (output_file, *cloudMono);
        }
        else if(color == vis_type){
        		cloudRGB->width = 1;
        		cloudRGB->height = cloudRGB->points.size();
        		pcl::io::savePCDFileASCII (output_file, *cloudRGB);
        }
    }

	    





}


// --------------
// -----Main-----
// --------------
int main (int argc, char** argv)
{
	int index = -1;

	/*Use config file*/
	
	if ((index=pcl::console::find_argument (argc, argv, "-cfg")) > 0){
		string file_path(argv[index+1]);
		processConfiguration(file_path);
		
	}

	else {
		cout<<"Only support cfg file now..."<<endl;
	}
}

