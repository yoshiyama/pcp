/******************
Software License Agreement (BSD License)

Point Cloud Library (PCL) - www.pointclouds.org
Copyright (c) 2009-2012, Willow Garage, Inc.
Copyright (c) 2012-, Open Perception, Inc.
Copyright (c) XXX, respective authors.

All rights reserved.
//////////////////////////////////////
Normal Calc+arctan and then functionalized

atan returns radian

go.exe input.pcd output.pcd radius[m]

Input file: PCD file -> X Y Z
Output file:
********************/


#include <iostream>
#include <string>
#include <fstream> //
#include <iomanip>
#include <vector>

#include <cmath>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_cloud.h>
#include <pcl/common/impl/transforms.hpp>
// pcl/common/impl/transforms.hpp
// #include <pcl/classification/PHVObjectClassifier.h>
// #include <pcl/features/sgfall.h>



// #include "normal_angle.hpp"
// #include 

using namespace std;

// string getFileName();

int main(int argc, char** argv)
{
	if (argc != 3)
    {
        cout << "Error!\n **.exe input.pcd output.pcd";
            return 0;
    }

	// float rd;
	// rd=atof(argv[3]);

	//smart pointer::変数の宣言，点群オブジェクトの宣言
	// input file
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);//<pcl::PointXYZRGB>型
	if(pcl::io::loadPCDFile(argv[1], *in_cloud) == -1)
	{
		PCL_ERROR("Coudn't read PCD file\n");
		return(-1);
	}
	
	//smart pointer for savingpw. This is smart pointer, too.
    // pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr out_cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    //u_int32_t r = 0, g = 0, b = 0;

    //点群の変換開始
	//ポインタのメンバにアクセスする(アロー演算子)
	// init data on pcd
	// void init_pcd_write()
    out_cloud->width = in_cloud->width;
    out_cloud->height = in_cloud->height;  
    out_cloud->is_dense = in_cloud->is_dense;
    out_cloud->points.resize (in_cloud->width * in_cloud->height);
	// cout<< "vp_x,vp_y,vp_z=" << in_cloud->sensor_origin_ << endl;
	// cout<< "vp_x,vp_y,vp_z=" << in_cloud->sensor_orientation_ << endl;
	out_cloud->sensor_origin_ =  in_cloud->sensor_origin_ ;
	// out_cloud->sensor_orientation_ = in_cloud->sensor_orientation_ ;
	// in_cloud->sensor_orientation_ = Eigen::Quaternionf(euler2Quaternion(orientX, orientY, orientZ));
	// in_Cloud->sensor_orientation_ = Eigen::Quaternionf(euler2Quaternion(orientX, orientY, orientZ));
	// cout<< "vp_x,vp_y,vp_z=" << Eigen::Quaternionf(euler2Quaternion(orientX, orientY, orientZ)) << endl;

// Eigen::Quaternionf(euler2Quaternion(orientX, orientY, orientZ));
	
	// cout << "Output filename" << endl;
	//ofstream ofs;
	//ofs.open(argv[2]);

	// int p_size;
	// vector<double> n_v(p_size);//for storing normal vectors
	// vector<int> ang(p_size); //

	// p_size = out_cloud->points.size();

	// cout<<"done1\n";

	for(size_t i=0; i < out_cloud->points.size() ; ++i)
	// for(size_t i=0; i < cloud->points.size() ; ++i)
	{
        out_cloud->points[i].x = in_cloud->points[i].x;
        out_cloud->points[i].y = in_cloud->points[i].y;
        out_cloud->points[i].z = in_cloud->points[i].z;
	//arrow->演算子
    //色情報を強引に変更している部分
        // r = cloud->points[i].r;
        // g = cloud->points[i].g;
        // b = cloud->points[i].b;
        out_cloud->points[i].rgb = in_cloud->points[i].rgb;
        // flipNormalTowardsViewpoint(out_cloud->points[i], 0.0, 0.0, 0.0, normals->points[i].normal_x, normals->points[i].normal_y, normals->points[i].normal_z);
	}
    pcl::io::savePCDFileASCII (argv[2], *out_cloud);
	// cout<<"done\n";

	// return(0);
}
