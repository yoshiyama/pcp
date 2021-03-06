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

1.normal calculation
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
#include <pcl/filters/filter.h>
#include <pcl/conversions.h>
#include <pcl/common/transforms.h>

using namespace std;

#include "f_max.hpp"
#include "normal_angle.hpp"
#include "histo.hpp"


// string getFileName();

int main(int argc, char** argv)
{
	if (argc != 5)
    {
        cout << "Error!\n **.exe input.pcd output.pcd radius[m] trans[m]\n";
            return 0;
    }

	float rd;
	rd=atof(argv[3]);

	//smart pointer::変数の宣言，点群オブジェクトの宣言
	//入力点群用のインスタンス
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr型
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);//<pcl::PointXYZRGB>型
	//データの読み込み
	if(pcl::io::loadPCDFile(argv[1], *cloud) == -1)
	{
		PCL_ERROR("Coudn't read PCD file\n");
		return(-1);
	}
	
	// Create the normal estimation class, and pass the input dataset to it
	// This is instancing, too.
	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;

	ne.setInputCloud (cloud);//This is smart pointer.
	//const修飾子はすぐ右のshared_ptr<>クラスにかかることになり、shared_ptrの指す先は変更できなくなりますが内容は変更可能なままです。
	//https://cdecrement.blog.fc2.com/blog-entry-58.html
	
	//This is smart pointer, too.
	// const pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
	pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);

	//Create an empty kdtree representation, and pass it to the normal estimation object.
	//Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
	// pcl::search::KdTree <pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree <pcl::PointXYZRGB> ());
	// ne.setSearchMethod (tree);
	//ne.setRadiusSearch (0.03);
	ne.setRadiusSearch (rd);
	
	//https://raymond-chen6.gitbooks.io/pcl/content/normal_estimation.html
	// Estimate the surface normals and 
	// store the result in "normals" 
	ne.compute (*normals);
	//smart pointer for savingpw. This is smart pointer, too.
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr out_cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    //u_int32_t r = 0, g = 0, b = 0;

    //点群の変換開始
	//ポインタのメンバにアクセスする(アロー演算子)
	// init data on pcd
	// void init_pcd_write()
    out_cloud->width=cloud->width;
    out_cloud->height=cloud->height;  
    out_cloud->is_dense=cloud->is_dense;
    out_cloud->points.resize (cloud->width * cloud->height);
	
	cout << "Output filename" << endl;
	//ofstream ofs;
	//ofs.open(argv[2]);

	int p_size;
	p_size = cloud->points.size();
	cout << "p_size=" << p_size <<endl;
	// vector<double> n_v(p_size);//for storing normal vectors
	// vector<int> ang(p_size); //
	// vector<int> h_freq(p_size); //
	p_size = out_cloud->points.size();

	cout<<"done1\n";

	for(size_t i=0; i < out_cloud->points.size() ; ++i)
	// for(size_t i=0; i < cloud->points.size() ; ++i)
	{
		// cout<<"before_go["<<i<<"]"<<endl;
        out_cloud->points[i].x = cloud->points[i].x;
        out_cloud->points[i].y = cloud->points[i].y;
        out_cloud->points[i].z = cloud->points[i].z;
		// cout<<"before_go_1["<<i<<"]"<<endl;
	//arrow->演算子
    //色情報を強引に変更している部分
        // r = cloud->points[i].r;
        // g = cloud->points[i].g;
        // b = cloud->points[i].b;
        out_cloud->points[i].rgb=cloud->points[i].rgb;
        // flipNormalTowardsViewpoint(out_cloud->points[i], 0.0, 0.0, 0.0, normals->points[i].normal_x, normals->points[i].normal_y, normals->points[i].normal_z);
        out_cloud->points[i].normal_x = normals->points[i].normal_x;
        out_cloud->points[i].normal_y = normals->points[i].normal_y;
		// cout<<"before_go_n_v_1["<<i<<"]"<<endl;
		// n_v[2*i] = normals->points[i].normal_x;
		// n_v[2*i+1] = normals->points[i].normal_y;
		// cout<<"after_go_n_v_1["<<i<<"]"<<endl;
        out_cloud->points[i].normal_z = normals->points[i].normal_z;
        out_cloud->points[i].curvature = normals->points[i].curvature;
		cout<<"go["<<i<<"]"<<endl;
	}

	std::vector<int> match_index;
	pcl::removeNaNNormalsFromPointCloud(*out_cloud,*out_cloud,match_index);

    // pcl::io::savePCDFileASCII (argv[2], *out_cloud);
	// cout<<"done\n";
	
	vector<double> n_v(2*(out_cloud->points.size()));//for storing normal vectors
	vector<int> ang(out_cloud->points.size()); //
	vector<int> h_freq(out_cloud->points.size()); //

	for(size_t i=0; i < out_cloud->points.size() ; ++i)
	// for(size_t i=0; i < cloud->points.size() ; ++i)
	{
		// cout<<"before_go["<<i<<"]"<<endl;
        // out_cloud->points[i].x = cloud->points[i].x;
        // out_cloud->points[i].y = cloud->points[i].y;
        // out_cloud->points[i].z = cloud->points[i].z;
		// cout<<"before_go_1["<<i<<"]"<<endl;
	//arrow->演算子
    //色情報を強引に変更している部分
        // r = cloud->points[i].r;
        // g = cloud->points[i].g;
        // b = cloud->points[i].b;
        // out_cloud->points[i].rgb=cloud->points[i].rgb;
        // flipNormalTowardsViewpoint(out_cloud->points[i], 0.0, 0.0, 0.0, normals->points[i].normal_x, normals->points[i].normal_y, normals->points[i].normal_z);
        // out_cloud->points[i].normal_x = normals->points[i].normal_x;
        // out_cloud->points[i].normal_y = normals->points[i].normal_y;
		// cout<<"before_go_n_v_1["<<i<<"]"<<endl;
		n_v[2*i] = out_cloud->points[i].normal_x;
		n_v[2*i+1] = out_cloud->points[i].normal_y;
		// cout<<"after_go_n_v_1["<<i<<"]"<<endl;
        // out_cloud->points[i].normal_z = normals->points[i].normal_z;
        // out_cloud->points[i].curvature = normals->points[i].curvature;
		// cout<<"go["<<i<<"]"<<endl;
	}
///////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////
//
// using vector
//
// calculate angle
//
/////////////////////////////////////////////////////////////////////////////////////////////////

	// void normal_angle(const double* nxy,int* angle,const int size);
	p_size = out_cloud->points.size();
	// normal_angle(&n_v[0],&ang,p_size);
	normal_angle(&n_v,&ang,p_size);

	int h_cls = 182;
	int h_hb = 1;
	int h_low = -90;
	cout<<"angle calculation done\n";
	cout << "階級数=" << h_cls <<endl;
	cout << "階級幅=" << h_hb <<endl;
	cout << "最小=" << h_low <<endl;	
	histo(p_size, h_cls, h_hb,h_low,&ang,&h_freq);

	int id_v=0;
	f_max(&h_freq,&id_v);
	cout<<"最大要素数を持つ偏角は"<<id_v + h_low <<"度です"<<endl;
	cout<<"ここでの偏角は，ｘ軸に対して"<<endl;

 	Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();

	//trans rotate
	int ty=0;
	ty=atof(argv[4]);

	transform_2.translation() <<0.0, -ty, 0.0;

	// The same rotation matrix as before; theta radians around Z axis
	// transform_2.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitZ()));
	float theta = ((float)(id_v + h_low)/180)*M_PI;
	transform_2.rotate (Eigen::AngleAxisf (-theta, Eigen::Vector3f::UnitZ()));

	cout<< "theta=" << theta <<endl;

	cout << "\nMethod #2: using an Affine3f\n";
	std::cout << transform_2.matrix() << std::endl;

	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZRGBNormal> ());

  	pcl::transformPointCloud (*out_cloud, *transformed_cloud, transform_2);

  	pcl::PCDWriter writer;
  	writer.write<pcl::PointXYZRGBNormal> (argv[2], *transformed_cloud, false);


// 	int m = out_cloud->points.size();
// 	int n = 2;//x,yで二つ
// //This is for saving normal vectors but just only x,y
// 	vector<vector<float> >v(m, vector<float>(n));

// //angle calculation
//     double pi = 2.0 * asin(1.0);            /* πの値 */
//     double unit_r = 180.0 / pi;                 /* ラジアン → 度 */

// //Display normal vector but only x,y
// 	for (size_t i = 0; i < out_cloud->points.size(); ++i) //gyou
// 	{
// 		// for (size_t j = 0; j < n; ++j) //retsu
// 		// {
// 			// cout << "x[" << i << "][" << j << "] = " << x[i][j] << '\n';
// 			v[i][0]=(float)normals->points[i].normal_x;
// 			v[i][1]=(float)normals->points[i].normal_y;
// 			cout << "x[" << i << "][" << 0 << "] = " << v[i][0] << ",";
// 			cout << "x[" << i << "][" << 1 << "] = " << v[i][1] << ',';

//             double n_x=v[i][0];//for calculating angle
//             double n_y=v[i][1];//for calculating angle
//             double angle = n_y/n_x;//radian
//             double az = atan(angle) * unit_r;//convert to degree
//             cout << "angle=" <<(int)az<<"度"<<'\n';
// 		// }
// 	}

	return(0);
}

/*
string getFileName()
{
	fflush(stdin);
	std::cout << "Input file name:";
	std::string file_name;
	std::getline( std::cin, file_name );
	return file_name;
}
*/