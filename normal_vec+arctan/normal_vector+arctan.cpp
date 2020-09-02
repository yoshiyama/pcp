/******************
Software License Agreement (BSD License)

Point Cloud Library (PCL) - www.pointclouds.org
Copyright (c) 2009-2012, Willow Garage, Inc.
Copyright (c) 2012-, Open Perception, Inc.
Copyright (c) XXX, respective authors.

All rights reserved.
//////////////////////////////////////
Normal Calc+arctan

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

using namespace std;

// string getFileName();

int main(int argc, char** argv)
{
	if (argc != 4)
    {
        cout << "Error!\n **.exe input.pcd output.pcd radius[m]\n";
            return 0;
    }

	float rd;
	rd=atof(argv[3]);

	//smart pointer::変数の宣言，点群オブジェクトの宣言
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);//<pcl::PointXYZRGB>型
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
	const pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);

	//Create an empty kdtree representation, and pass it to the normal estimation object.
	//Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
	// pcl::search::KdTree <pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree <pcl::PointXYZRGB> ());
	// ne.setSearchMethod (tree);
	//ne.setRadiusSearch (0.03);
	ne.setRadiusSearch (rd);
	
	// Compute the features
	ne.compute (*normals);
	//smart pointer for savingpw
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr out_cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    //u_int32_t r = 0, g = 0, b = 0;

    //点群の変換開始
	//ポインタのメンバにアクセスする(アロー演算子)
    out_cloud->width=cloud->width;
    out_cloud->height=cloud->height;  
    out_cloud->is_dense=cloud->is_dense;
    out_cloud->points.resize (cloud->width * cloud->height);
	
	cout << "Output filename" << endl;
	//ofstream ofs;
	//ofs.open(argv[2]);

	for(size_t i=0; i < out_cloud->points.size() ; ++i)
	// for(size_t i=0; i < cloud->points.size() ; ++i)
	{
        out_cloud->points[i].x = cloud->points[i].x;
        out_cloud->points[i].y = cloud->points[i].y;
        out_cloud->points[i].z = cloud->points[i].z;
	//arrow->演算子
    //色情報を強引に変更している部分
        // r = cloud->points[i].r;
        // g = cloud->points[i].g;
        // b = cloud->points[i].b;
        out_cloud->points[i].rgb=cloud->points[i].rgb;
        // flipNormalTowardsViewpoint(out_cloud->points[i], 0.0, 0.0, 0.0, normals->points[i].normal_x, normals->points[i].normal_y, normals->points[i].normal_z);
        out_cloud->points[i].normal_x=normals->points[i].normal_x;
        out_cloud->points[i].normal_y=normals->points[i].normal_y;
        out_cloud->points[i].normal_z=normals->points[i].normal_z;
        out_cloud->points[i].curvature=normals->points[i].curvature;
	}
    pcl::io::savePCDFileASCII (argv[2], *out_cloud);
///////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////
	int m = out_cloud->points.size();
	int n = 2;//x,yで二つ

	vector<vector<float> >v(m, vector<float>(n));

    double pi = 2.0 * asin(1.0);            /* πの値 */
    double unit_r = 180.0 / pi;                 /* ラジアン → 度 */
	
//Display normal vector but only x,y
	for (size_t i = 0; i < out_cloud->points.size(); ++i) //gyou
	{
		// for (size_t j = 0; j < n; ++j) //retsu
		// {
			// cout << "x[" << i << "][" << j << "] = " << x[i][j] << '\n';
			v[i][0]=(float)normals->points[i].normal_x;
			v[i][1]=(float)normals->points[i].normal_y;
			cout << "x[" << i << "][" << 0 << "] = " << v[i][0] << ",";
			cout << "x[" << i << "][" << 1 << "] = " << v[i][1] << ',';
            double n_x=v[i][0];
            double n_y=v[i][1];
            double angle = n_y/n_x;
            double az = atan(angle) * unit_r;
            cout << "angle=" <<(int)az<<"度"<<'\n';
		// }
	}

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