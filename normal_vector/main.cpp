/******************
Normal Calc argv

Input file: PCD file -> X Y Z rgb
Output file:
********************/


#include <iostream>
#include <string>
#include <fstream> //
#include <iomanip>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>

using namespace std;

string getFileName();

int main(int argc, char** argv)
{
	float rd;
	
	rd=atof(argv[3]);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);


	if(pcl::io::loadPCDFile(argv[1], *cloud) == -1)
	{
		PCL_ERROR("Coudn't read PCD file\n");
		return(-1);
	}

	
	// Create the normal estimation class, and pass the input dataset to it
	pcl::NormalEstimation <pcl::PointXYZRGB, pcl::Normal> ne;
	ne.setInputCloud (cloud);
	
	const pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);

	//Create an empty kdtree representation, and pass it to the normal estimation object.
	//Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
	// pcl::search::KdTree <pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree <pcl::PointXYZRGB> ());
	// ne.setSearchMethod (tree);

	//
	//ne.setRadiusSearch (0.03);
	ne.setRadiusSearch (rd);


	//ñ@ê¸ÇÃåvéZåãâ éÊìæ

	
	// Compute the features
	ne.compute (*normals);

    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr out_cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    //u_int32_t r = 0, g = 0, b = 0;

    //点群の変換開始
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
        out_cloud->points[i].x=cloud->points[i].x;
        out_cloud->points[i].y=cloud->points[i].y;
        out_cloud->points[i].z=cloud->points[i].z;

    //色情報を強引に変更している部分
        // r = cloud->points[i].r;
        // g = cloud->points[i].g;
        // b = cloud->points[i].b;
        out_cloud->points[i].rgb=cloud->points[i].rgb;
        flipNormalTowardsViewpoint(out_cloud->points[i], 0.0, 0.0, 0.0, normals->points[i].normal_x, normals->points[i].normal_y, normals->points[i].normal_z);
        out_cloud->points[i].normal_x=normals->points[i].normal_x;
        out_cloud->points[i].normal_y=normals->points[i].normal_y;
        out_cloud->points[i].normal_z=normals->points[i].normal_z;
        out_cloud->points[i].curvature=normals->points[i].curvature;
	}

    pcl::io::savePCDFileASCII (argv[2], *out_cloud);

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