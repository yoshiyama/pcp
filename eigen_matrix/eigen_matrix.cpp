#include <iostream>
#include <string>

#include <pcl/common/transforms.h>//This includes Eigen
#include <pcl/io/pcd_io.h>

using namespace std;

// string getFileName();

void eigen_matrix(const char *str_inp,int trans, float rot)
// int main(int argc, char** argv)
{
    // if (argc !=5)
    // {
    //     //trans::スキャン位置間距離・移動量（現場メモ）
    //     cout << "Error!\n **.exe input.pcd output.pcd trans[m] rotate[deg]\n";
    //         return 0;
    // }

    	//smart pointer::変数の宣言，点群オブジェクトの宣言
	//入力点群用のインスタンス
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr型
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);//<pcl::PointXYZRGB>型
	//load input file
	if(pcl::io::loadPCDFile(*str_inp, *cloud) == -1)
	// if(pcl::io::loadPCDFile(argv[1], *cloud) == -1)
	{
		PCL_ERROR("Coudn't read PCD file\n");
		return(-1);
	}

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());

    //3.点群を移動する（前に算出した最頻偏角値とスキャン移動量で）Eigenを活用
	//setting translation
 	Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
//3.1 並進移動行列の作成
	//trans
	int ty=0;
	// ty=atof(argv[3]);//移動量
	ty=atof(trans);//移動量

	transform_2.translation() <<0.0, -ty, 0.0;
//3.2　回転移動行列の作成
	// The same rotation matrix as before; theta radians around Z axis
	// transform_2.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitZ()));
	// float theta = ((float)(id_v + h_low)/180)*M_PI;
	float theta = 0;
    float theta_deg=0;
    // theta_deg = atof(argv[4]);
    theta_deg = atof(rot);
	// float theta = ((float)(theta_deg)/180)*M_PI;
	theta = (((float)theta_deg)/180)*M_PI;
	transform_2.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitZ()));

	cout<< "theta=" << theta <<endl;

	cout << "\nMethod #2: using an Affine3f\n";
	std::cout << transform_2.matrix() << std::endl;

	// pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZRGBNormal> ());
//3.3 点群の移動
  	pcl::transformPointCloud (*cloud, *transformed_cloud, transform_2);
	//so far, transformeed_cloud is final cloud.

    pcl::io::savePCDFile (argv[2], *transformed_cloud, true);
}