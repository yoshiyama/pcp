//Display normal vector but only x,y
#include <vector>
#include <iostream>
#include <cmath>

using namespace std;

//void normal_angle(double* nx,double*ny,int size)
void normal_angle(const double* nxy,int* angle,const int size)
{
    // int m = out_cloud->points.size();
	// int n = 2;//x,yで二つ
    int m;
	int n = 2;//x,yで二つ

    vector<vector<double>> v(m, vector<double>(n));
    vector<int> v_angle(m);

    m=size;
    //angle calculation
    double pi = 2.0 * asin(1.0);            /* πの値 */
    double unit_r = 180.0 / pi;                 /* ラジアン → 度 */

    cout<<"right now\n";

	// for (size_t i = 0; i < size; ++i) //gyou
	for (unsigned int i = 0; i < size; ++i) //gyou
	{
		// for (size_t j = 0; j < n; ++j) //retsu
		// {
			// cout << "x[" << i << "][" << j << "] = " << x[i][j] << '\n';
			// v[i][0]=(float)normals->points[i].normal_x;
			// v[i][1]=(float)normals->points[i].normal_y;
			// v[i][0]=nxy[i][0];
			// v[i][1]=nxy[i][1];
			v[i][0]=nxy[2*i];
			v[i][1]=nxy[2*i+1];
			cout << "x[" << i << "][" << 0 << "] = " << v[i][0] << ",";
			cout << "x[" << i << "][" << 1 << "] = " << v[i][1] << '\n';

            double n_x=v[i][0];//for calculating angle
            double n_y=v[i][1];//for calculating angle

            // double angle = n_y/n_x;//radian
            // double az = atan(angle) * unit_r;//convert to degree
            // v_angle[i]=(int)az;
            // cout << "angle=" <<(int)az<<"度"<<'\n';
		// }
	}

}
