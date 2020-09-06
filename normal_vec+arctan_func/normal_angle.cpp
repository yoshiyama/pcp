//Display normal vector but only x,y
#include <vector>
#include <iostream>
#include <cmath>

using namespace std;

// int m;
// vector<int> v_angle(m);//for saving angle

//void normal_angle(double* nx,double*ny,int size)
void normal_angle(const double* nxy, vector<int>* v_angle, const int size)
{
    // int m = out_cloud->points.size();
	// int n = 2;//x,yで二つ
    // int m;
	// // int n = 2;//x,yで二つ

    // // vector<vector<double>> v(m, vector<double>(n));

    // // vector<int> v(2*m);//for saving normal_x・y
    // vector<int> v_angle(m);//for saving angle

    // m=size;
    //angle calculation
    double pi = 2.0 * asin(1.0);       // πの値
    double unit_r = 180.0 / pi;        // ラジアン → 度

    cout<<"right now\n";

	for (size_t i = 0; i < size; ++i) //gyou
	{
		// for (size_t j = 0; j < n; ++j) //retsu
		// {
			// cout << "x[" << i << "][" << j << "] = " << x[i][j] << '\n';
			// v[i][0]=(float)normals->points[i].normal_x;
			// v[i][1]=(float)normals->points[i].normal_y;
			// v[i][0]=nxy[i][0];
			// v[i][1]=nxy[i][1];
			// v[i][0]=nxy[2*i];
			// v[i][1]=nxy[2*i+1];
            cout << "normal_x[" << i <<"]=" <<nxy[2*i]<<", ";
            cout << "normal_y[" << i <<"]="<<nxy[2*i+1]<<endl;
			// v.push_back(nxy[2*i]);
			// v.push_back(nxy[2*i+1]);
			// cout << "x[" << i << "][" << 0 << "] = " << v[2*i] << ",";
			// cout << "x[" << i << "][" << 1 << "] = " << v[2*i+1] << '\n';

            // double n_x=v[i][0];//for calculating angle
            // double n_y=v[i][1];//for calculating angle

            double angle = nxy[2*i+1]/nxy[2*i];//radian
            double az = atan(angle) * unit_r;//convert to degree
			if(nxy[2*i]!=nan)
			{
				v_angle->push_back((int)az);
				// v_angle[i]=(int)az;
				cout << "angle=" <<(int)az<<"度"<<endl;
			}


		// }
	}

}