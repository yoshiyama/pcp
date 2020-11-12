//Display normal vector but only x,y
#include <vector>
#include <iostream>
#include <cmath>

using namespace std;

// int m;
// vector<int> v_angle(m);//for saving angle
// vector<double> v_nxy(2*m);//for saving angle

//void normal_angle(double* nx,double*ny,int size)
void normal_angle(const vector<double>* v_nxy, vector<int>* v_angle, const int size)
{
// void normal_angle(const double* nxy, int* angle, const int size)
// {
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

    cout<<"normals are being calculating right now"<<endl;
    #pragma omp parallel
    {
		#pragma omp for
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
			// cout << "normal_x[" << i <<"]=" <<v_nxy->at(2*i)<<", ";
			// cout << "normal_y[" << i <<"]="<<v_nxy->at(2*i+1)<<endl;
				// v.push_back(nxy[2*i]);
				// v.push_back(nxy[2*i+1]);
				// cout << "x[" << i << "][" << 0 << "] = " << v[2*i] << ",";
				// cout << "x[" << i << "][" << 1 << "] = " << v[2*i+1] << '\n';

				// double n_x=v[i][0];//for calculating angle
				// double n_y=v[i][1];//for calculating angle

				// double x = v_nxy->at(2*i);
				// double y = v_nxy->at(2*i+1);
				

				if(v_nxy->at(2*i)==0 && v_nxy->at(2*i+1)>0)
				{
					// v_angle->at(i) = ((int)az);
					v_angle->at(i) = 90;
					cout << "angle=" <<v_angle->at(i)<<"度"<<endl;
				}
				else if(v_nxy->at(2*i)==0 && v_nxy->at(2*i+1)==0)
				{
					v_angle->at(i) = 91;
					cout << "angle=" <<v_angle->at(i)<<"度"<<endl;			
				}
				else if(v_nxy->at(2*i)==0 && v_nxy->at(2*i+1)<0)
				{
					v_angle->at(i) = -90;
					cout << "angle=" <<v_angle->at(i)<<"度"<<endl;		
				}
				else
				{
					double angle = v_nxy->at(2*i+1) / v_nxy->at(2*i);//radian
					cout << "angle_calc"<<endl;
					double az = atan(angle) * unit_r;//convert to degree
					cout << "convert to degree"<<endl;
					// v_angle->push_back((int)az);
					v_angle->at(i) = ((int)az);
					// v_angle[i]=(int)az;
					cout << "angle=" <<(int)az<<"度"<<endl;
				}
			// }
		}
	}

}