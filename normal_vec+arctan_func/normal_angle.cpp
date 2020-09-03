//Display normal vector but only x,y
#include <vector>

void normal_angle(double* nx,double*ny,int size)
{
    // int m = out_cloud->points.size();
	// int n = 2;//x,yで二つ
    int m = size;
	int n = 2;//x,yで二つ
    vector<vector<float> >v(m, vector<float>(n));

	for (size_t i = 0; i < out_cloud->points.size(); ++i) //gyou
	{
		// for (size_t j = 0; j < n; ++j) //retsu
		// {
			// cout << "x[" << i << "][" << j << "] = " << x[i][j] << '\n';
			v[i][0]=(float)normals->points[i].normal_x;
			v[i][1]=(float)normals->points[i].normal_y;
			cout << "x[" << i << "][" << 0 << "] = " << v[i][0] << ",";
			cout << "x[" << i << "][" << 1 << "] = " << v[i][1] << ',';

            double n_x=v[i][0];//for calculating angle
            double n_y=v[i][1];//for calculating angle
            double angle = n_y/n_x;//radian
            double az = atan(angle) * unit_r;//convert to degree
            cout << "angle=" <<(int)az<<"度"<<'\n';
		// }
	}

}
