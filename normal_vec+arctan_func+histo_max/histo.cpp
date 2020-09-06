// size:number of contents
// cls : 階級数
// hb :階級幅
// low : 最小値
// v :value
// freq:度数保存用
#include <cstddef>
#include <iostream>
#include <vector>

using namespace std;

void histo(const int size, const int cls, const int hb,const int low,const vector<int>* v,vector<int>* freq)
// void histo(const int size, const int cls, const int hb,const int low,const int* v,int* freq)
{

    // int N = size;
    // int M = cls;

//度数の計算
//引数:
    // for (int i = 0; i < N; i++) //N:number of data
    for (int i = 0; i < size; ++i) //N:number of data
    {
    // for (int j = 0; j < M; j++) //階級をまわしている
        for (int j = 0; j < cls; ++j) //階級をまわしている
        {//z:階級幅
            if (v->at(i) < low + hb * (j + 1)) //This is good.
            {
                freq->at(j)++;
                break;
            }
        }
    }

    cout <<"low="<<low<<endl;
    // for (int i = 0; i < M; i++) 
    for (int i = 0; i < cls; ++i) 
	{
        // cout <<"i="<<i<<endl;
        // cout <<"low="<<low<<endl;
        // cout <<"hb="<<hb<<endl;
        // cout <<"low+hb*i="<<low+(hb*i)<<endl;
	// printf("%.1f - %.1f | ", low + z * i, low + z * (i + 1));
        std::cout<< low + hb * i << "-" << low + hb * (i + 1) <<"|";//("%.1f - %.1f | ", , low + z * (i + 1));
        // std::cout<< v_freq[i]<< " " << cum[i] <<"\n";//("%.1f - %.1f | ", , low + z * (i + 1));
        std::cout<< freq->at(i)<< " " <<"\n";//("%.1f - %.1f | ", , low + z * (i + 1));
	// printf("%3d %3d\n", freq[i], cum[i]);
	}
}

void histo(const int size, const int cls, const double hb,const double low,const vector<double>* v,vector<int>* freq)
// void histo(const int size, const int cls, const int hb,const int low,const int* v,int* freq)
{

    // int N = size;
    // int M = cls;

//度数の計算
//引数:
    // for (int i = 0; i < N; i++) //N:number of data
    for (int i = 0; i < size; ++i) //N:number of data
    {
    // for (int j = 0; j < M; j++) //階級をまわしている
        for (int j = 0; j < cls; ++j) //階級をまわしている
        {//z:階級幅
            if (v->at(i) < low + hb * (j + 1)) //This is good.
            {
                freq->at(j)++;
                break;
            }
        }
    }
}