///////////////////////////////////////////////////////////////////
// 元のソースコードの出典は下記です。それを，改造しました。
//
// 『お気楽Ｃ言語プログラミング超入門』
//
// http://www.nct9.ne.jp/m_hiroi/linux/clang.html#copyright
//
// はじめての統計学　コロナ社
//
////////////////////////////////////////////////////////////

// #include <stdio.h>
#include <cstdio>
#include <iostream>
#include <vector>
#include "histo.hpp"
#include "f_max.hpp"

#define N 100//number of data
#define M 8//階級数

using namespace std;

// double height[N] = {

//     ・・・省略・・・

// };

//same
double height[] = 
{
	148.7, 149.5, 133.7, 157.9, 154.2, 147.8, 154.6, 159.1, 148.2, 153.1,
	138.2, 138.7, 143.5, 153.2, 150.2, 157.3, 145.1, 157.2, 152.3, 148.3,
	152.0, 146.0, 151.5, 139.4, 158.8, 147.6, 144.0, 145.8, 155.4, 155.5,
	153.6, 138.5, 147.1, 149.6, 160.9, 148.9, 157.5, 155.1, 138.9, 153.0,
	153.9, 150.9, 144.4, 160.3, 153.4, 163.0, 150.9, 153.3, 146.6, 153.3,
	152.3, 153.3, 142.8, 149.0, 149.4, 156.5, 141.7, 146.2, 151.0, 156.5,
	150.8, 141.0, 149.0, 163.2, 144.1, 147.1, 167.9, 155.3, 142.9, 148.7,
	164.8, 154.1, 150.4, 154.2, 161.4, 155.0, 146.8, 154.2, 152.7, 149.7,
	151.5, 154.5, 156.8, 150.3, 143.2, 149.5, 145.6, 140.4, 136.5, 146.9,
	158.9, 144.4, 148.1, 155.5, 152.4, 153.3, 142.3, 155.3, 153.1, 152.3
};

//要素数の指定は，vector<int> X2(16); //要素数は16

int main(void)
{
	//配列->vector
	//M and N are already defined above.
	vector<int> freq(M);
	// int freq[M] = {0};   // 度数分布表
	vector<int> cum(M);
	// int cum[M];          // 累積度数表
	double low = 130.0;
	double z = 5.0;
	// 度数分布表の作成
	vector<double> height_v(N);
	vector<int> v_freq(M);

	//import to vector
	for(int i=0;i<N;++i)
	{
	height_v[i]=height[i];
	}


	histo(N, M, z,low,&height_v[0],&v_freq[0]);
	//get frequencies
	// void histo(const int size, const int cls, const int hb,const int low,const int* v,int* freq)
	//度数の計算
	//引数:
	// for (int i = 0; i < N; i++) //N:number of data
	// for (size_t i = 0; i < N; ++i) //N:number of data
	// {
	//   // for (int j = 0; j < M; j++) //階級をまわしている
	//   for (size_t j = 0; j < M; ++j) //階級をまわしている
	//   {
	//     if (height_v[i] < low + z * (j + 1)) //This is good.
	//     {
	//       freq[j]++;
	//       break;
	//     }
	//   }
	// }

	// f_max(vector<int>* v);
	f_max(&v_freq);

	// 累積度数表の作成
	cum[0] = v_freq[0];

	for (int i = 1; i < M; i++)
	{
	cum[i] = cum[i - 1] + v_freq[i];
	}

	// 表示
	for (int i = 0; i < M; i++) 
	{
	// printf("%.1f - %.1f | ", low + z * i, low + z * (i + 1));
	std::cout<< low + z * i << "-" << low + z * (i + 1) <<"|";//("%.1f - %.1f | ", , low + z * (i + 1));
	std::cout<< v_freq[i]<< " " << cum[i] <<"\n";//("%.1f - %.1f | ", , low + z * (i + 1));
	// printf("%3d %3d\n", freq[i], cum[i]);
	}
	return 0;
}