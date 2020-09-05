// size:number of contents
// cls : 階級数
// hb :階級幅
// low : 最小値
// v :value
// freq:度数保存用
#include <cstddef>

using namespace std;

void histo(const int size, const int cls, const int hb,const int low,const int* v,int* freq)
// void histo(const int size, const int cls, const int hb,const int low,const int* v,int* freq)
{

    // int N = size;
    // int M = cls;

//度数の計算
//引数:
    // for (int i = 0; i < N; i++) //N:number of data
    for (size_t i = 0; i < size; ++i) //N:number of data
    {
    // for (int j = 0; j < M; j++) //階級をまわしている
        for (size_t j = 0; j < cls; ++j) //階級をまわしている
        {//z:階級幅
            if (v[i] < low + hb * (j + 1)) //This is good.
            {
                freq[j]++;
                break;
            }
        }
    }
}

void histo(const int size, const int cls, const double hb,const double low,const double* v,int* freq)
// void histo(const int size, const int cls, const int hb,const int low,const int* v,int* freq)
{

    // int N = size;
    // int M = cls;

//度数の計算
//引数:
    // for (int i = 0; i < N; i++) //N:number of data
    for (size_t i = 0; i < size; ++i) //N:number of data
    {
    // for (int j = 0; j < M; j++) //階級をまわしている
        for (size_t j = 0; j < cls; ++j) //階級をまわしている
        {//z:階級幅
            if (v[i] < low + hb * (j + 1)) //This is good.
            {
                freq[j]++;
                break;
            }
        }
    }
}