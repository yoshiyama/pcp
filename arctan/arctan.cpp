/////////////////////////////////
//出典
//
//https://www.sist.ac.jp/~suganuma/cpp/man/function/atan.htm
//
//
////////////////////////////////////////////

// #include <math.h>
// #include <stdio.h>
#include <cmath>
#include <cstdio>
#include <iostream>

int main()
{
	double deg = 30.0;                   /* ３０度 */
	double ax, ay, az, aw, deg_r, pi, unit, unit_r, x,y,z;

	pi     = 2.0 * asin(1.0);            /* πの値 */
	unit   = pi / 180.0;                 /* 度 → ラジアン */
	unit_r = 180.0 / pi;                 /* ラジアン → 度 */

	deg_r  = deg * unit;//deg_r:radian
/*
		 三角関数の計算
*/
	x = sin(deg_r);
	y = cos(deg_r);
	z = tan(deg_r);
	// printf("３０度の正弦は %f\n", x);
	// printf("３０度の余弦は %f\n", y);
	// printf("３０度の正接は %f\n", z);
	std::cout << "３０度の正弦は" << x << '\n';
	std::cout << "３０度の余弦は" << y << '\n';
	std::cout << "３０度の正接は" << z << '\n';
/*
		 逆三角関数の計算
*/
	ax = asin(x) * unit_r;
	ay = acos(y) * unit_r;
	az = atan(z) * unit_r;
	// printf("正弦が %f になる角度は %f 度\n", x, ax);
	// printf("余弦が %f になる角度は %f 度\n", y, ay);
	// printf("正接が %f になる角度は %f 度\n", z, az);
	std::cout << "正弦が" << x <<  "になる角度は" << ax << "度" <<'\n';
	std::cout << "余弦が" << y <<  "になる角度は" << ay << "度" <<'\n';
	std::cout << "正接が" << z <<  "になる角度は" << az << "度" <<'\n';
	// std::cout << "余弦が %f になる角度は %f 度\n", y, ay);
	// std::cout << "正接が %f になる角度は %f 度\n", z, az);

	aw = atan2(1.0, -1.0) * unit_r;
	// printf("ｘ成分が－１，ｙ成分が１になる角度は %f 度\n", aw);
	std::cout << "ｘ成分が－１，ｙ成分が１になる角度は" << aw <<"度\n";

	return 0;
}