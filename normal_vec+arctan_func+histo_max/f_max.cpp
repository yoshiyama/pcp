#include <iostream>
#include <vector>
#include <algorithm>
// #include <bits/stdc++.h>

using namespace std;

//import vector v
void f_max(vector<int>* v,int* id)
// void max(int* v)
{

//max_element::
    vector<int>::iterator itr_max = max_element(v->begin(), v->end());

    // size_t min_index = std::distance(v.begin(), itr_min);
    size_t max_index = distance(v->begin(), itr_max);
    *id = max_index;
    std::cout << "max element id=" << max_index << endl;
    // std::cout << "max element (hairetsu):" << v[max_index] << std::endl;
    std::cout << "max element (hairetsu):" << v->at(max_index) << std::endl;
    std::cout << "max element (pointer):" << *itr_max << std::endl;
}