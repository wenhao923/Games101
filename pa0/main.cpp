#include<cmath>
#include<eigen3/Eigen/Core>
#include<eigen3/Eigen/Dense>
#include<iostream>
using namespace std;

int main(){


   Eigen::Vector3f P(2.0,1.0,1.0);//定义向量，齐次坐标形式

   Eigen::Matrix3f Rotaion,Trans;
   Rotaion<<cos(45.0/180.0),-sin(45.0/180.0),0,
            sin(45.0/180.0),cos(45.0/180.0),0,
            0,0,1;

    Trans<<1,0,1,
            0,1,2,
            0,0,1;

    Eigen::Vector3f result=Trans*Rotaion*P;

    cout<<result;
    return 0;
}