#include<iostream>
#include<eigen3/Eigen/Dense>
#include<eigen3/Eigen/Core>
#include<eigen3/Eigen/Geometry>

using namespace std;
using namespace Eigen;

int main()
{
    Eigen::Quaternionf imu_q_l(0.99090224973327068,
                                0.00095051670014565813,
                                -0.0084222184858180373,
                                0.13431639597354814);
     Eigen::Quaternionf imu_q_r(0.99073762672679389,
                                -0.00013648999867379373,
                                -0.015306242884176362,
                                0.13492462817073628); 

    Eigen::Vector3f imu_p_l(-0.050720060477640147,
                                -0.0017414170413474165,
                                0.0022943667597148118);      
    Eigen::Vector3f imu_p_r(0.051932496584961352,
                                -0.0011555929083120534,
                                0.0030949732069645722);        
    imu_q_l.normalize();
    imu_q_r.normalize();

    Eigen::Quaternionf l_q_r=imu_q_l.conjugate()*imu_q_r;
    Eigen::Vector3f l_p_r=imu_q_l.conjugate()*(imu_p_l-imu_p_r);

    cout<<"右眼相机坐标系到左眼相机坐标系的旋转l_q_r为："<<l_q_r.coeffs() <<endl;
    cout<<"右眼相机坐标系到左眼相机坐标系的平移l_p_r为："<<l_p_r<<endl;
  
}
