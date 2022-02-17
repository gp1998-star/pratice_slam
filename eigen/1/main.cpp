#include<iostream>
#include<eigen3/Eigen/Dense>
#include<eigen3/Eigen/Core>
#include<eigen3/Eigen/Geometry>

using namespace std;
using namespace Eigen;

int main()
{
    //eigen四元数输入为（ｗ,x,y,z）
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

    Eigen::Isometry3f imu_T_l=Eigen::Isometry3f::Identity();
    Eigen::Isometry3f imu_T_r=Eigen::Isometry3f::Identity();
    Eigen::Isometry3f l_T_r=Eigen::Isometry3f::Identity();

    imu_T_l.rotate(imu_q_l.toRotationMatrix());
    imu_T_l.pretranslate(imu_p_l);
    imu_T_r.rotate(imu_q_r.toRotationMatrix());
    imu_T_r.pretranslate(imu_p_r);
    
    l_T_r=imu_T_l.inverse()*imu_T_r;
    cout<<"右眼相机坐标系到左眼相机坐标系的变换矩阵："<<endl<<l_T_r.matrix()<<endl<<endl;
    cout<<"右眼相机坐标系到左眼相机坐标系的旋转矩阵："<<endl<<l_T_r.rotation()<<endl<<endl;
    cout<<"右眼相机坐标系到左眼相机坐标系的平移矩阵："<<endl<<l_T_r.translation()<<endl<<endl;

    Eigen::Quaternionf l_q_r(l_T_r.rotation());
    cout<<"右眼相机坐标系到左眼相机坐标系的四元数旋转（x,y,z,w）："<<endl<<l_q_r.coeffs()<<endl<<endl;
    
    
    //Eigen::Quaternionf l_q_r=imu_q_l.conjugate()*imu_q_r;
    //Eigen::Vector3f l_p_r=imu_q_l.conjugate()*(imu_p_r-imu_p_l);
    //cout<<"右眼相机坐标系到左眼相机坐标系的旋转l_q_r为："<<l_q_r.coeffs() <<endl;
    //cout<<"右眼相机坐标系到左眼相机坐标系的平移l_p_r为："<<l_p_r<<endl;
  
}
