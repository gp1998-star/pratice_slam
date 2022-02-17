#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp> 
#include <eigen3/Eigen/Dense>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/mat.hpp>
#include <cstdio>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <sophus/se3.h>
using namespace std;
using namespace cv;
using namespace Eigen;

typedef vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> VecVector3d;

Point2d pixe12cam(const Point2d &p,const Mat&  K)
{
    return Point2d(
        (p.x-K.at<double>(0,2)/K.at<double>(0,0)),
        (p.y-K.at<double>(1,2)/K.at<double>(1,1))
    );
}

//新版本
struct ReprojectionError {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    ReprojectionError(const double observed_X,const double observed_Y)
      : observed_x(observed_X), observed_y(observed_Y){}    //或者obs_x?
    template <typename T>
    bool operator()(const T* const ceres_rot, const T* const ceres_trans, const T* const point,T* residuals) const {
    //step1: 读取平移旋转的数据，转换成为Eigen的格式
    //新版本
    T p[3];
    ceres::AngleAxisRotatePoint(ceres_rot,point,p);
    p[0]+=ceres_trans[0];
    p[1]+=ceres_trans[1];
    p[2]+=ceres_trans[2];
    T xp = p[0] / p[2];
    T yp = p[1] / p[2];
    T predicted_x = xp*458.654+367.215;
    T predicted_y = yp*457.296+248.375;
    residuals[0] = predicted_x - observed_x;
    residuals[1] = predicted_y - observed_y;
    
    return true;
  }
  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(const double observed_X,
                                     const double observed_Y) {
    return (new ceres::AutoDiffCostFunction<ReprojectionError, 2, 3, 3, 3>(
                new ReprojectionError(observed_X,observed_Y)));
  }
  double observed_x;
  double observed_y;
};

/**
void bundleAdjustmentCeres(const vector<Point3d> &points_3d_L, const vector<double> &observe_X ,const vector<double> &observe_Y ,const Eigen::VectorXd &pose_ceres ) {
    //create problem
    ceres::Problem problem;
    //loss function for filter out outliers
    ceres::LossFunction* loss_function = NULL;

    //transform Sophus::SE3d to ceres data array
    
    double ceres_rot[3] = {pose_ceres(0),pose_ceres(1),pose_ceres(2)};
    double ceres_trans[3] = {pose_ceres(3),pose_ceres(4),pose_ceres(5)};
    cout<<"输入参数ｒ:"<<endl<<ceres_rot[0] <<","<<ceres_rot[1] <<","<<ceres_rot[2] << endl;
    cout<<"输入参数t:"<<endl<<ceres_trans[0] <<","<<ceres_trans[1] <<","<<ceres_trans[2] << endl;

    //cout << "measurement size is: " << points_2d.size() << endl;
    for(size_t i = 0; i< points_3d_L.size(); ++i) {
        double points_3d_l[3]={points_3d_L[i].x , points_3d_L[i].y , points_3d_L[i].z};
        ceres::CostFunction* cost_function =
        ReprojectionError::Create(observe_X[i],observe_Y[i]);//传递测量值到costFunction,右图的像素坐标
        // cout << "Adding meaurement to ceres: point_3d: (" << points_3d[i].transpose()
        //         << "). point_2d: (" << points_2d[i].transpose()
        //         << "). Camera intrinsic matrix: " <<  endl << K_eigen << "[" << i << "]" << endl;
        
        problem.AddResidualBlock(cost_function,loss_function,ceres_rot,ceres_trans); //传递外参数 points_3d_l
    }

    // Make Ceres automatically detect the bundle structure. Note that the
    // standard solver, SPARSE_NORMAL_CHOLESKY, also works fine but it is slower
    // for standard bundle adjustment problems.
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    //std::cout << summary.FullReport() << "\n";
    
    cout << "Result r: " << endl << ceres_rot[0] <<","<<ceres_rot[1] <<","<<ceres_rot[2] << endl;
    cout << "Result t: " << endl<<ceres_trans[0] <<","<<ceres_trans[1] <<","<<ceres_trans[2] << endl;
}
**/

int main ( int argc, char** argv )
{
    if ( argc != 3 )
    {
        cout<<"usage: feature_extraction img1 img2"<<endl;
        return 1;
    }
    //-- 读取图像
    Mat img_1 = imread ( argv[1], CV_LOAD_IMAGE_COLOR );
    Mat img_2 = imread ( argv[2], CV_LOAD_IMAGE_COLOR );

    //-- 初始化
    std::vector<KeyPoint> keypoints_1, keypoints_2;
    Mat descriptors_1, descriptors_2;
    Ptr<FeatureDetector> detector = ORB::create();
    Ptr<DescriptorExtractor> descriptor = ORB::create();
    // Ptr<FeatureDetector> detector = FeatureDetector::create(detector_name);
    // Ptr<DescriptorExtractor> descriptor = DescriptorExtractor::create(descriptor_name);
    Ptr<DescriptorMatcher> matcher  = DescriptorMatcher::create ( "BruteForce-Hamming" );

    //-- 第一步:检测 Oriented FAST 角点位置
    detector->detect ( img_1,keypoints_1 );
    detector->detect ( img_2,keypoints_2 );

    //-- 第二步:根据角点位置计算 BRIEF 描述子
    descriptor->compute ( img_1, keypoints_1, descriptors_1 );
    descriptor->compute ( img_2, keypoints_2, descriptors_2 );

    Mat outimg1;
    //drawKeypoints( img_1, keypoints_1, outimg1, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
    //imshow("左图ORB特征点",outimg1);
    Mat outimg2;
    //drawKeypoints( img_2, keypoints_2, outimg2, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
    //imshow("右图ORB特征点",outimg2);

    //-- 第三步:对两幅图像中的BRIEF描述子进行匹配，使用 Hamming 距离
    vector<DMatch> matches;
    matcher->match ( descriptors_1, descriptors_2, matches );

    //-- 第四步:匹配点对筛选
    double min_dist=10000, max_dist=0;

    //找出所有匹配之间的最小距离和最大距离, 即是最相似的和最不相似的两组点之间的距离
    for ( int i = 0; i < descriptors_1.rows; i++ )
    {
        double dist = matches[i].distance;
        if ( dist < min_dist ) min_dist = dist;
        if ( dist > max_dist ) max_dist = dist;
    }
    
    // 仅供娱乐的写法
    min_dist = min_element( matches.begin(), matches.end(), [](const DMatch& m1, const DMatch& m2) {return m1.distance<m2.distance;} )->distance;
    max_dist = max_element( matches.begin(), matches.end(), [](const DMatch& m1, const DMatch& m2) {return m1.distance<m2.distance;} )->distance;

    printf ( "-- Max dist : %f \n", max_dist );
    printf ( "-- Min dist : %f \n", min_dist );

    //当描述子之间的距离大于两倍的最小距离时,即认为匹配有误.但有时候最小距离会非常小,设置一个经验值30作为下限.
    std::vector< DMatch > good_matches;
    for ( int i = 0; i < descriptors_1.rows; i++ )
    {
        if ( matches[i].distance <= max (2*min_dist, 30.0 ) )
        {
            good_matches.push_back ( matches[i] );
        }
    }

    //-- 第五步:绘制匹配结果
    Mat img_match;
    Mat img_goodmatch;
    //drawMatches ( img_1, keypoints_1, img_2, keypoints_2, matches, img_match );
    //drawMatches ( img_1, keypoints_1, img_2, keypoints_2, good_matches, img_goodmatch );
    //imshow ( "所有匹配点对", img_match );
    //imshow ( "优化后匹配点对", img_goodmatch );
    //cout<<"所有匹配点对数目"<<matches.size()<<endl;
    //cout<<"优化后匹配点对数目"<<good_matches.size()<<endl;
    waitKey(0);

    //第六步：RANSAC
    vector<DMatch> m_matches;
    m_matches=good_matches;
    int ptcount=good_matches.size();
    
    vector<KeyPoint>RAN_kp1,RAN_kp2;
    for(size_t i=0;i<m_matches.size();i++)
    {
        RAN_kp1.push_back(keypoints_1[good_matches[i].queryIdx]);
        RAN_kp2.push_back(keypoints_2[good_matches[i].trainIdx]);
    }
    

    vector<Point2f>point1,point2;
    for(size_t i=0;i<m_matches.size();i++)
    {
        point1.push_back(RAN_kp1[i].pt);
        point2.push_back(RAN_kp2[i].pt);
    }
    //cout<<"point1:"<<endl<<point2[0].x<<endl;

    vector<double> obs_x,obs_y;
    for(int i=0;i<point2.size();i++)
    {
        obs_x.push_back(point2[i].x);
        obs_y.push_back(point2[i].y);
        //cout<<"obs:"<<endl<<obs_x[i]<<endl;
    }
    //cout<<"obs:"<<endl<<obs_x<<endl;

    //求基础矩阵
    vector<uchar>RansacStatus;
    Mat Fundamental=findFundamentalMat(point1,point2,RansacStatus,FM_RANSAC,3,0.99);

    //存储关键点和基础矩阵
    vector<KeyPoint>RR_kp1,RR_kp2;
    vector<DMatch>RR_matches;
    int index=0;
    for(size_t i=0;i<m_matches.size();i++)
    {
        if(RansacStatus[i]!=0)
        {
            RR_kp1.push_back(RAN_kp1[i]);
            RR_kp2.push_back(RAN_kp2[i]);
            m_matches[i].queryIdx=index;
            m_matches[i].trainIdx=index;
            RR_matches.push_back(m_matches[i]);
            index++;
        }
    }

    //cout<<"RANSAC后的匹配数目:"<<RR_kp1.size()<<endl;

    //cout<<"匹配对像素坐标左:"<<RR_kp1[0].pt.x<<endl;
    //cout<<"匹配对坐标右:"<<RR_kp2[0].pt<<endl;
    Mat img_RANSAC_matches;
    //drawMatches(img_1,RR_kp1,img_2,RR_kp2,RR_matches,img_RANSAC_matches);
    //imshow("RANSAC_matches",img_RANSAC_matches);
    //waitKey(0);

    //opencv求解Ｒ，ｔ 
    Mat R,t; 
    
    //F  
    Mat K=(Mat_<double>(3,3)<< 458.654 , 0 , 367.215,  0 , 457.296 , 248.375,  0 , 0 , 1);
    //cout<<"K:"<<endl<<K<<endl;
    Mat Fundamental_matrix;
    Fundamental_matrix=findFundamentalMat(point1,point2,CV_FM_8POINT);
    //cout<<"fundamental:"<<endl<<Fundamental_matrix<<endl;

    //E
    Mat Essential_matrix;
    Essential_matrix=findEssentialMat(point1,point2,K,RANSAC);
    //cout<<"essential_matrix:"<<endl<<Essential_matrix<<endl;

    //R,t
    cv::recoverPose(Essential_matrix,point1,point2,R,t,RANSAC);
    //cout<<"R:"<<endl<<R<<endl;
    //cout<<"t:"<<endl<<t<<endl;

    //验证Ｅ＝ｔ×Ｒ
    Mat t_x=(Mat_<double>(3,3)<<
        0,-t.at<double>(2,0),t.at<double>(1,0),
        t.at<double>(2,0),0,-t.at<double>(0,0),
        -t.at<double>(1,0),t.at<double>(0,0),0
    );
    //cout<<"E=  "<<endl<<t_x*R<<endl;
    //对极约束准确
    double u_1=RR_kp1[0].pt.x , v_1=RR_kp1[0].pt.y,   U_1=RR_kp2[0].pt.x,V_1=RR_kp2[0].pt.y;
    Mat x1=(Mat_<double>(3,1)<<u_1,v_1,1);
    Mat x2=(Mat_<double>(3,1)<<U_1,V_1,1);
    Mat X1=K.inv()*x1;
    Mat X2=K.inv()*x2;
    Mat d=X2.t()*t_x*R*X1;
    //cout<<"误差："<<d<<endl;

    //三角化
    vector<Point3d> points;
    //像素－>相机坐标
    vector<Point2d> pts_1,pts_2;

    for(DMatch m:RR_matches)    //matches
    {
        //pts_1.push_back(pixe12cam(keypoints_1[m.queryIdx].pt,K));
        pts_1.push_back(pixe12cam(RR_kp1[m.queryIdx].pt,K));
        pts_2.push_back(pixe12cam(RR_kp2[m.trainIdx].pt,K));
    }

    Mat T1=(Mat_<double>(3,4)<<1,0,0,0,0,1,0,0,0,0,1,0);
    Mat T2=(Mat_<double>(3,4)<<R.at<double>(0,0),R.at<double>(0,1),R.at<double>(0,2),t.at<double>(0,0),
                              R.at<double>(1,0),R.at<double>(1,1),R.at<double>(1,2),t.at<double>(1,0),
                              R.at<double>(2,0),R.at<double>(2,1),R.at<double>(2,2),t.at<double>(2,0));
    Mat point_4d;
    triangulatePoints(T1,T2,pts_1,pts_2,point_4d);
    //cout<<"3Ｄ坐标："<<endl<<point_4d.col(1)<<endl;
    //非齐次坐标
    for(int i=0;i<point_4d.cols;i++)
    {
        Mat x=point_4d.col(i);
        x/=x.at<double>(3,0);
        Point3d p(
            x.at<double>(0,0),
            x.at<double>(1,0),
            x.at<double>(2,0)
        );
        points.push_back(p);
    }
    //cout<<"3d坐标："<<endl<<points<<endl;

    //ceres求解BA优化（Ｒ，ｔ）和3Ｄ坐标
    //输入右图的像素坐标作为观测传输
    //double obs_x=point1.at

    //1:输入左图和右图对应的３Ｄ坐标    observed_L,observed_R;
    vector<Point3d> observed_L;
    vector<Point3d> observed_R;
    for( int i=0;i<RR_kp1.size();i++)
    {
        Point3d observed1(points[i].x,points[i].y,points[i].z);
        observed_L.push_back(observed1);   
        //cout<<"左图3d坐标："<<endl<<observed_L<<endl;
        Mat observed2=R*(Mat_<double>(3,1)<<points[i].x,points[i].y,points[i].z)+t;
        Point3d observed3(observed2.at<double>(0,0),observed2.at<double>(1,0),observed2.at<double>(2,0));
        observed_R.push_back(observed3);   
    }
    VecVector3d pts_3d_L;
    VecVector3d pts_3d_R;
    for(int i = 0; i < observed_L.size(); i++) {
        pts_3d_L.push_back(Eigen::Vector3d(observed_L[i].x,observed_L[i].y,observed_L[i].z));
        pts_3d_R.push_back(Eigen::Vector3d(observed_R[i].x,observed_R[i].y,observed_R[i].z));
    }
    //cout<<"左图3d坐标："<<endl<<pts_3d_L.size()<<endl;
    cout<<"右图3d坐标："<<endl<<pts_3d_R[2]<<endl;

    //2:旋转矩阵－旋转向量
    /**
    Eigen::Matrix3d rotation_matrix;
    rotation_matrix<<R.at<double>(0,0),R.at<double>(0,1),R.at<double>(0,2),
                    R.at<double>(1,0),R.at<double>(1,1),R.at<double>(1,2),
                    R.at<double>(2,0),R.at<double>(2,1),R.at<double>(2,2);
    cout<<"R:"<<endl<<rotation_matrix<<endl;
    
    Eigen::AngleAxisd cere_r;
    cere_r.fromRotationMatrix(rotation_matrix);
    cout<<"cere_r:"<<endl<<cere_r.angle()*cere_r.axis()<<endl;
    **/
    Mat cere_r;
    cv::Rodrigues(R,cere_r);
    //cout<<"cere_r:"<<endl<<cere_r<<endl;
    Eigen::VectorXd pose_ceres(6,1);
    pose_ceres<<cere_r.at<double>(0,0),cere_r.at<double>(1,0),cere_r.at<double>(2,0),t.at<double>(0,0),t.at<double>(1,0),t.at<double>(2,0);
    //cout<<"pose_ceres:"<<endl<<pose_ceres()<<endl;

    /**
    //为自定义雅克比输入pose
    double para_Pose[7];
    //Vector3d P = noise_cameras[i].translation();
    para_Pose[0] = t.at<double>(0,0);
    para_Pose[1] = t.at<double>(1,0);
    para_Pose[2] = t.at<double>(2,0);

    Eigen::Matrix3d  rot_matrix;
    rot_matrix<<R.at<double>(0,0),R.at<double>(0,1),R.at<double>(0,2),
                R.at<double>(1,0),R.at<double>(1,1),R.at<double>(1,2),
                R.at<double>(2,0),R.at<double>(2,1),R.at<double>(2,2);
    //cout<<"rot_matrix:"<<endl<<rot_matrix<<endl;
    Eigen::Quaterniond q(rot_matrix);
    para_Pose[3] = q.x();
    para_Pose[4] = q.y();
    para_Pose[5] = q.z();
    para_Pose[6] = q.w();
    //cout<<"para_Pose:"<<endl<<para_Pose[1]<<endl;
    **/

    //bundleAdjustmentCeres(observed_L,obs_x,obs_y,pose_ceres);
    //××××××××××××写进函数中的autoＢＡ
    //定义初始值
    ceres::Problem problem;
    double ceres_rot[3] = {pose_ceres(0),pose_ceres(1),pose_ceres(2)};
    double ceres_trans[3] = {pose_ceres(3),pose_ceres(4),pose_ceres(5)};
    cout<<"输入初始参数ｒ:"<<endl<<ceres_rot[0] <<","<<ceres_rot[1] <<","<<ceres_rot[2] << endl;
    cout<<"输入初始参数t:"<<endl<<ceres_trans[0] <<","<<ceres_trans[1] <<","<<ceres_trans[2] << endl;
    
    //将３d坐标转换为double形式送入addresidualblock
    double point_l[observed_L.size()][3];
    for(int i = 0; i < observed_L.size(); i++) {
        point_l[i][0]=observed_L[i].x;
        point_l[i][1]=observed_L[i].y;
        point_l[i][2]=observed_L[i].z;
    }
    cout<<"输入初始参数point:"<<endl<<point_l[0][0] <<","<<point_l[0][1] <<","<<point_l[0][2] << endl;

    for(size_t i = 0; i< observed_L.size(); ++i) {
        //double points_3d_l[3]={points_3d_L[i].x , points_3d_L[i].y , points_3d_L[i].z};
        ceres::CostFunction* cost_function =
        ReprojectionError::Create(obs_x[i],obs_y[i]);//传递测量值到costFunction,右图的像素坐标
    
        problem.AddResidualBlock(cost_function,NULL,ceres_rot,ceres_trans,point_l[i]); //传递外参数 points_3d_l,pts_3d_L[i].transpose()
        
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    //std::cout << summary.FullReport() << "\n";
    cout << "Using ceres the camera pose estimation is: " << endl << pose_ceres.transpose() << endl;
    cout<<"优化后参数ｒ:"<<endl<<ceres_rot[0] <<","<<ceres_rot[1] <<","<<ceres_rot[2] << endl;
    cout<<"优化后参数t:"<<endl<<ceres_trans[0] <<","<<ceres_trans[1] <<","<<ceres_trans[2] << endl;
    cout<<"优化后参数point1:"<<endl<<point_l[0][0] <<","<<point_l[0][1] <<","<<point_l[0][2] << endl;
    cout << summary.FullReport() << "\n";
    return 0;
}