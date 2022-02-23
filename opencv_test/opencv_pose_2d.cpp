#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <sophus/se3.h>

using namespace std;
using namespace cv;

void find_feature_matches(const Mat &img1,
                            const Mat &img2,
                            vector<KeyPoint> &keypoints_1,
                            vector<KeyPoint> &keypoints_2,
                            vector<DMatch> &matches);

void pose_estimation_2d2d(vector<KeyPoint> &keypoints_1,
                        vector<KeyPoint> &keypoints_2,
                        vector<DMatch> matches,
                        Mat &R,
                        Mat &t);

void triangulation(
                const vector<KeyPoint> &keypoint_1,
                const vector<KeyPoint> &keypoint_2,
                const vector<DMatch> &matches,
                const Mat &R, const Mat &t,
                vector<Point3d> &points
);

inline cv::Scalar get_color(float depth) {
  float up_th = 20.0, low_th = 6.0, th_range = up_th - low_th;
  if (depth > up_th) depth = up_th;
  if (depth < low_th) depth = low_th;
  return cv::Scalar(255 * depth / th_range, 0, 255 * (1 - depth / th_range));
}

Point2d pixel2cam(const Point2d &p, const Mat &K);

//ａｕｔｏ－－－ｂａ
struct ReprojectionError {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    ReprojectionError( const double u_,const double v_ )
      : u(u_),v(v_){}    
    template <typename T>
    bool operator()(const T* const ceres_rot, const T* const ceres_trans, const T* const point,T* residuals) const {

    //cout<<"检查trans1:"<<endl<<ceres_trans[0]<<endl;  //<<","<<ceres_trans[1]<<","<<ceres_trans[2]
    T p[3];
    ceres::AngleAxisRotatePoint(ceres_rot,point,p);
    p[0]+=ceres_trans[0];
    p[1]+=ceres_trans[1];
    p[2]+=ceres_trans[2];
    T xp = p[0] / p[2];
    T yp = p[1] / p[2];
    T predicted_x = xp*458.654+367.215;
    T predicted_y = yp*457.296+248.375;
    residuals[0] = predicted_x - u;
    residuals[1] = predicted_y - v;

    return true;
  }
  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(const double u_,const double v_) {
    return (new ceres::AutoDiffCostFunction<ReprojectionError, 2, 3, 3, 3>(
                new ReprojectionError(u_,v_)));
  }
  double u,v;
};

//自定义雅克比－－－ｂａ
//反对称矩阵
template <typename Derived>
static Eigen::Matrix<typename Derived::Scalar, 3, 3> skewSymmetric(const Eigen::MatrixBase<Derived> &q)
{
    Eigen::Matrix<typename Derived::Scalar, 3, 3> ans;
    ans << typename Derived::Scalar(0), -q(2), q(1),
	q(2), typename Derived::Scalar(0), -q(0),
	-q(1), q(0), typename Derived::Scalar(0);
    return ans;
}

//自定义雅克比
class jacobin_ba : public ceres::SizedCostFunction<2,6,3> 
{
 public:
    virtual ~jacobin_ba() {}
    jacobin_ba(const double u_,const double v_)
    : u(u_), v(v_) {}

    virtual bool Evaluate(double const* const* parameters,
                        double* residuals,
                        double** jacobians) const {
    
    Eigen::Vector3d trans(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Quaterniond quaterd(parameters[0][6],parameters[0][3],parameters[0][4], parameters[0][5]);   //改旋转向量
    Eigen::Vector3d point(parameters[1][0], parameters[1][1], parameters[1][2]);
    //验证ｐａｒａｍ
    //cout<<"param1:"<<endl<<parameters[0][0]<<","<< parameters[0][1]<<","<<parameters[0][2]<<","<<parameters[0][3]<<","<<parameters[0][4]<<","<<parameters[0][5]<<","<<parameters[0][6]<<endl;
    //cout<<"param2-3d:"<<endl<<parameters[1][0]<<","<< parameters[1][1]<<","<<parameters[1][2]<<endl;
    Eigen::Vector3d p=quaterd * point + trans; //旋转向量 =rot * point + trans

    //定义fx,fy,cx,cy
    double f_x=458.654;
    double f_y=457.296;
    double c_x=367.215;
    double c_y=248.375;

    double f_x_z = f_x / p[2];
    double f_y_z = f_y / p[2];
    residuals[0] = u - f_x_z * p[0] + c_x;
    residuals[1] = v - f_y_z * p[1] + c_y ;

    Eigen::Matrix<double, 2, 3, Eigen::RowMajor> J_cam;
    double f_x_zz = f_x_z / p[2];
    double f_y_zz = f_y_z / p[2];

    J_cam << f_x_z,    0  , -f_x_zz * p[0],
            0      , f_y_z, -f_y_zz * p[1];

    Eigen::Matrix3d p_;
    p_ << 0, -p(2), p(1),
	    p(2), 0, -p(0),
	    -p(1), p(0), 0;

    if(jacobians != NULL)
    {
        if(jacobians[0] != NULL)    //位姿扰动求导
        {
            Eigen::Map<Eigen::Matrix<double, 2, 6, Eigen::RowMajor> > J_se3(jacobians[0]);
            J_se3.setZero();
            J_se3.block<2,3>(0,0) =  J_cam * p_;  //skew定义
            J_se3.block<2,3>(0,3) = - J_cam;
        }
        if(jacobians[1] != NULL)    //对空间点求导
        {
            Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor> > J_point(jacobians[1]);
            J_point = -J_cam * quaterd.toRotationMatrix();   //旋转向量转为矩阵
        }
    }

    return true;
  }

    const double u,v;
};

int main(int argc,char* argv[]) {
    if(argc != 3) {
        cout << "Usage: feature_extraction img1 img2." << endl;
        return 1;
    }
    //read image from disk
    Mat img_1 = imread(argv[1],cv::IMREAD_COLOR);
    Mat img_2 = imread(argv[2],cv::IMREAD_COLOR);
    assert(img_1.data != nullptr && img_2.data != nullptr);
    
    vector<DMatch> matches;
    vector<KeyPoint> keypoints_1, keypoints_2;
    //找到两张图像的特征匹配对
    find_feature_matches(img_1,img_2,keypoints_1,keypoints_2,matches);
    vector<cv::Point2f> L_2d;
    vector<cv::Point2f> R_2d;
    //按照mathces，找到匹配对
    for(int i = 0; i < matches.size(); i++) {
        L_2d.push_back(keypoints_1[matches[i].queryIdx].pt);
        R_2d.push_back(keypoints_2[matches[i].trainIdx].pt);
    }
    cout<<L_2d[1].x<<endl;
    //cout<<"数量："<<matches.size()<<endl;

    //计算基础矩阵和本质矩阵
    Mat R, t;
    pose_estimation_2d2d(keypoints_1,keypoints_2,matches,R,t);

    //三角化
    vector<Point3d> points;
    triangulation(keypoints_1,keypoints_2,matches,R,t,points);
    //cout<<"3d_points:"<<endl<<points<<endl;

    //ceres
    ceres::Problem problem;
    //传入左右相机的像素观测点：observed_2d(point_L_x,point_L_x ; point_R_x,point_R_y)
    double observed_2d[L_2d.size()][4]; 
    for(int i = 0; i < L_2d.size(); i++) {
        observed_2d[i][0]=L_2d[i].x;
        observed_2d[i][1]=L_2d[i].y;
        observed_2d[i][2]=R_2d[i].x;
        observed_2d[i][3]=R_2d[i].y;
        //cout<<observed_2d[i][0]<<","<<observed_2d[i][1]<<","<<observed_2d[i][2]<<","<<observed_2d[i][3]<<endl;
    }
    //cout<<"L_2d:"<<endl<<observed_2d[1][0]<<","<<observed_2d[1][1]<<endl;
    //cout<<"R_2d:"<<endl<<observed_2d[1][2]<<","<<observed_2d[1][3]<<endl;
    
    //传入左相机观测的３d点作为初始值：转换为double形式送入addresidualblock
    double observed_3d[points.size()][3];
    for(int i = 0; i < points.size(); i++) {
        observed_3d[i][0]=points[i].x;
        observed_3d[i][1]=points[i].y;
        observed_3d[i][2]=points[i].z;
    }
    //cout<<"输入初始参数point:"<<endl<<observed_3d[0][0] <<","<<observed_3d[0][1] <<","<<observed_3d[0][2] << endl;
    
    //传入旋转和平移初始值
    Mat rot_vector;
    cv::Rodrigues(R,rot_vector);
    //cout<<"旋转向量："<<endl<<rot_vector.row(1);
    Eigen::VectorXd pose_ceres(6,1);
    pose_ceres<<rot_vector.at<double>(0,0),rot_vector.at<double>(1,0),rot_vector.at<double>(2,0),t.at<double>(0,0),t.at<double>(1,0),t.at<double>(2,0);
    cout<<"pose_ceres:"<<endl<<pose_ceres<<endl;

    double ceres_rot[2][3] = {{0,0,0} , {pose_ceres(0),pose_ceres(1),pose_ceres(2)} };
    double ceres_trans[2][3] = {{0,0,0} , {pose_ceres(3),pose_ceres(4),pose_ceres(5)} };
    //double ceres_rot[3] = {pose_ceres(0),pose_ceres(1),pose_ceres(2)};
    //double ceres_trans[3] = {pose_ceres(3),pose_ceres(4),pose_ceres(5)};
    cout<<"优化前参数ｒ:"<<endl<<ceres_rot[0] <<","<<ceres_rot[1] <<","<<ceres_rot[2] <<","<< endl;
    cout<<"优化前参数t:"<<endl<<ceres_trans[0]<<","<<ceres_trans[1]<<","<<ceres_trans[2] << endl;

    
    /** 验证
    double point1[3]={-0.6853, -0.2125, 4.2936};
    double p1[3];
    ceres::AngleAxisRotatePoint(ceres_rot,point1,p1);
    p1[0]+=ceres_trans[0];
    p1[1]+=ceres_trans[1];
    p1[2]+=ceres_trans[2];
    double xp = p1[0] / p1[2];
    double yp = p1[1] / p1[2];
    double predicted_x = xp*458.654+367.215;
    double predicted_y = yp*457.296+248.375;
    cout<<"u,v:"<<predicted_x<<","<<predicted_y<<endl;
    //cout<<"p:"<<p1[0]<<","<<p1[1]<<","<<p1[2]<<endl;
    **/
    /**  ceres auto
    for(int j=0;j<2;j++)    //两个相机观测帧，j=0左相机，j=1右相机
    {
        for(size_t i = 0; i< L_2d.size(); ++i)    //size对２d匹配的观测点数据（左相机，右相机）
        {
            int a=0;
            int b=1;
            if(j=1)
            {
                 a=2;
                 b=3;
            }
            ceres::CostFunction* cost_function =
            ReprojectionError::Create(observed_2d[i][a],observed_2d[i][b]);//传递测量值到costFunction,右图的像素坐标
            //cout<<"检查传入的2ｄ点坐标："<<observed_2d[i][j]<<","<<observed_2d[i][j+1]<<endl;
            problem.AddResidualBlock(cost_function,NULL,ceres_rot[j],ceres_trans[j],observed_3d[i]); //传递外参数 points_3d_l,pts_3d_L[i].transpose()  
            //cout<<"检查传入的3ｄ点坐标："<<observed_3d[i][0]<<","<<observed_3d[i][1]<<endl; 
            //cout<<"检查传入的ceres_rot："<<observed_2d[i][j]<<","<<observed_2d[i][j+1]<<endl;
            //cout<<"检查传入的ceres_trans："<<observed_2d[i][j]<<","<<observed_2d[i][j+1]<<endl;
        }
    }

    **/

    //自定义雅克比－－ｂａ
    //××××××××××××写进函数中的自定义ＢＡ
    //定义初始值
    
    double para_Pose[2][7];
    para_Pose[0][0]=0;
    para_Pose[0][1]=0;
    para_Pose[0][2]=0;

    para_Pose[0][3]=0;
    para_Pose[0][4]=0;
    para_Pose[0][5]=0;
    para_Pose[0][6]=1;
    //Vector3d P = noise_cameras[i].translation();
    para_Pose[1][0] = t.at<double>(0,0);
    para_Pose[1][1] = t.at<double>(1,0);
    para_Pose[1][2] = t.at<double>(2,0);
    //cout<<"位移验证："<<endl<<
    
    /**
    //转换单位矩阵的四元数
    Eigen::Matrix3d r1;
    r1<<1,0,0,0,1,0,0,0,1;
    Eigen::Quaterniond rot_angle;
    rot_angle=r1;
    cout << "转换单位矩阵的四元数:" << endl << rot_angle.coeffs() << endl;
    **/
    
    Eigen::Matrix3d rot_matrix;
    rot_matrix<<R.at<double>(0,0),R.at<double>(0,1),R.at<double>(0,2),
                R.at<double>(1,0),R.at<double>(1,1),R.at<double>(1,2),
                R.at<double>(2,0),R.at<double>(2,1),R.at<double>(2,2);
    //cout<<"rot_matrix:"<<endl<<rot_matrix<<endl;
    Eigen::Quaterniond q(rot_matrix);
    //cout<<"q:"<<endl<<q.coeffs()<<endl;

    para_Pose[1][3] = q.x();
    para_Pose[1][4] = q.y();
    para_Pose[1][5] = q.z();
    para_Pose[1][6] = q.w();
    

    cout<<"输入初始参数t:"<<endl<<para_Pose[1][0]<<","<<para_Pose[1][1]<<","<<para_Pose[1][2]<<endl;
    cout<<"输入初始参数四元数(x,y,z,w):"<<endl<<para_Pose[1][3]<<","<<para_Pose[1][4]<<","<<para_Pose[1][5]<<","<<para_Pose[1][6]<<","<<endl;
    

    ceres::LossFunction* loss_function = NULL;
    
    for(int j=0;j<2;j++)  
    {
        for(size_t i = 0; i< L_2d.size(); ++i) 
        {
            int a=0;
            int b=1;
            if(j=1)
            {
                 a=2;
                 b=3;
            }
            ceres::CostFunction* cost_function = new jacobin_ba(observed_2d[i][a],observed_2d[i][b]);//传递测量值到costFunction,右图的像素坐标
            //jacobin_ba *f = new jacobin_ba(observe_X[i],observe_Y[i]);
            problem.AddResidualBlock(cost_function,loss_function,para_Pose[j],observed_3d[i]); //传递参数,points_3d_l  旋转向量
        }
    }
    
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = true;
    //options.max_num_iterations=200;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << "\n";
    

    //cout<<"优化后参数ｒ:"<<endl<<ceres_rot[1][0] <<","<<ceres_rot[1][1] <<","<<ceres_rot[1][2] <<","<< endl;
    //cout<<"优化后参数t:"<<endl<<ceres_trans[1][0]<<","<<ceres_trans[1][1]<<","<<ceres_trans[1][2] << endl;
    //cout<<"优化后参数point:"<<endl<<observed_3d[0]<<","<< endl;

    //cout<<"优化后参数point1:"<<endl<<point_l[0][0] <<","<<point_l[0][1] <<","<<point_l[0][2] << endl;
    cout<<"输入优化后参数t:"<<endl<<para_Pose[1][0]<<","<<para_Pose[1][1]<<","<<para_Pose[1][2]<<endl;
    cout<<"输入优化后参数四元数:"<<endl<<para_Pose[1][3]<<","<<para_Pose[1][4]<<","<<para_Pose[1][5]<<","<<para_Pose[1][6]<<endl;
    
    /**
    //相对误差判别　(q1.inv*q2).inv*(p1.inv*p2)
    Eigen::Quaterniond q2;
    q2.x()=para_Pose[1][3];
    q2.y()=para_Pose[1][4];
    q2.z()=para_Pose[1][5];
    q2.w()=para_Pose[1][6];
    cout<<"q2(x,y,z,w):"<<endl<<q2.coeffs()<<endl;
    Eigen::Vector3d p2(para_Pose[1][0],para_Pose[1][1],para_Pose[1][2]);
    Eigen::Quaterniond q_1=q.inverse()*q2;
    //double error_RPE=q_1.inverse();
    **/


    return 0;
}

void find_feature_matches(const Mat &img_1,
                            const Mat &img_2,
                            vector<KeyPoint> &keypoints_1,
                            vector<KeyPoint> &keypoints_2,
                            vector<DMatch> &matches) {
    Mat descriptors_1, descriptors_2;
    vector<DMatch> raw_matches;
    Ptr<FeatureDetector> detector = ORB::create();
    Ptr<DescriptorExtractor> descriptor = ORB::create();
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");

    //检测Oriented FAST 角点位置
    //chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    detector->detect(img_1,keypoints_1);
    detector->detect(img_2,keypoints_2);

    //根据角点位置计算BRIEF描述子
    descriptor->compute(img_1,keypoints_1,descriptors_1);
    descriptor->compute(img_2,keypoints_2,descriptors_2);

    //根据计算得到的描述子进行匹配，使用汉明距离

    matcher->match(descriptors_1,descriptors_2,raw_matches);

    //过滤raw_matches
    double dis_max = 0;
    double dis_min = 99999;

    //找到matches里面距离最大和最小，代表了特征点匹配程度，distance越大，匹配程度越低
    for(int i = 0; i < raw_matches.size();i++) {
        if(raw_matches[i].distance < dis_min) {
            dis_min = raw_matches[i].distance;
        }
        if(raw_matches[i].distance > dis_max) {
            dis_max = raw_matches[i].distance;
        }
    }

    printf("-- Max dist : %f \n",dis_max);
    printf("-- Min dist : %f \n",dis_min);

    for(int i = 0;i < raw_matches.size();i++) {
        if(raw_matches[i].distance < max(2* dis_min,30.0)) {
            matches.push_back(raw_matches[i]);
        }
    }
    cout <<"Final matches size is: " << matches.size() << endl;
}


void pose_estimation_2d2d(vector<KeyPoint> &keypoints_1,
                        vector<KeyPoint> &keypoints_2,
                        vector<DMatch> matches,
                        Mat &R,
                        Mat &t) {
    // 相机内参,TUM Freiburg2
    Mat K = (Mat_<double>(3, 3) << 458.654, 0, 367.215, 0, 457.296, 248.375, 0, 0, 1);

    //把keyPoints转换成vector<Point2f>
    vector<cv::Point2f> points_1;
    vector<cv::Point2f> points_2;
    //按照mathces，找到匹配对
    for(int i = 0; i < matches.size(); i++) {
        points_1.push_back(keypoints_1[matches[i].queryIdx].pt);
        points_2.push_back(keypoints_2[matches[i].trainIdx].pt);
    }

    Mat fundamental_matrix; //基础矩阵：左右两侧的点坐标系是图像坐标系
    fundamental_matrix = findFundamentalMat(points_1,points_2,FM_8POINT); // 八点法求基础矩阵，还有其他解法
    cout << "Fundamental matrix is: " << endl << fundamental_matrix << endl;

    Mat essential_matrix;
    Point2d principal_point(367.215, 248.375);  //相机光心, TUM dataset标定值
    double focal_length = 458.654;      //相机焦距, TUM dataset标定值
    essential_matrix = findEssentialMat(points_1,points_2,focal_length,principal_point);
    cout << "Essential matrix is: " << endl << essential_matrix << endl;

  recoverPose(essential_matrix, points_1, points_2, R, t, focal_length, principal_point);
  cout << "R is " << endl << R << endl;
  cout << "t is " << endl << t << endl;
}


Point2d pixel2cam(const Point2d &p, const Mat &K) {
  return Point2d
    (
      (p.x - K.at<double>(0, 2)) / K.at<double>(0, 0),
      (p.y - K.at<double>(1, 2)) / K.at<double>(1, 1)
    );
}


void triangulation(
                const vector<KeyPoint> &keypoint_1,
                const vector<KeyPoint> &keypoint_2,
                const vector<DMatch> &matches,
                const Mat &R, const Mat &t,
                vector<Point3d> &points) {
  Mat T1 = (Mat_<float>(3,4) << 
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0); //这里的3*4 矩阵代表了三维空间内的一点其次坐标投影到相机坐标系下的坐标。相机坐标和世界坐标系重合
  Mat T2 = (Mat_<float>(3,4) << 
        R.at<double>(0,0),R.at<double>(0,1),R.at<double>(0,2),t.at<double>(0,0),
        R.at<double>(1,0),R.at<double>(1,1),R.at<double>(1,2),t.at<double>(1,0),
        R.at<double>(2,0),R.at<double>(2,1),R.at<double>(2,2),t.at<double>(2,0)); // 传递过来的R和t相当于R21和t21，就是在2坐标系下的坐标点X2=[R21|t21]X1得到

  Mat K = (Mat_<double>(3,3) << 458.654, 0,367.215, 0, 457.296, 248.375, 0, 0, 1);
  vector<Point2f> pts_1,pts_2;

  for(DMatch m : matches) {
      pts_1.push_back(pixel2cam(keypoint_1[m.queryIdx].pt,K));
      pts_2.push_back(pixel2cam(keypoint_2[m.trainIdx].pt,K));
  }
  Mat pts_4d;
  
  cv::triangulatePoints(T1,T2,pts_1,pts_2,pts_4d); // 计算结果存储形式是按列排列的齐次坐标

  for(int i = 0; i < pts_4d.cols; i++) {
      Mat x = pts_4d.col(i);
      x /= x.at<float>(3,0);

      Point3d p (
          x.at<float>(0,0),
          x.at<float>(1,0),
          x.at<float>(2,0)
      );

      points.push_back(p);
  }
}