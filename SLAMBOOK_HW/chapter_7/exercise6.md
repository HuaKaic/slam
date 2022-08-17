# exercise7
1. 由于相机1位姿未知，故需要分别计算世界坐标系到相机1坐标系的位姿、到相机坐标2坐标系的位姿这样才能获得相机1到相机2的相对位姿。
***
```C++
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <chrono>
#include <sophus/se3.h>

using namespace std;
using namespace cv; 

void find_feature_matches(
  const Mat &img_1, const Mat &img_2,
  std::vector<KeyPoint> &keypoints_1,
  std::vector<KeyPoint> &keypoints_2,
  std::vector<DMatch> &matches);

// 像素坐标转相机归一化坐标
Point2d pixel2cam(const Point2d &p, const Mat &K);

// BA by g2o
typedef vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> VecVector2d;
typedef vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> VecVector3d;

void bundleAdjustmentG2O(
  const VecVector3d &points_3d,
  const VecVector2d &points1_2d,
  const VecVector2d &points2_2d,
  const Mat &K,
  Sophus::SE3 &pose
);
int main (int argc, char** argv) {
    if (argc != 4) {
    cout << "usage: pose_estimation_3d2d img1 img2 depth1 depth2" << endl;
    return 1;
  }
  //-- 读取图像
    Mat img_1 = imread(argv[1], CV_LOAD_IMAGE_COLOR);
    Mat img_2 = imread(argv[2], CV_LOAD_IMAGE_COLOR);
    assert(img_1.data && img_2.data && "Can not load images!");

    vector<KeyPoint> keypoints_1, keypoints_2;
    vector<DMatch> matches;
    find_feature_matches(img_1, img_2, keypoints_1, keypoints_2, matches);
    cout << "一共找到了" << matches.size() << "组匹配点" << endl;

    // 建立3D点
    Mat d1 = imread(argv[3], CV_LOAD_IMAGE_UNCHANGED);       // 深度图为16位无符号数，单通道图像
    Mat K = (Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
    vector<Point3f> pts_3d;
    vector<Point2f> pts1_2d,pts2_2d;
    for (DMatch m:matches) {
        ushort d = d1.ptr<unsigned short>(int(keypoints_1[m.queryIdx].pt.y))[int(keypoints_1[m.queryIdx].pt.x)];
        if (d == 0)   // bad depth
        continue;
        float dd = d / 1000.0;
        Point2d p1 = pixel2cam(keypoints_1[m.queryIdx].pt, K);
        pts_3d.push_back(Point3f(p1.x * dd, p1.y * dd, dd));//第一个相机观察到的3D点坐标
        pts1_2d.push_back(keypoints_1[m.queryIdx].pt);//特征点在第一个相机中的投影
        pts2_2d.push_back(keypoints_2[m.trainIdx].pt);//特征点在第二个相机中的投影
    }

    cout << "3d-2d pairs: " << pts_3d.size() << endl;

    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    Mat r, t;
    solvePnP(pts_3d, pts2_2d, K, Mat(), r, t, false); // 调用OpenCV 的 PnP 求解，可选择EPNP，DLS等方法
    Mat R;
    cv::Rodrigues(r, R); // r为旋转向量形式，用Rodrigues公式转换为矩阵
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << "solve pnp in opencv cost time: " << time_used.count() << " seconds." << endl;

    cout << "R=" << endl << R << endl;
    cout << "t=" << endl << t << endl;

    VecVector3d pts_3d_eigen;
    VecVector2d pts1_2d_eigen,pts2_2d_eigen;
    for (size_t i = 0; i < pts_3d.size(); i++) {
        pts_3d_eigen.push_back(Eigen::Vector3d(pts_3d[i].x, pts_3d[i].y, pts_3d[i].z));
        pts1_2d_eigen.push_back(Eigen::Vector2d(pts1_2d[i].x, pts1_2d[i].y));
        pts2_2d_eigen.push_back(Eigen::Vector2d(pts2_2d[i].x, pts2_2d[i].y));
    }

    cout << "calling bundle adjustment by g2o" << endl;
    Sophus::SE3 pose_g2o;
    t1 = chrono::steady_clock::now();
    bundleAdjustmentG2O(pts_3d_eigen, pts1_2d_eigen,pts2_2d_eigen, K, pose_g2o);
    t2 = chrono::steady_clock::now();
    time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << "solve pnp by g2o cost time: " << time_used.count() << " seconds." << endl;

    return 0;
}

class VertexPose : public g2o::BaseVertex<6, Sophus::SE3> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  virtual void setToOriginImpl() override {
    _estimate = Sophus::SE3();
  }

  /// left multiplication on SE3
  virtual void oplusImpl(const double *update) override {
    Eigen::Matrix<double, 6, 1> update_eigen;
    update_eigen << update[0], update[1], update[2], update[3], update[4], update[5];
    _estimate = Sophus::SE3::exp(update_eigen) * _estimate;
  }

  virtual bool read(istream &in) override {}

  virtual bool write(ostream &out) const override {}
};

class EdgeProjection : public g2o::BaseUnaryEdge<2, Eigen::Vector2d, VertexPose> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    EdgeProjection(const Eigen::Vector3d& pos, const Eigen::Matrix3d& K): _pos3d(pos), _K(K) {}

    virtual void computeError() override {
        const VertexPose* v = static_cast<VertexPose*> (_vertices[0]);
        Sophus::SE3 T = v->estimate();
        Eigen::Vector3d pos_pixel = _K * (T * _pos3d);
        pos_pixel /= pos_pixel[2];
        _error = _measurement - pos_pixel.head<2>();
    }

    virtual void linearizeOplus() override {
        const VertexPose* v = static_cast<VertexPose*> (_vertices[0]);
        Sophus::SE3 T = v->estimate();
        Eigen::Vector3d pos_cam = T * _pos3d;
        double fx = _K(0, 0);
        double fy = _K(1, 1);
        double cx = _K(0, 2);
        double cy = _K(1, 2);
        double X = pos_cam[0];
        double Y = pos_cam[1];
        double Z = pos_cam[2];
        double Z2 = Z * Z;
        _jacobianOplusXi
        << -fx / Z, 0, fx * X / Z2, fx * X * Y / Z2, -fx - fx * X * X / Z2, fx * Y / Z,
        0, -fy / Z, fy * Y / (Z * Z), fy + fy * Y * Y / Z2, -fy * X * Y / Z2, -fy * X / Z;
    }
    virtual bool read(istream& in) override {};
    virtual bool write(ostream& out) const override {};

private:
    Eigen::Vector3d _pos3d;
    Eigen::Matrix3d _K;
};

void bundleAdjustmentG2O (
    const VecVector3d& points_3d,
    const VecVector2d& points1_2d,
    const VecVector2d& points2_2d,
    const Mat& K,
    Sophus::SE3& pose) {


      typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 3>> Block;  // pose is 6, landmark is 3
      Block::LinearSolverType* linearsolver = new g2o::LinearSolverCSparse<Block::PoseMatrixType> (); // 线性求解器类型 LinearSolverCSparse
      // 梯度下降方法，可以从GN, LM, DogLeg 中选
      Block* solver_ptr = new Block ( linearsolver );     // 矩阵块求解器
      g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg ( solver_ptr );
      g2o::SparseOptimizer optimizer;
      optimizer.setAlgorithm ( solver );
      optimizer.setVerbose(true);       // 打开调试输出

      // 新增部分：vertex --(0,0,0)  第一个相机的位姿此时是未知的，待优化
      VertexPose *vertex_pose0 = new VertexPose(); // camera vertex_pose
      vertex_pose0->setId(0);
      Eigen::Matrix3d R=Eigen::Matrix3d::Identity();
      Eigen::Vector3d t(0,0,0);
      Sophus::SE3 SE3_Rt(R,t);
      vertex_pose0->setEstimate(SE3_Rt);
      optimizer.addVertex(vertex_pose0);
      
      // vertex --第二个相机的位姿，待优化
      VertexPose *vertex_pose = new VertexPose(); // camera vertex_pose
      vertex_pose->setId(1);
      vertex_pose->setEstimate(Sophus::SE3());
      optimizer.addVertex(vertex_pose);

      // K
      Eigen::Matrix3d K_eigen;
      K_eigen <<
      K.at<double>(0, 0), K.at<double>(0, 1), K.at<double>(0, 2),
      K.at<double>(1, 0), K.at<double>(1, 1), K.at<double>(1, 2),
      K.at<double>(2, 0), K.at<double>(2, 1), K.at<double>(2, 2);

      // edges
      int index = 1;
      
      //新增部分：第一个相机作为顶点连接的边
      for (size_t i = 0; i < points1_2d.size(); ++i) {
          auto p2d = points1_2d[i];
          auto p3d = points_3d[i];
          EdgeProjection *edge = new EdgeProjection(p3d, K_eigen);
          edge->setId(index);
          edge->setVertex(0, optimizer.vertex(0));
          edge->setMeasurement(p2d);
          edge->setInformation(Eigen::Matrix2d::Identity());
          optimizer.addEdge(edge);
          index++;
      }
      
      //第二个相机作为顶点连接的边
      for (size_t i = 0; i < points2_2d.size(); ++i) {
          auto p2d = points2_2d[i];
          auto p3d = points_3d[i];
          EdgeProjection *edge = new EdgeProjection(p3d, K_eigen);
          edge->setId(index);
          edge->setVertex(0, optimizer.vertex(1));
          edge->setMeasurement(p2d);
          edge->setInformation(Eigen::Matrix2d::Identity());
          optimizer.addEdge(edge);
          index++;
      }

      chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
      optimizer.setVerbose(true);
      optimizer.initializeOptimization();
      optimizer.optimize(100);
      chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
      chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
      cout << "optimization costs time: " << time_used.count() << " seconds." << endl;
      cout << "pose estimated of camera1 by g2o =\n" << vertex_pose0->estimate().matrix() << endl;
      cout<<"********************************************************************************************"<<endl;
      cout << "pose estimated of camera2 by g2o =\n" << vertex_pose->estimate().matrix() << endl;
      cout<<"********************************************************************************************"<<endl;
      pose = vertex_pose0->estimate().inverse()*vertex_pose->estimate();//此时待求的两个相机之间的相对位姿
      cout << "pose estimated by g2o =\n" << pose.matrix() << endl;
      cout<<"********************************************************************************************"<<endl;
}


    Point2d pixel2cam(const Point2d &p, const Mat &K) {
  return Point2d
    (
      (p.x - K.at<double>(0, 2)) / K.at<double>(0, 0),
      (p.y - K.at<double>(1, 2)) / K.at<double>(1, 1)
    );
}


    void find_feature_matches(const Mat &img_1, const Mat &img_2,
                          std::vector<KeyPoint> &keypoints_1,
                          std::vector<KeyPoint> &keypoints_2,
                          std::vector<DMatch> &matches) {
  //-- 初始化
  Mat descriptors_1, descriptors_2;
  // used in OpenCV3
  Ptr<FeatureDetector> detector = ORB::create();
  Ptr<DescriptorExtractor> descriptor = ORB::create();
  // use this if you are in OpenCV2
  // Ptr<FeatureDetector> detector = FeatureDetector::create ( "ORB" );
  // Ptr<DescriptorExtractor> descriptor = DescriptorExtractor::create ( "ORB" );
  Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");
  //-- 第一步:检测 Oriented FAST 角点位置
  detector->detect(img_1, keypoints_1);
  detector->detect(img_2, keypoints_2);

  //-- 第二步:根据角点位置计算 BRIEF 描述子
  descriptor->compute(img_1, keypoints_1, descriptors_1);
  descriptor->compute(img_2, keypoints_2, descriptors_2);

  //-- 第三步:对两幅图像中的BRIEF描述子进行匹配，使用 Hamming 距离
  vector<DMatch> match;
  // BFMatcher matcher ( NORM_HAMMING );
  matcher->match(descriptors_1, descriptors_2, match);

  //-- 第四步:匹配点对筛选
  double min_dist = 10000, max_dist = 0;

  //找出所有匹配之间的最小距离和最大距离, 即是最相似的和最不相似的两组点之间的距离
  for (int i = 0; i < descriptors_1.rows; i++) {
    double dist = match[i].distance;
    if (dist < min_dist) min_dist = dist;
    if (dist > max_dist) max_dist = dist;
  }

  printf("-- Max dist : %f \n", max_dist);
  printf("-- Min dist : %f \n", min_dist);

  //当描述子之间的距离大于两倍的最小距离时,即认为匹配有误.但有时候最小距离会非常小,设置一个经验值30作为下限.
  for (int i = 0; i < descriptors_1.rows; i++) {
    if (match[i].distance <= max(2 * min_dist, 30.0)) {
      matches.push_back(match[i]);
    }
  }
}


CMAKE:

cmake_minimum_required( VERSION 2.8 )
project( exercise )

set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )
# 添加cmake模块以使用g2o
list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )


find_package( OpenCV 3.1 REQUIRED )
# find_package( OpenCV REQUIRED ) # use this if in OpenCV2 
# find_package( G2O REQUIRED )
# find_package( CSparse REQUIRED )

IF(CSPARSE_FOUND) # upper format
   message("csparse is found:" ${CSPARSE_INCLUDE_DIR})
ENDIF(CSPARSE_FOUND)

include_directories( 
    ${OpenCV_INCLUDE_DIRS} 
    ${G2O_INCLUDE_DIRS}
    "/usr/include/eigen3/"
    "/usr/local/include/csparse/"
)
include_directories( "/home/liuhuakai/code_slam/slambook/3rdparty/Sophus")
add_executable(exercise main.cpp)
target_link_libraries( exercise "/home/liuhuakai/code_slam/slambook/3rdparty/Sophus/build/libSophus.so" 
${OpenCV_LIBS}
${OpenCV_LIBS}
${CSPARSE_LIBRARY}
g2o_types_sba 
g2o_core
g2o_solver_csparse
g2o_csparse_extension
g2o_ext_csparse
g2o_stuff
cxsparse )
```