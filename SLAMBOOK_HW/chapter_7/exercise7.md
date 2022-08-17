### 空间点的优化需要在图优化过程中加入二元边，其中一个顶点是相机2到相机1位姿，一个顶点是待优化的相机1下的空间点
***
```C++
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <chrono>
#include <sophus/se3.h>

using namespace std;
using namespace cv;

class VertexPoint : public g2o::BaseVertex<3, Eigen::Vector3d> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    virtual void setToOriginImpl() override {
        _estimate = Eigen::Vector3d(0, 0, 0);
    }

    virtual void oplusImpl(const double* update) override {
        _estimate += Eigen::Vector3d(update[0], update[1], update[2]);
    }

    virtual bool read(istream& in) override {}

    virtual bool write(ostream& out) const override {}
};

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


class Edgeproject : public g2o::BaseBinaryEdge<3, Eigen::Vector3d, g2o::VertexSE3Expmap, VertexPoint> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
   // Edgeproject( const Eigen::Vector3d& point ) : _point(point) {}
    virtual void computeError() override {
        const VertexPoint* point = static_cast< const VertexPoint* > (_vertices[1]);
        const g2o::VertexSE3Expmap* pose  = static_cast< const g2o::VertexSE3Expmap* > (_vertices[0]);
        _error = _measurement - pose->estimate() * point->estimate();
    }

    virtual bool read(istream& in) override {}

    virtual bool write(ostream& out) const override {}
protected:
    Eigen::Vector3d _point;
};


void bundeadjustmentG2o(
    const vector<Point3f>& pts1_3d,
    vector<Point3f>& pts2_3d,
    Mat& R, Mat& t);

Point2d pixel2cam(const Point2d &p, const Mat &K);

void find_feature_matches(
  const Mat &img_1, const Mat &img_2,
  std::vector<KeyPoint> &keypoints_1,
  std::vector<KeyPoint> &keypoints_2,
  std::vector<DMatch> &matches);

int main (int argc, char** argv) {
    if (argc != 5) {
        cout << "usage: pose_estimation_3d3d img1 img2 depth1 depth2" << endl;
        return 1;
    }
    //-- 读取图像
    Mat img_1 = imread(argv[1], CV_LOAD_IMAGE_COLOR);
    Mat img_2 = imread(argv[2], CV_LOAD_IMAGE_COLOR);

    vector<KeyPoint> keypoints_1, keypoints_2;
    vector<DMatch> matches;
    find_feature_matches(img_1, img_2, keypoints_1, keypoints_2, matches);
    cout << "一共找到了" << matches.size() << "组匹配点" << endl;

    // 建立3D点
    Mat depth1 = imread(argv[3], CV_LOAD_IMAGE_UNCHANGED);       // 深度图为16位无符号数，单通道图像
    Mat depth2 = imread(argv[4], CV_LOAD_IMAGE_UNCHANGED);       // 深度图为16位无符号数，单通道图像
    Mat K = (Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
    vector<Point3f> pts1, pts2;

    for (DMatch m:matches) {
        ushort d1 = depth1.ptr<unsigned short>(int(keypoints_1[m.queryIdx].pt.y))[int(keypoints_1[m.queryIdx].pt.x)];
        ushort d2 = depth2.ptr<unsigned short>(int(keypoints_2[m.trainIdx].pt.y))[int(keypoints_2[m.trainIdx].pt.x)];
        if (d1 == 0 || d2 == 0)   // bad depth
        continue;
        Point2d p1 = pixel2cam(keypoints_1[m.queryIdx].pt, K);
        Point2d p2 = pixel2cam(keypoints_2[m.trainIdx].pt, K);
        float dd1 = float(d1) / 1000.0;
        float dd2 = float(d2) / 1000.0;
        pts1.push_back(Point3f(p1.x * dd1, p1.y * dd1, dd1));
        pts2.push_back(Point3f(p2.x * dd2, p2.y * dd2, dd2));
    }

    cout << "3d-3d pairs: " << pts1.size() << endl;
    Mat R, t;
    Eigen::Matrix4d T1;

    bundeadjustmentG2o(pts1, pts2, R, t);
    /*R = (Mat_<double>(3, 3) << 
        T1 ( 0,0 ), T1 ( 0,1 ), T1 ( 0,2 ),
        T1 ( 1,0 ), T1 ( 1,1 ), T1 ( 1,2 ),
         T1 ( 2,0 ), T1 ( 2,1 ), T1 ( 2,2 ));
    t = (Mat_<double>(1 ,3) << T1(0, 3), T1(1, 3), T1(2, 3));
    double errot1total = 0;
    for(size_t i = 0; i < pts1.size(); i++)
    {
        cv::Mat error = (Mat_<double>(3, 1) << pts1[i].x, pts1[i].y, pts1[i].z) - (R * (Mat_<double>(3, 1) << pts2[i].x, pts2[i].y, pts2[i].z) + t);
        errot1total += norm(error);  
    }
    cout << "total error is " << errot1total << endl;*/

    return 0;
}

Point2d pixel2cam(const Point2d &p, const Mat &K) {
  return Point2d(
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

void bundeadjustmentG2o(
    const vector<Point3f>& pts1_3d,
    vector<Point3f>& pts2_3d,
    Mat& R, Mat& t) {
    
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 3>> Block;
    Block::LinearSolverType* linearsolver =  new g2o::LinearSolverDense<Block::PoseMatrixType>();
    Block* solverptr = new Block(linearsolver);
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solverptr);
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(true);

    g2o::VertexSE3Expmap* vertex_pose = new g2o::VertexSE3Expmap();
    vertex_pose->setId(0);
    vertex_pose->setEstimate(g2o::SE3Quat(Eigen::Matrix3d::Identity(),
    Eigen::Vector3d(0, 0, 0)));
    optimizer.addVertex(vertex_pose);

    int index = 1;

    for (size_t i = 0; i < pts2_3d.size(); i++) {
        VertexPoint* vertex_point = new VertexPoint();
        vertex_point->setId(index);
        vertex_point->setEstimate(Eigen::Vector3d(pts2_3d[i].x, pts2_3d[i].y, pts2_3d[i].z));
        vertex_point->setMarginalized ( true );
        optimizer.addVertex(vertex_point);
        index++;
    }

    for (size_t i = 0; i < pts1_3d.size(); i++) {
        Edgeproject* edge = new Edgeproject();
        edge->setId(i + 1);
        edge->setVertex(0, vertex_pose);
        edge->setVertex(1, dynamic_cast<VertexPoint*> (optimizer.vertex(i + 1)));
        edge->setMeasurement(Eigen::Vector3d(pts1_3d[i].x, pts1_3d[i].y, pts1_3d[i].z));
        edge->setInformation(Eigen::Matrix3d::Identity() * 1e-4);
        optimizer.addEdge(edge);
    }

    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    optimizer.initializeOptimization();
    optimizer.optimize(10);
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>> (t2 - t1);
    cout << "optimization costs time: " << time_used.count() << " seconds." << endl;

    cout << endl << "after optimization:" << endl;
    cout << "T=\n" << Eigen::Isometry3d(vertex_pose->estimate()).matrix() << endl;

    // convert to cv::Mat
    Eigen::Matrix3d R_ = vertex_pose->estimate().rotation().toRotationMatrix();
    Eigen::Vector3d t_ = vertex_pose->estimate().translation();
      R = (Mat_<double>(3, 3) <<
        R_(0, 0), R_(0, 1), R_(0, 2),
        R_(1, 0), R_(1, 1), R_(1, 2),
        R_(2, 0), R_(2, 1), R_(2, 2)
    );
    t = (Mat_<double>(3, 1) << t_(0, 0), t_(1, 0), t_(2, 0));
  

    for(size_t i = 0; i < pts2_3d.size(); i++)
    {
        Eigen::Vector3d vertex_point = dynamic_cast<VertexPoint *> ( optimizer.vertex ( i+1 ) )->estimate();
        pts2_3d[i] = Point3f(vertex_point(0),vertex_point(1),vertex_point(2));
    }
}
```

***
***
## 优化结果

![](2022-02-25-12-07-11.png)