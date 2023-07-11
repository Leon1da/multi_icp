#pragma once
#include <iostream>
#include <unistd.h>
#include <istream>
#include <iterator>
#include <vector>
#include <iomanip>
#include <queue>
#include <limits.h>

#include <Eigen/Eigenvalues> 
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Cholesky>
#include <Eigen/StdVector>
#include "opencv2/opencv.hpp"

using namespace std;
using namespace cv;

typedef std::vector<Eigen::VectorXf, Eigen::aligned_allocator<Eigen::VectorXf> > VectorXfVector;
typedef std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > Vector3fVector;
typedef std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f> > Vector2fVector;
typedef std::vector<Eigen::Isometry2f, Eigen::aligned_allocator<Eigen::Isometry2f> > Isometry2fVector;

typedef Eigen::Matrix<float, 2, 1> Vector2f;
typedef Eigen::Matrix<float, 3, 1> Vector3f;
typedef Eigen::Matrix<float, 4, 1> Vector4f;
typedef Eigen::Matrix<float, 6, 1> Vector6f;


typedef Eigen::Matrix<double, 2, 2> Matrix2d;

typedef Eigen::Matrix<float, 2, 2> Matrix2f;
typedef Eigen::Matrix<float, 3, 3> Matrix3f;
typedef Eigen::Matrix<float, 6, 6> Matrix6f;

typedef Eigen::Matrix<float, 1, 3> Matrix1_3f;
typedef Eigen::Matrix<float, 2, 3> Matrix2_3f;




typedef std::vector<Eigen::VectorXd, Eigen::aligned_allocator<Eigen::VectorXd> > VectorXdVector;
typedef std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > Vector3dVector;
typedef std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > Vector2dVector;
typedef std::vector<Eigen::Isometry2d, Eigen::aligned_allocator<Eigen::Isometry2d> > Isometry2dVector;

typedef Eigen::Matrix<double, 1, 3> Matrix1_3d;

typedef Eigen::Matrix<double, 2, 1> Vector2d;
typedef Eigen::Matrix<double, 3, 1> Vector3d;
typedef Eigen::Matrix<double, 4, 1> Vector4d;


typedef std::vector<int> IntVector;
typedef std::vector<float> FloatVector;
typedef std::vector<double> DoubleVector;
typedef std::vector<bool> BoolVector;

typedef std::pair<float, float> FloatPair;
typedef std::pair<int,int> IntPair;


typedef std::vector<IntPair> IntPairVector;

typedef std::tuple<int, int, int> IntTriple;
typedef std::pair<IntTriple, IntTriple> TriplePair;
typedef std::vector<TriplePair> TriplePairVector;

// typedef std::pair<IntPair, IntPair> Correspondence;

class Correspondence{
    public:
        int _src_pose;
        int _src_point;
        int _dst_pose;
        int _dst_point;

        Correspondence(){};

        Correspondence(int src_pose, int src_point, int dst_pose, int dst_point){
            _src_pose = src_pose;
            _src_point = src_point;
            _dst_pose = dst_pose;
            _dst_point = dst_point;
        };

        Correspondence(IntPair src, IntPair dst){
            _src_pose = src.first;
            _src_point = src.second;
            _dst_pose = dst.first;
            _dst_point =dst.second;
        };
};

inline Eigen::Isometry3f v2t(const Vector6f& t){
    Eigen::Isometry3f T;
    T.setIdentity();
    T.translation()=t.head<3>();
    float w=t.block<3,1>(3,0).squaredNorm();
    if (w<1) {
        w=sqrt(1-w);
        T.linear()=Eigen::Quaternionf(w, t(3), t(4), t(5)).toRotationMatrix();
    } else {
        T.linear().setIdentity();
    }
    return T;
}

inline Vector6f t2v(const Eigen::Isometry3f& t){
    Vector6f v;
    v.head<3>()=t.translation();
    Eigen::Quaternionf q(t.linear());
    v.block<3,1>(3,0)=q.matrix().block<3,1>(1,0);
    if (q.w()<0)
        v.block<3,1>(3,0) *= -1.0f;
    return v;
}

inline Eigen::Isometry2f v2t(const Eigen::Vector3f& t){
    Eigen::Isometry2f T;
    T.setIdentity();
    T.translation()=t.head<2>();
    float c = cos(t(2));
    float s = sin(t(2));
    T.linear() << c, -s, s, c;
    return T;
}

inline Eigen::Isometry2d v2t(const Eigen::Vector3d& t){
    Eigen::Isometry2d T;
    T.setIdentity();
    T.translation()=t.head<2>();
    float c = cos(t(2));
    float s = sin(t(2));
    T.linear() << c, -s, s, c;
    return T;
}

inline Eigen::Vector3f t2v(const Eigen::Isometry2f& t){
    Eigen::Vector3f v;
    v.head<2>()=t.translation();
    v(2) = atan2(t.linear()(1,0), t.linear()(0,0));
    return v;
}

inline Eigen::Vector3d t2v(const Eigen::Isometry2d& t){
    Eigen::Vector3d v;
    v.head<2>()=t.translation();
    v(2) = atan2(t.linear()(1,0), t.linear()(0,0));
    return v;
}

inline Eigen::Matrix3f Rx(float rot_x){
    float c=cos(rot_x);
    float s=sin(rot_x);
    Eigen::Matrix3f R;
    R << 1,  0, 0,
        0,  c,  -s,
        0,  s,  c;
    return R;
}

inline Eigen::Matrix3f Ry(float rot_y){
    float c=cos(rot_y);
    float s=sin(rot_y);
    Eigen::Matrix3f R;
    R << c,  0,  s,
        0 , 1,  0,
        -s,  0, c;
    return R;
}

inline Eigen::Matrix3f Rz(float rot_z){
    float c=cos(rot_z);
    float s=sin(rot_z);
    Eigen::Matrix3f R;
    R << c,  -s,  0,
        s,  c,  0,
        0,  0,  1;
    return R;
}

inline Eigen::Matrix3d Rz(double rot_z){
    double c=cos(rot_z);
    double s=sin(rot_z);
    Eigen::Matrix3d R;
    R << c,  -s,  0,
        s,  c,  0,
        0,  0,  1;
    return R;
}

inline Eigen::Isometry2d dRz(double rot_z){
    Eigen::Isometry2d R;
    double c=cos(rot_z);
    double s=sin(rot_z);
    R.setIdentity();
    R.linear() << -s, -c, c, -s;
    return R;
}

inline Eigen::Isometry2f dRz(float rot_z){
    Eigen::Isometry2f R;
    float c=cos(rot_z);
    float s=sin(rot_z);
    R.setIdentity();
    R.linear() << -s, -c, c, -s;
    return R;
}


inline Eigen::Isometry3f v2tEuler(const Vector6f& v){
    Eigen::Isometry3f T;
    T.linear()=Rx(v[3])*Ry(v[4])*Rz(v[5]);
    T.translation()=v.head<3>();
    return T;
}

inline Eigen::Matrix3f skew(const Eigen::Vector3f& v){
    Eigen::Matrix3f S;
    S << 0, -v[2], v[1],
        v[2], 0, -v[0],
        -v[1], v[0], 0;
    return S;
}




    
class MapPoint{
    private:

        
        int _pose_index;            // pose index inside the Vector3fVector poses 
        int _point_index;           // point index inside the Vector2fVector points.
        int _normal_index;          // normal index inside the Vector2fVecto normals.

        int _local_correspondences_index;
        int _global_correspondences_index;
    
        size_t _num_local_correspondences;
        size_t _num_global_correspondences;

        int _points_2dtree_index;
        int _points_4dtree_index;


    public:
        
        MapPoint(){
            _pose_index = -1;
            _point_index = -1;
            _normal_index = -1;

            _local_correspondences_index = -1;
            _global_correspondences_index = -1;
            _num_local_correspondences = 0;
            _num_global_correspondences = 0;

            _points_2dtree_index = -1;
            _points_4dtree_index = -1;
        };

        ~MapPoint(){};
        
        void set_pose_index(int pose_index){
            _pose_index = pose_index;
        }

        void set_point_index(int point_index){
            _point_index = point_index;

        }

        void set_normal_index(int normal_index){
            _normal_index = normal_index;
        }


        void set_local_correspondences_index(int local_correspondences_index, int num_local_correspondences){
            _local_correspondences_index = local_correspondences_index;
            _num_local_correspondences = num_local_correspondences;
        }
        
        void set_global_correspondences_index(int global_correspondences_index, int num_global_correspondences){
            _global_correspondences_index = global_correspondences_index;
            _num_global_correspondences = num_global_correspondences;
        }

        
        void clear_global_correspondeces(){
            _global_correspondences_index = -1;
            _num_global_correspondences = 0;
        }

        void set_points_2dtree_index(int points_2dtree_index) {
            _points_2dtree_index = points_2dtree_index;
        }

        void set_points_4dtree_index(int points_4dtree_index) {
            _points_4dtree_index = points_4dtree_index;
        }
        
        
        // getter
        int& pose_index() {return _pose_index;} 
        int& point_index() {return _point_index;} 
        int& normal_index() {return _normal_index;} 
        
        int& local_correspondences_index() {return _local_correspondences_index;} 
        int& global_correspondences_index() {return _global_correspondences_index;} 
        
        int& points_2dtree_index() {return _points_2dtree_index;}
        int& points_4dtree_index() {return _points_4dtree_index;}
        
        bool has_normal()
        { 
            if(_normal_index != -1) return true;
            else return false;
        }

        int num_local_correspondences(){ return _num_local_correspondences;}

        int num_global_correspondences(){ return _num_global_correspondences;}
        

};


