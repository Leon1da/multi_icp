#pragma once
#include <iostream>
#include <unistd.h>
#include <istream>
#include <iterator>
#include <vector>
#include <iomanip>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Cholesky>
#include <Eigen/StdVector>
#include "opencv2/opencv.hpp"

using namespace std;
using namespace cv;


typedef std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > Vector3fVector;
typedef std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f> > Vector2fVector;

typedef std::vector<Eigen::Isometry2f, Eigen::aligned_allocator<Eigen::Isometry2f> > Isometry2fVector;



typedef Eigen::Matrix<float, 2, 1> Vector2f;

typedef Eigen::Matrix<float, 2, 3> Matrix2_3f;
typedef Eigen::Matrix<float, 3, 3> Matrix3f;
typedef Eigen::Matrix<float, 3, 1> Vector3f;

typedef Eigen::Matrix<float, 2, 2> Matrix2f;
typedef Eigen::Matrix<float, 2, 4> Matrix2_4f;

typedef Eigen::Matrix<float, 4, 4> Matrix4f;
typedef Eigen::Matrix<float, 4, 1> Vector4f;

typedef Eigen::Matrix<float, 3, 6> Matrix3_6f;

typedef Eigen::Matrix<float, 6, 6> Matrix6f;
typedef Eigen::Matrix<float, 6, 1> Vector6f;

typedef Eigen::Matrix<float, 6, 3> Matrix6_3f;


typedef std::vector<float> FloatVector;
typedef std::pair<float, float> FloatPair;
typedef std::vector<int> IntVector;
typedef std::pair<int,int> IntPair;
typedef std::vector<IntPair > IntPairVector;
typedef std::pair<Eigen::Vector2f, Eigen::Vector2f> Vector2fPair;

class Point2D : public std::array<float, 2>{
    
        
    public:

        // dimension of space (or "k" of k-d tree)
        // KDTree class accesses this member
        static const int DIM = 2;

        // the constructors
        Point2D() {}
        
        Point2D(float x, float y) {
            (*this)[0] = x;
            (*this)[1] = y;
            

        }
        
        Point2D(Eigen::Vector2f point)
        { 
            (*this)[0] = point.x();
            (*this)[1] = point.y();
            
        }

        float x(){
            return (*this)[0];
        }

        float y(){
            return (*this)[1];
        }

};

class Pose2D{
    
    private:

        Eigen::Vector3f _pose;

        
    public:

        Pose2D(){

        }

        Pose2D(float x, float y, float theta){
            _pose = Eigen::Vector3f(x, y, theta);
        }

        Pose2D(Eigen::Vector3f pose){
            _pose = pose;
        }

        float x(){
            return _pose.x();
        }

        float y(){
            return _pose.y();
        }

        float theta(){
            return _pose.z();
        }

};

class Line2D{
    private:
        Point2D _src;
        Point2D _dst;
    public:
        
        Line2D(Point2D src, Point2D dst){
            _src = src;
            _dst = dst;
        }

        Point2D src(){
            return _src;
        }

        Point2D dst(){
            return _dst;
        }

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

inline Eigen::Vector3f t2v(const Eigen::Isometry2f& t){
    Eigen::Vector3f v;
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

