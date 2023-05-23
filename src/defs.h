#pragma once
#include <iostream>
#include <unistd.h>
#include <istream>
#include <iterator>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Cholesky>
#include <Eigen/StdVector>
#include "opencv2/opencv.hpp"

using namespace std;
using namespace cv;

// user-defined point type
// inherits std::array in order to use operator[]
class Point2D : public std::array<float, 2>
{

    private:
        float _x;
        float _y;
        // int _pose_id;

    public:

        // dimension of space (or "k" of k-d tree)
        // KDTree class accesses this member
        static const int DIM = 2;

        // the constructors
        Point2D() {}

        Point2D(float x, float y)
        { 
            (*this)[0] = x;
            (*this)[1] = y;
            _x = x;
            _y = y;
        }

        Point2D(Eigen::Vector2f point)
        { 
            (*this)[0] = point.x();
            (*this)[1] = point.y();
            _x = point.x();
            _y = point.y();

        }

        float x(){
            return _x;
        }

        float y(){
            return _y;
        }

        // float pose_id(){
        //     return _pose_id;
        // }

};

class Pose2D{
    
    private:

        float _x;
        float _y;
        float _theta;
    
    public:

        Pose2D(float x, float y, float theta){
            _x = x;
            _y = y;
            _theta = theta;
        }

        Pose2D(Eigen::Vector3f pose){
            _x = pose.x();
            _y = pose.y();
            _theta = pose.z();
        }

        float x(){
            return _x;
        }

        float y(){
            return _y;
        }

        float theta(){
            return _theta;
        }

};
