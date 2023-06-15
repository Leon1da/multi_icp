#include "defs.h"

class ICPSolver{
    
    private:

      bool errorAndJacobian(Eigen::Vector2f& error,
          Matrix2_3f& jacobian,
          const Eigen::Vector2f& point,
          const Eigen::Vector2f& point_normal,
          const Eigen::Vector2f& measurement,
          const Eigen::Vector2f& measurement_normal);

      void linearize(const IntPairVector& correspondences, bool keep_outliers);

              
      Eigen::Isometry2f* _state;                  //< this will hold our state
      float _kernel_thereshold;        //< threshold for the kernel
      float _damping;                  //< damping, to slow the solution
      int _min_num_inliers;            //< if less inliers than this value, the solver stops
      const Vector2fVector* _points;
      const Vector2fVector* _normals;
      const Vector3fVector* _poses;
      Matrix3f _H;
      Vector3f _b;
      float _chi_inliers;
      float _chi_outliers;
      int _num_inliers;

    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

      //! ctor
      ICPSolver();

      //! init method, call it at the beginning
      //! @param state: the state
      //! @param poses: the poses of the world
      //! @param points: the points of the world
      void init(Eigen::Isometry2f& state,
          const Vector3fVector& poses,
          const Vector2fVector& points,
          const Vector2fVector& normals);

      inline float kernelThreshold() const {return _kernel_thereshold;}

      inline void setKernelThreshold(float kernel_threshold) 
      {_kernel_thereshold=kernel_threshold;}


    
      //! accessor to the state
      Eigen::Isometry2f* state() {return _state;}

      //! chi square of the "good" points
      const float chiInliers() const {return _chi_inliers;}
      
      //! chi square of the "bad" points
      const float chiOutliers() const {return _chi_outliers;}
      
      //! number of inliers (an inlier is a point whose error is below kernel threshold)
      const int numInliers() const {return _num_inliers;}
      
      //! performs one iteration of optimization
      //! @param correspondences: the correspondences (first: measurement, second:model);
      //! param keep_outliers: if true, the outliers are considered in the optimization 
      //! (but cut by the kernel)
      bool oneRound(const IntPairVector& correspondences, bool keep_outliers);

};



  
ICPSolver::ICPSolver(){
  _state=0;
  _poses=0;
  _points=0;
  _normals=0;
  _damping=1;
  _min_num_inliers=0;
  _num_inliers=0;
  _kernel_thereshold=1000; // 33 pixels
}

void ICPSolver::init(Eigen::Isometry2f& state, const Vector3fVector& poses, const Vector2fVector& points, const Vector2fVector& normals){
  _state=&state;
  _poses=&poses;
  _points=&points;
  _normals=&normals;
}


bool ICPSolver::errorAndJacobian(Eigen::Vector2f& error,
                                  Matrix2_3f& jacobian,
                                  const Eigen::Vector2f& point,
                                  const Eigen::Vector2f& normal,
                                  const Eigen::Vector2f& measurement, 
                                  const Eigen::Vector2f& measurement_normal){

  error = point - measurement;
  
  // compute the jacobian of the transformation
  Matrix2_3f J=Eigen::Matrix<float,2,3>::Zero();
  J.block<2,2>(0,0).setIdentity();
  J.block<2,1>(0,2) = Vector2f(-point.y(), point.x()); 
  jacobian=J;
  return true;
}



void ICPSolver::linearize(const IntPairVector& correspondences, bool keep_outliers){
  _H.setZero();
  _b.setZero();
  _num_inliers=0;
  _chi_inliers=0;
  _chi_outliers=0;
  for (const IntPair& correspondence: correspondences){
    Eigen::Vector2f e;
    Matrix2_3f J;
    int curr_idx=correspondence.first;
    int ref_idx=correspondence.second;
    bool inside = errorAndJacobian(e, J, (*_points)[curr_idx], (*_normals)[curr_idx], (*_points)[ref_idx], (*_normals)[ref_idx]);
    if (!inside)
      continue;

    float chi=e.dot(e);
    float lambda=1;
    bool is_inlier=true;
    if (chi>_kernel_thereshold){
      lambda=sqrt(_kernel_thereshold/chi);
      is_inlier=false;
      _chi_outliers+=chi;
    } else {
      _chi_inliers+=chi;
      _num_inliers++;
    }
    
    if (is_inlier || keep_outliers){
      _H+=J.transpose()*J*lambda;
      _b+=J.transpose()*e*lambda;
    }
  }
}

bool ICPSolver::oneRound(const IntPairVector& correspondences, bool keep_outliers){
  linearize(correspondences, keep_outliers);
  _H+=Matrix3f::Identity()*_damping;
  if(_num_inliers<_min_num_inliers) {
    cerr << "too few inliers, skipping" << endl;
    return false;
  }
  //compute a solution
  Vector3f dx = _H.ldlt().solve(-_b);
  *_state = v2t(dx);
  return true;
}

