#include "defs.h"

class MultiICPSolver{
    
    private:

      bool errorAndJacobian(Eigen::Vector2f& error,
          Matrix2_3f& jacobian,
          const Eigen::Vector2f& point,
          const Eigen::Vector2f& point_normal,
          const Eigen::Vector2f& measurement,
          const Eigen::Vector2f& measurement_normal);

      void linearize(const IntPairVector& point_point_correspondences,  const IntPairVector& pose_point_correspondences, bool keep_outliers);

              
      Isometry2fVector* _state;                  //< this will hold our state
      float _kernel_thereshold;        //< threshold for the kernel
      float _damping;                  //< damping, to slow the solution
      int _min_num_inliers;            //< if less inliers than this value, the solver stops
      const Vector2fVector* _points;
      const Vector2fVector* _normals;
      const Vector3fVector* _poses;
      Eigen::MatrixXf _H;
      Eigen::VectorXf _b;
      float _chi_inliers;
      float _chi_outliers;
      int _num_inliers;
      int _num_outliers;

    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

      //! ctor
      MultiICPSolver();

      //! init method, call it at the beginning
      //! @param state: the state
      //! @param poses: the poses of the world
      //! @param points: the points of the world
      void init(Isometry2fVector& state,
          const Vector3fVector& poses,
          const Vector2fVector& points,
          const Vector2fVector& normals);

      inline float kernelThreshold() const {return _kernel_thereshold;}

      inline void setKernelThreshold(float kernel_threshold) 
      {_kernel_thereshold=kernel_threshold;}


    
      //! accessor to the state
      Isometry2fVector* state() {return _state;}

      //! chi square of the "good" points
      const float chiInliers() const {return _chi_inliers;}
      
      //! chi square of the "bad" points
      const float chiOutliers() const {return _chi_outliers;}
      
      //! number of inliers (an inlier is a point whose error is below kernel threshold)
      const int numInliers() const {return _num_inliers;}
      
      //! number of inliers (an inlier is a point whose error is below kernel threshold)
      const int numOutliers() const {return _num_outliers;}
      
      //! performs one iteration of optimization
      //! @param point_point_correspondences: the point_point_correspondences (first: measurement, second:model);
      //! param keep_outliers: if true, the outliers are considered in the optimization 
      //! (but cut by the kernel)
      bool oneRound(const IntPairVector& point_point_correspondences, const IntPairVector& pose_point_correspondences, bool keep_outliers);

};



  
MultiICPSolver::MultiICPSolver(){
  _state=0;
  _poses=0;
  _points=0;
  _normals=0;
  _damping=1;
  _min_num_inliers=0;
  _num_inliers=0;
  _num_outliers=0;
  _kernel_thereshold=0.00001;
}

void MultiICPSolver::init(Isometry2fVector& state, const Vector3fVector& poses, const Vector2fVector& points, const Vector2fVector& normals){
  _state=&state;
  _poses=&poses;
  _points=&points;
  _normals=&normals;
  
  size_t num_poses = poses.size();
  cout << num_poses << endl;
  for (size_t i = 0; i < num_poses; i++) (*_state)[i].setIdentity();
  
  _H.resize(num_poses*3, num_poses*3);
  _b.resize(num_poses*3);

  cout << "H dimension: " << _H.rows() << " " << _H.cols() << endl;
  cout << "b dimension: " << _b.rows() << " " << _b.cols() << endl;

  return;
  
}



bool MultiICPSolver::errorAndJacobian(Eigen::Vector2f& error,
                                  Matrix2_3f& jacobian,
                                  const Eigen::Vector2f& point,
                                  const Eigen::Vector2f& normal,
                                  const Eigen::Vector2f& measurement, 
                                  const Eigen::Vector2f& measurement_normal){
  error = point - measurement;
  error = error * (normal.x()*measurement_normal.y()-measurement_normal.x()*normal.y());
  // compute the jacobian of the transformation
  Matrix2_3f J=Eigen::Matrix<float,2,3>::Zero();
  J.block<2,2>(0,0).setIdentity();
  J.block<2,1>(0,2) = Vector2f(-point.y(), point.x()); 
  jacobian=J;
  return true;
}



void MultiICPSolver::linearize(const IntPairVector& point_point_correspondences, const IntPairVector& pose_point_correspondences, bool keep_outliers){
  
  // _H.setZero();
  _H.setIdentity();
  _b.setZero();
  
  _num_inliers=0;
  _chi_inliers=0;
  _num_outliers=0;
  _chi_outliers=0;
    
  for (const IntPair& correspondence: point_point_correspondences){

    int curr_point_id, ref_point_id, curr_pose_id, ref_pose_id;
    
    curr_point_id = correspondence.first;
    curr_pose_id = pose_point_correspondences[curr_point_id].first;
    ref_point_id = correspondence.second;
    ref_pose_id = pose_point_correspondences[ref_point_id].first;

    Eigen::Vector2f e;
    Matrix2_3f J;
    if(!errorAndJacobian(e, J, (*_points)[curr_point_id], (*_normals)[curr_point_id], (*_points)[ref_point_id], (*_normals)[ref_point_id])) continue;
    
    float chi=e.dot(e);
    float lambda=1;
    bool is_inlier=true;
    if (chi > _kernel_thereshold){
      lambda=sqrt(_kernel_thereshold/chi);
      is_inlier=false;
      _chi_outliers+=chi;
      _num_outliers++;
    } else {
      _chi_inliers+=chi;
      _num_inliers++;
    }

    // cout  << '\r' 
    //       << "landmark [ " << curr_point_id << " " << ref_point_id 
    //       << " ] pose [ " << curr_pose_id << " " << ref_pose_id << " ]" 
    //       << flush;
   
    
    if (is_inlier || keep_outliers){
      _H.block<3, 3>(curr_pose_id*3, ref_pose_id*3) += J.transpose()*J*lambda;
      _H.block<3, 3>(curr_pose_id*3, ref_pose_id*3) += (J.transpose()*J*lambda).transpose();
      _b.segment(curr_pose_id*3, 3) += J.transpose()*e*lambda;
    }

    // if (is_inlier || keep_outliers){
    //   _H+=J.transpose()*J*lambda;
    //   _b+=J.transpose()*e*lambda;
    // }
  }

  cout << endl;

}

bool MultiICPSolver::oneRound(const IntPairVector& point_point_correspondences, const IntPairVector& pose_point_correspondences,bool keep_outliers){

  linearize(point_point_correspondences, pose_point_correspondences, keep_outliers);

  if(_num_inliers<_min_num_inliers) {
    cerr << "too few inliers, skipping" << endl;
    return false;
  }

  Eigen::VectorXf dx = _H.ldlt().solve(-_b);

  for (size_t i = 0; i < (*_state).size(); i++){
    Vector3f vector = dx.segment(i*3, 3);
    (*_state)[i] = v2t(vector);
  }

  return true;
}

