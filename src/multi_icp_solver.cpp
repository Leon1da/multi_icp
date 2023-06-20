#include "defs.h"

class MultiICPSolver{
    
    private:
              
      Isometry2fVector* _state;                  //< this will hold our state
      float _kernel_thereshold;        //< threshold for the kernel
      float _damping;                  //< damping, to slow the solution
      int _min_num_inliers;            //< if less inliers than this value, the solver stops
      const vector<Vector2fVector>* _points;
      const vector<Vector2fVector>* _normals;
      const Vector3fVector* _poses;
      Eigen::MatrixXf _H;
      Eigen::VectorXf _b;
      float _chi_inliers;
      float _chi_outliers;
      int _num_inliers;
      int _num_outliers;

      bool errorAndJacobian(float& error, Matrix1_3f& Ji, Matrix1_3f& Jj, const Correspondence& correspondence);

      void linearize(const vector<Correspondence>& corresponendeces, bool keep_outliers);


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
          const vector<Vector2fVector>& points,
          const vector<Vector2fVector>& normals);

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
      bool oneRound(const vector<Correspondence>& correspondences,  bool keep_outliers);

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
  _kernel_thereshold=0.01;
}

void MultiICPSolver::init(Isometry2fVector& state, const Vector3fVector& poses, const vector<Vector2fVector>& points, const vector<Vector2fVector>& normals){
  _state=&state;
  _poses=&poses;
  _points=&points;
  _normals=&normals;
  
  size_t num_poses = poses.size();
  
  _H.resize(num_poses*3, num_poses*3);
  _b.resize(num_poses*3);

  cout << "H dimension: " << _H.rows() << " " << _H.cols() << endl;
  cout << "b dimension: " << _b.rows() << " " << _b.cols() << endl;

  return;
  
}



bool MultiICPSolver::errorAndJacobian(float& error,
                                  Matrix1_3f& Ji,
                                  Matrix1_3f& Jj,
                                  const Correspondence& correspondence){

    int cur_pose_index = correspondence.first.first;
    int cur_point_index = correspondence.first.second;

    int ref_pose_index = correspondence.second.first;
    int ref_point_index = correspondence.second.second;

    Eigen::Isometry2f Xi = v2t((*_poses)[cur_pose_index]);
    Eigen::Vector2f pki = (*_points)[cur_pose_index][cur_point_index];
    Eigen::Vector2f nki = (*_normals)[cur_pose_index][cur_point_index];

    Eigen::Isometry2f Xj = v2t((*_poses)[ref_pose_index]);
    Eigen::Vector2f pkj = (*_points)[ref_pose_index][ref_point_index];
    Eigen::Vector2f nkj = (*_normals)[ref_pose_index][ref_point_index];
    
    Vector2f alpha = (Xi*pki - Xj*pkj);
    Vector2f beta = (Xi.rotation()*nki + Xj.rotation()*nkj);
    error = alpha.dot(beta);

    Eigen::Isometry2f dRi = dRz((*_poses)[cur_pose_index].z());
    Eigen::Isometry2f dRj = dRz((*_poses)[ref_pose_index].z());

    // compute the jacobian of the transformation
    Ji = Eigen::Matrix<float, 1, 3>(beta.x(), beta.y(), beta.dot(dRi * pki) + alpha.dot(dRi * nki));
    Jj = Eigen::Matrix<float, 1, 3>(-beta.x(), -beta.y(), -beta.dot(dRj * pkj) + alpha.dot(dRj * nkj));
    
    return true;
}



void MultiICPSolver::linearize(const vector<Correspondence>& correspondences, bool keep_outliers){
  
  _H.setZero();
  _b.setZero();

  _num_inliers=0;
  _chi_inliers=0;
  _num_outliers=0;
  _chi_outliers=0;

  for (const Correspondence& correspondence: correspondences){

    int cur_pose_index = correspondence.first.first;
    int cur_point_index = correspondence.first.second;

    int ref_pose_index = correspondence.second.first;
    int ref_point_index = correspondence.second.second;

    float e;
    Matrix1_3f J_cur;
    Matrix1_3f J_ref;

    if(!errorAndJacobian(e, J_cur, J_ref, correspondence)) continue;
    
    float chi = e*e;
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

    
    if (is_inlier || keep_outliers){
      _H.block<3, 3>(cur_pose_index * 3, cur_pose_index * 3) += J_cur.transpose()*J_cur*lambda; // Ji' * Ji
      _H.block<3, 3>(cur_pose_index * 3, ref_pose_index * 3) += J_cur.transpose()*J_ref*lambda; // Ji' * Jj
      _H.block<3, 3>(ref_pose_index * 3, cur_pose_index * 3) += J_ref.transpose()*J_cur*lambda; // Jj' * Jj
      _H.block<3, 3>(ref_pose_index * 3, ref_pose_index * 3) += J_ref.transpose()*J_ref*lambda; // Jj' * Ji

      _b.block<3, 1>(cur_pose_index * 3, 0) += J_cur.transpose()*e*lambda;  // Ji' * e
      _b.block<3, 1>(ref_pose_index * 3, 0) += J_ref.transpose()*e*lambda;  // Jj' * e

    }

  }


}

bool MultiICPSolver::oneRound(const vector<Correspondence>& correspondences, bool keep_outliers){

  cout << "linearize.." << endl;
  linearize(correspondences, keep_outliers);

  if(_num_inliers<_min_num_inliers) {
    cerr << "too few inliers, skipping" << endl;
    return false;
  }

  
  cout << "solving system.." << endl;
  Eigen::VectorXf dx = _H.ldlt().solve(-_b);

  
  cout << "updating state.." << endl;
  for (size_t i = 0; i < (*_state).size(); i++){
    Vector3f vector = dx.segment(i*3, 3);
    (*_state)[i] = v2t(vector);
  }

  return true;
}

