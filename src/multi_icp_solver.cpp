#include "defs.h"

class MultiICPSolver{
    
    private:
              
      Isometry2dVector* _state;                  //< this will hold our state
      double _kernel_thereshold;        //< threshold for the kernel
      double _damping;                  //< damping, to slow the solution
      int _min_num_inliers;            //< if less inliers than this value, the solver stops
      const Vector2dVector* _points;
      const Vector2dVector* _normals;
      const Vector3dVector* _poses;
      Eigen::MatrixXd _H;
      Eigen::VectorXd _b;
      double _chi_inliers;
      double _chi_outliers;
      int _num_inliers;
      int _num_outliers;

      bool errorAndJacobian(double& error, Matrix1_3d& Ji, Matrix1_3d& Jj, const TriplePair& correspondence);

      void linearize(const TriplePairVector& corresponendeces, bool keep_outliers, vector<Eigen::Triplet<double>>& coefficients);


    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

      //! ctor
      MultiICPSolver();

      //! init method, call it at the beginning
      //! @param state: the state
      //! @param poses: the poses of the world
      //! @param points: the points of the world
      void init(Isometry2dVector& state,
          const Vector3dVector& poses,
          const Vector2dVector& points,
          const Vector2dVector& normals,
          double kernel_threshold=1.0, 
          double damping=1.0);

      inline float kernelThreshold() const {return _kernel_thereshold;}

      inline void setKernelThreshold(float kernel_threshold) 
      {_kernel_thereshold=kernel_threshold;}


    
      //! accessor to the state
      Isometry2dVector* state() {return _state;}

      //! chi square of the "good" points
      const double chiInliers() const {return _chi_inliers;}
      
      //! chi square of the "bad" points
      const double chiOutliers() const {return _chi_outliers;}
      
      //! number of inliers (an inlier is a point whose error is below kernel threshold)
      const int numInliers() const {return _num_inliers;}
      
      //! number of inliers (an inlier is a point whose error is below kernel threshold)
      const int numOutliers() const {return _num_outliers;}
      
      //! performs one iteration of optimization
      //! param keep_outliers: if true, the outliers are considered in the optimization 
      //! (but cut by the kernel)
      bool oneRound(const TriplePairVector& correspondences, bool keep_outliers);

};



  
MultiICPSolver::MultiICPSolver(){
  _state=0;
  _poses=0;
  _points=0;
  _normals=0;
  
  _min_num_inliers=0;
  _num_inliers=0;
  _num_outliers=0;

  _damping=1;
  _kernel_thereshold=1;
}

void MultiICPSolver::init(Isometry2dVector& state, const Vector3dVector& poses, const Vector2dVector& points, const Vector2dVector& normals, double kernel_threshold, double damping){
  _state = &state;
  _poses = &poses;
  _points = &points;
  _normals = &normals;
  _damping = damping;
  _kernel_thereshold = kernel_threshold;
  
  size_t num_poses = poses.size();
  
  _H.resize(num_poses*3, num_poses*3);
  _b.resize(num_poses*3);

  cout << "H dimension: " << _H.rows() << " " << _H.cols() << endl;
  cout << "b dimension: " << _b.rows() << " " << _b.cols() << endl;

  return;
  
}



bool MultiICPSolver::errorAndJacobian(double& error,
                                  Matrix1_3d& Ji,
                                  Matrix1_3d& Jj,
                                  const TriplePair& correspondence){

    int cur_pose_index = get<0>(correspondence.first);
    int cur_point_index = get<1>(correspondence.first);
    int cur_normal_index = get<2>(correspondence.first);

    int ref_pose_index = get<0>(correspondence.second);
    int ref_point_index = get<1>(correspondence.second);
    int ref_normal_index = get<2>(correspondence.second);
 
    Vector3d Xi = (*_poses)[cur_pose_index];
    Vector2d pi = (*_points)[cur_point_index];
    Vector2d ni = (*_normals)[cur_normal_index];
    
    Vector3d Xj = (*_poses)[ref_pose_index];
    Vector2d pj = (*_points)[ref_point_index];
    Vector2d nj = (*_normals)[ref_normal_index];

      
    Eigen::Isometry2d Xki = v2t(Xi);
    Eigen::Isometry2d Xkj = v2t(Xj);
    
    Vector2d alpha = (Xki*pi - Xkj*pj);
    Vector2d beta = (Xki.rotation()*ni + Xkj.rotation()*nj);
    
    Eigen::Isometry2d dRi = dRz(Xi.z());
    Eigen::Isometry2d dRj = dRz(Xj.z());

    // compute the jacobian of the transformation
    Ji = Matrix1_3d(beta.x(), beta.y(), beta.dot(dRi * pi) + alpha.dot(dRi * ni));
    Jj = Matrix1_3d(-beta.x(), -beta.y(), -beta.dot(dRj * pj) + alpha.dot(dRj * nj));

    error = alpha.dot(beta);

    return true;
}



void MultiICPSolver::linearize(const TriplePairVector& correspondences, bool keep_outliers, vector<Eigen::Triplet<double>> &coefficients){
  
  _H.setZero();
  _b.setZero();

  _num_inliers=0;
  _chi_inliers=0;
  _num_outliers=0;
  _chi_outliers=0;

  
  for (size_t index = 0; index < correspondences.size(); index++)
  {
    TriplePair correspondence = correspondences[index];
    
    int cur_pose_index = get<0>(correspondence.first);
    int ref_pose_index = get<0>(correspondence.second);
    
    double e;
    Matrix1_3d J_cur;
    Matrix1_3d J_ref;

    if( !errorAndJacobian(e, J_cur, J_ref, correspondence) ) continue;
    
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
      
      Eigen::Matrix3d Jii = J_cur.transpose()*J_cur*lambda;
      Eigen::Matrix3d Jij = J_cur.transpose()*J_ref*lambda;
      Eigen::Matrix3d Jji = J_ref.transpose()*J_cur*lambda;
      Eigen::Matrix3d Jjj = J_ref.transpose()*J_ref*lambda;
      
      for (size_t i = 0; i < 3; i++)
      {
        for (size_t j = 0; j < 3; j++)
        {
          coefficients.push_back(Eigen::Triplet<double>(cur_pose_index*3 + i, cur_pose_index*3 + j, Jii.coeff(i,j)));
          coefficients.push_back(Eigen::Triplet<double>(cur_pose_index*3 + i, ref_pose_index*3 + j, Jij.coeff(i,j)));
          coefficients.push_back(Eigen::Triplet<double>(ref_pose_index*3 + i, cur_pose_index*3 + j, Jji.coeff(i,j)));
          coefficients.push_back(Eigen::Triplet<double>(ref_pose_index*3 + i, ref_pose_index*3 + j, Jjj.coeff(i,j)));

        }
        
      }
      
      _H.block<3, 3>(cur_pose_index * 3, cur_pose_index * 3) += Jii; // Ji' * Ji
      _H.block<3, 3>(cur_pose_index * 3, ref_pose_index * 3) += Jij; // Ji' * Jj
      _H.block<3, 3>(ref_pose_index * 3, cur_pose_index * 3) += Jji; // Jj' * Ji
      _H.block<3, 3>(ref_pose_index * 3, ref_pose_index * 3) += Jjj; // Jj' * Jj

      _b.block<3, 1>(cur_pose_index * 3, 0) += J_cur.transpose()*e*lambda;  // Ji' * e
      _b.block<3, 1>(ref_pose_index * 3, 0) += J_ref.transpose()*e*lambda;  // Jj' * e

      
    }

  }


}

bool MultiICPSolver::oneRound(const TriplePairVector& correspondences, bool keep_outliers){

  std::vector<Eigen::Triplet<double>> coefficients;

  cout << "linearize.." << endl;
  linearize(correspondences,  keep_outliers, coefficients);

  if(_num_inliers <_min_num_inliers) {
    cerr << "too few inliers, skipping" << endl;
    return false;
  }

  size_t state_size = (*_state).size();
  
  // cout << _H << endl << endl << endl;

  Eigen::MatrixXd damping(3 * state_size, 3 * state_size);
  damping.setIdentity();
  damping = damping * _damping;

  _H += damping;


  // cout << _H << endl << endl << endl;
  
  cout << "solving system.." << endl;
  Eigen::VectorXd dx;

  // dx = _H.bdcSvd().solve(-_b);
  // cout << dx << endl << endl;

  // dx = _H.fullPivHouseholderQr().solve(-_b);
  // cout << dx << endl << endl;
  
  // dx = _H.colPivHouseholderQr().solve(-_b);
  // // cout << dx << endl << endl;

  // dx = _H.fullPivLu().solve(-_b);
  // // cout << dx << endl << endl;
  
  // dx = _H.colPivHouseholderQr().solve(-_b);
  // cout << dx << endl << endl;
  
  // dx = _H.ldlt().solve(-_b);
  // cout << dx << endl << endl;

  
  Eigen::SparseMatrix<double> _H_sparse(state_size*3, state_size*3);

  // damping
  for (size_t i = 0; i < state_size * 3; i++) coefficients.push_back(Eigen::Triplet<double>(i, i, _damping));
  
  // build sparse system using saved triplets
  _H_sparse.setFromTriplets(coefficients.begin(), coefficients.end());

  // cout << _H_sparse << endl << endl << endl;

  // Eigen::SimplicialCholesky<Eigen::SparseMatrix<double>> solver(_H_sparse);       // performs a Cholesky factorization of A
  // dx = solver.solve(-_b);                                                         // use the factorization to solve for the given right hand side
  // cout << dx << endl << endl;

  // Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver(_H_sparse);
  // dx = solver.solve(-_b);
  // cout << dx << endl << endl;

  Eigen::CholmodSupernodalLLT<Eigen::SparseMatrix<double>> solver(_H_sparse);
  dx = solver.solve(-_b);
  // cout << dx << endl << endl;

  
  
  cout << "updating state.." << endl;
  for (size_t i = 0; i < state_size; i++){
    Vector3d dx_pose = dx.segment(i*3, 3);
    (*_state)[i] = v2t(dx_pose);
  }
  
  return true;
}

