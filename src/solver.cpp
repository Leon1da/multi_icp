#include "defs.h"

class NICP2dSolver{
    
    private:
                               //< this will hold our state
      double _kernel_thereshold;        //< threshold for the kernel
      double _damping;                  //< damping, to slow the solution
      int _min_num_inliers;            //< if less inliers than this value, the solver stops
      
      const Vector2dVector* _points;
      const Vector2dVector* _normals;
      const Vector3dVector* _poses;

      
      Eigen::VectorXd* _state;
      
      size_t _state_dim;
      Eigen::VectorXd _b;
      
      double _chi_inliers;
      double _chi_outliers;
      int _num_inliers;
      int _num_outliers;

      bool compute_error_and_jacobian(double& error, Matrix1_3d& jacobian, const TriplePair& correspondence);

      void linearize(const vector<TriplePairVector>& corresponendeces, vector<Eigen::Triplet<double>>& coefficients, bool keep_outliers);

      void KahanSummation(double &sum, double &correction, double &value){
        double y = value - correction;
        double t = sum + y;
        correction = (t - sum) - y;
        sum = t;
      }

      void KahanMatrixSummation(Eigen::Matrix3d& sum, Eigen::Matrix3d& correction, Eigen::Matrix3d& value){
        for (size_t i = 0; i < 3; i++)
          for (size_t j = 0; j < 3; j++)
            KahanSummation(sum(i, j), correction(i, j), value(i, j));
      }
      
      void KahanVectorSummation(Eigen::Vector3d& sum, Eigen::Vector3d correction, Eigen::Vector3d value){
        for (size_t i = 0; i < 3; i++)
          KahanSummation(sum(i), correction(i), value(i));
      }


      

    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

      //! ctor
      NICP2dSolver();

      //! init method, call it at the beginning
      //! @param poses: the poses of the world
      //! @param points: the points of the world
      void init(
          Eigen::VectorXd& state,
          const Vector3dVector& poses,
          const Vector2dVector& points,
          const Vector2dVector& normals,
          double kernel_threshold=1.0, 
          double damping=1.0
          );

      inline float kernelThreshold() const {return _kernel_thereshold;}

      inline void setKernelThreshold(float kernel_threshold) 
      {_kernel_thereshold=kernel_threshold;}


    
      //! accessor to the state
      Eigen::VectorXd* state() {return _state;}

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
      bool oneRound(const vector<TriplePairVector>& correspondences, bool keep_outliers);

};
 
NICP2dSolver::NICP2dSolver(){
  _poses=0;
  _points=0;
  _normals=0;
  
  _min_num_inliers=0;
  _num_inliers=0;
  _num_outliers=0;
  
  _damping=1;
  _kernel_thereshold=1;
  
}

void NICP2dSolver::init(Eigen::VectorXd& state, const Vector3dVector& poses, const Vector2dVector& points, const Vector2dVector& normals, double kernel_threshold, double damping){
  _state = &state;
  _poses = &poses;
  _points = &points;
  _normals = &normals;
  _damping = damping;
  _kernel_thereshold = kernel_threshold;
  
  _state_dim = poses.size();
  _b.resize(_state_dim*3);
}

bool NICP2dSolver::compute_error_and_jacobian(double& error,
                                  Matrix1_3d& jacobian,
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

    // normals in world frame
    Vector2d w_ni = v2t(Xi).rotation()*ni;
    Vector2d w_nj = v2t(Xj).rotation()*nj;
    
    // points in world frames
    Vector2d w_pi = v2t(Xi)*pi;
    Vector2d w_pj = v2t(Xj)*pj;

    
                      // [ni_x + nj_x, ni_y + nj_y,     ni_y*pj_x     - ni_x*pj_y     - nj_x*pi_y     + nj_y*pi_x]
    jacobian = Matrix1_3d((w_ni + w_nj).x(), (w_ni + w_nj).y(), w_ni.y()*w_pj.x() - w_ni.x()*w_pj.y() - w_nj.x()*w_pi.y() + w_nj.y()*w_pi.x());
    error = (w_pi - w_pj).dot(w_ni + w_nj);

    // Vector2d normal_sum = w_ni + w_ni;
    // Vector2d points_diff = w_pi - w_pj;

    // jacobian = Matrix1_3d(normal_sum.x(), normal_sum.y(), w_pi.x()*w_nj.y() - w_pi.y()*w_nj.x() + w_pj.x()*w_ni.y() - w_pj.y()*w_ni.x());
    
    // // compute the jacobian of the transformation
    // Ji = Matrix1_3d(beta.x(), beta.y(), pi_w.x()*nj_w.y()-pi_w.y()*nj_w.x() + pj_w.x()*ni_w.y()-pj_w.y()*ni_w.x());
    // Jj = Matrix1_3d(-beta.x(), -beta.y(), -(pi_w.x()*nj_w.y()-pi_w.y()*nj_w.x() + pj_w.x()*ni_w.y()-pj_w.y()*ni_w.x()));
    
    // error = (points_diff).dot(normal_sum);

    return true;
}

void NICP2dSolver::linearize(const vector<TriplePairVector>& correspondences, vector<Eigen::Triplet<double>> &coefficients, bool keep_outliers){

  _num_inliers=0;
  _chi_inliers=0;
  _num_outliers=0;
  _chi_outliers=0;

  _b.setZero();

  for (size_t block_index = 0; block_index < correspondences.size(); block_index++)
  {
    
    TriplePairVector pose_pose_correspondences = correspondences[block_index];
    
    Eigen::Matrix3d H;
    H.setZero();
    
    Eigen::Matrix3d cH;
    cH.setZero();
    
    Eigen::Vector3d b;
    b.setZero();
    
    Eigen::Vector3d cb;
    cb.setZero();
    

    if (!pose_pose_correspondences.size()){
      cout << "No correspondences found ( " << block_index << " / " << correspondences.size() << " )" << endl;
      continue;
    }

    int cur_pose_index, ref_pose_index; 
    for (size_t index = 0; index < pose_pose_correspondences.size(); index++)
    {

      
      TriplePair correspondence = pose_pose_correspondences[index];
      cur_pose_index = get<0>(correspondence.first);
      ref_pose_index = get<0>(correspondence.second);
      
      double e;
      Matrix1_3d J;
      
      if (!compute_error_and_jacobian(e, J, correspondence)) continue;
      
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

        Eigen::Matrix3d Hv = J.transpose()*J*lambda;
        
        KahanMatrixSummation(H, cH, Hv);
        
        Eigen::Vector3d bv = J.transpose()*e*lambda;

        KahanVectorSummation(b, cb, bv);

      }
    }
            
    for (size_t i = 0; i < 3; i++)
    {
      for (size_t j = 0; j < 3; j++)
      {
        if (cur_pose_index) coefficients.push_back(Eigen::Triplet<double>(cur_pose_index*3+i, cur_pose_index*3+j, H.coeff(i, j)));
        if (cur_pose_index && ref_pose_index)
        {
          coefficients.push_back(Eigen::Triplet<double>(cur_pose_index*3+i, ref_pose_index*3+j, -H.coeff(i, j)));
          coefficients.push_back(Eigen::Triplet<double>(ref_pose_index*3+i, cur_pose_index*3+j, -H.coeff(i, j)));
        }
        if (ref_pose_index) coefficients.push_back(Eigen::Triplet<double>(ref_pose_index*3+i, ref_pose_index*3+j, H.coeff(i, j)));
      }
    }

    if(cur_pose_index) _b.block(cur_pose_index*3, 0, 3, 1) += b;  
    if(ref_pose_index) _b.block(ref_pose_index*3, 0, 3, 1) += -b;

  }
  
}

bool NICP2dSolver::oneRound(const vector<TriplePairVector>& correspondences, bool keep_outliers){

  std::vector<Eigen::Triplet<double>> coefficients;

  linearize(correspondences, coefficients, keep_outliers);

  if(_num_inliers <_min_num_inliers) {
    cerr << "too few inliers, skipping" << endl;
    return false;
  }


  Eigen::SparseMatrix<double> sparse_system(_state_dim * 3, _state_dim * 3);
  
  // damping
  for (size_t i = 0; i < _state_dim * 3; i++) coefficients.push_back(Eigen::Triplet<double>(i, i, _damping));

  Eigen::VectorXd dx;
  
  // build sparse system using saved triplets
  sparse_system.setFromTriplets(coefficients.begin(), coefficients.end());
  
  Eigen::CholmodSupernodalLLT<Eigen::SparseMatrix<double>> solver(sparse_system);  
  dx = solver.solve(-_b);                                                         
  
  *_state = dx;

  return true;
}

