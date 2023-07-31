#include "defs.h"

  class MultiICPSolver{
    
    private:

      bool errorAndJacobian(double& error, Matrix1_3d& Ji, Matrix1_3d& Jj, const TriplePair& correspondence);

      void linearize(const TriplePairVector& correspondences, vector<Tripletd>& coefficients, bool keep_outliers);

      Isometry2dVector* _state;                   //< this will hold our state
      double _kernel_thereshold;                   //< threshold for the kernel
      double _damping;                             //< damping, to slow the solution
      int _min_num_inliers;                       //< if less inliers than this value, the solver stops
      const Vector3dVector* _poses;
      const Vector2dVector* _points;      
      const Vector2dVector* _normals;      
      Eigen::VectorXf _b;
      double _chi_inliers;
      double _chi_outliers;
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
      void init(Isometry2dVector& state,
          const Vector3dVector& poses,
          const Vector2dVector& points, 
          const Vector2dVector& normals,
          double kernel_threshold=0.05, 
          double damping=1.0
          );
    
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
      //! @param correspondences: the correspondences (first: measurement, second:model);
      //! param keep_outliers: if true, the outliers are considered in the optimization 
      //! (but cut by the kernel)
      bool oneRound(const TriplePairVector& correspondences, bool keep_outliers);

};

