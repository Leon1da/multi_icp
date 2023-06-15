#include "defs.h"


  /**
     Solver for point-to-point problem.
     Implements a least squares solver using translation+euler angles as minimal parameterization
     A simple saturating robust kernel, and an adjustable damping coefficient;
     To use it:
     - create an object
     - initialize it passing:
       - the image points (that represent the measurements)
       - the world points (that represent the model)
       - A state whose pose is initialized at initial_guess
     - call oneRound(<correspondences>) a bunch of times, with the correspondences returned by the finder;
       at each call, the solution will be subject to one ls operation
   */
  class ICPSolver{
    
    private:

      bool errorAndJacobian(Eigen::Vector2f& error,
          Matrix2_3f& jacobian,
          const Eigen::Vector2f& point,
          const Eigen::Vector2f& normal,
          const Eigen::Vector2f& measurement,
          const Eigen::Vector2f& measurement_normal);

      void linearize(const IntPairVector& correspondences, bool keep_outliers);

              
      Eigen::Isometry2f* _state;                  //< this will hold our state
      float _kernel_thereshold;        //< threshold for the kernel
      float _damping;                  //< damping, to slow the solution
      int _min_num_inliers;            //< if less inliers than this value, the solver stops
      const Vector3fVector* _poses;
      const Vector2fVector* _points;      
      const Vector2fVector* _normals;      
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

