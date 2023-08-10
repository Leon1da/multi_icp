# include "defs.h"

// Vector2d cv_point_to_eigen_point(Point p);
// Point eigen_point_to_cv_point(Vector2d p);

bool estimate_normal(Vector2dVector &points, IntVector& indices, double &angle);
bool estimate_normal(Vector2dVector &points, IntVector& indices, Vector2d &normal);





bool read_configuration(
    int argc, char** argv,
    string& dataset_filename,
    int& dataset_from_record_number,
    int& dataset_num_records,
    int& points_kdtree_dim,
    int& poses_kdtree_dim,
    int& min_poses_correspondences,
    int& min_local_correspondences,
    double& kernel_threshold,
    double& damping,
    bool& keep_outliers
);

bool print_configuration(
    string& dataset_filename,
    int& dataset_from_record_number,
    int& dataset_num_records,
    int& points_kdtree_dim,
    int& poses_kdtree_dim,
    int& min_poses_correspondences,
    int& min_local_correspondences,
    double& kernel_threshold,
    double& damping,
    bool& keep_outliers

);