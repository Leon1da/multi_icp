# include "defs.h"

bool estimate_line_fitting_points(Vector2fVector &points, IntVector ids, float &angle);
bool estimate_normal_of_line_fitting_points(Vector2fVector &points, IntVector ids, float &angle);
bool normal_of_points(vector<Vector2fVector>& points, IntPairVector& points_cloud_indices, Vector2f& normal);
bool normal_of_points(vector<Vector2fVector> &points, IntPairVector& points_cloud_indices, float &angle);

bool compute_covariance(vector<Vector2fVector>& points, IntPairVector& points_cloud_indices, Matrix2f& covariance);

