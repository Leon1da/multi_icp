# include "defs.h"

// Vector2d cv_point_to_eigen_point(Point p);
// Point eigen_point_to_cv_point(Vector2d p);

bool estimate_normal(Vector2dVector &points, IntVector& indices, double &angle);
bool estimate_normal(Vector2dVector &points, IntVector& indices, Vector2d &normal);



