#include "defs.h"

// line fitting point cloud
bool estimate_line_fitting_points(Vector2fVector &points, IntVector &ids, float &angle){

    int n_points = ids.size();
    Eigen::Vector2f mean(0, 0);
    for(auto id : ids) mean = mean + points[id];
    mean = mean / n_points;
    
    float num = 0, den = 0, m = 0;
    for(auto id : ids) 
    {
        num = num + (points[id].x() - mean.x()) * (points[id].y() - mean.y());
        den = den + pow(points[id].x() - mean.x(), 2);
    }
    
    if(!den) angle = 90 * M_PI / 180; // 90 degrees
    else if(!num) angle = 0; // 0 degrees
    else{
        m = num/den;
        angle = atan(m);
    }
    
    return true;

}

// normal of line fitting point cloud
bool estimate_normal_of_line_fitting_points(Vector2fVector &points, IntVector ids, float &angle){
    
    int n_points = ids.size();
    Eigen::Vector2f mean(0, 0);
    for(auto id : ids) mean = mean + points[id];
    mean = mean / n_points;
    
    float num = 0, den = 0, m = 0;
    for(auto id : ids) 
    {
        num = num + (points[id].x() - mean.x()) * (points[id].y() - mean.y());
        den = den + pow(points[id].x() - mean.x(), 2);
    }
    
    
    if(!den) angle = 90 * M_PI / 180; // 90 degrees
    else if(!num) angle = 0; // 0 degrees
    else{
        
        m = num / den; // slope of line
        m = -1/m; // slope of normal
        angle = atan(m);
    }
    return true;

}

// normal of line fitting point cloud
bool normal_of_points(vector<Vector2fVector> &points, IntPairVector& points_cloud_indices, float &angle){

    size_t num_points = points_cloud_indices.size();
    
    // cout << "num_points: " << endl << num_points << endl;
    Vector2f mean(0, 0);
    for (size_t index = 0; index < num_points; index++)
    {

        size_t pose_index = points_cloud_indices[index].first;
        size_t point_index = points_cloud_indices[index].second;
        mean = mean + points[pose_index][point_index];
        // cout << "[ " << pose_index << " " << point_index << " ]";
        // cout << "vector: " << endl << points[pose_index][point_index] << endl;
    }

    mean = mean / num_points;

    // cout << "Mean vector: " << endl << mean << endl;


    float num = 0, den = 0, m = 0;

    for (size_t index = 0; index < num_points; index++)
    {
        size_t pose_index = points_cloud_indices[index].first;
        size_t point_index = points_cloud_indices[index].second;

        Vector2f point_ = points[pose_index][point_index] - mean;

        num = num + point_.x() * point_.y();
        den = den + pow(point_.x(), 2);
    }

    
    if(!den) angle = 90 * M_PI / 180; // 90 degrees
    else if(!num) angle = 0; // 0 degrees
    else{
        
        // m = num / den; // slope of line
        // m = -1/m; // slope of normal
        // angle = atan(m);

        // angle = atan2(num, den);
        angle = atan2(-den, num);
    }
    return true;

}

bool compute_covariance(vector<Vector2fVector>& points, IntPairVector& points_cloud_indices, Matrix2f& covariance){
    size_t num_points = points_cloud_indices.size();
    
    Vector2f mean(0, 0);
    for (size_t index = 0; index < num_points; index++)
    {
        size_t pose_index = points_cloud_indices[index].first;
        size_t point_index = points_cloud_indices[index].second;
        mean = mean + points[pose_index][point_index];
    }

    mean = mean / num_points;


    covariance = Matrix2f::Zero();    
    for (size_t index = 0; index < num_points; index++)
    {
        size_t pose_index = points_cloud_indices[index].first;
        size_t point_index = points_cloud_indices[index].first;
        covariance = covariance + (points[pose_index][point_index] - mean) * (points[pose_index][point_index] - mean).transpose();
    
    }

    return true;

}

bool normal_of_points(vector<Vector2fVector>& points, IntPairVector& points_cloud_indices, Vector2f& normal){
    Matrix2f covariance;
    compute_covariance(points, points_cloud_indices, covariance);

    Eigen::EigenSolver<Matrix2f> es(covariance);
    if (es.eigenvalues().x().real() < es.eigenvalues().y().real())
    {
        normal = es.eigenvectors().col(0).real();
    } else {
        
        normal = es.eigenvectors().col(1).real();
    }
    
    // cout << "The eigenvalues of covariance are:" << endl << es.eigenvalues() << endl;
    // cout << "The matrix of eigenvectors, V, is:" << endl << es.eigenvectors() << endl << endl;
        
    
    return true;

    
}