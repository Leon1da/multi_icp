#include "defs.h"

// Vector2d cv_point_to_eigen_point(Point p){
//     return Vector2d(p.x, p.y);
// }

// Point eigen_point_to_cv_point(Vector2d p){
//     return Point(p.x(), p.y());
// }

// normal of line fitting point cloud
bool estimate_normal(Vector2dVector &points, IntVector& indices, double &angle){

    size_t num_points = indices.size();
    
    // cout << "num_points: " << endl << num_points << endl;
    Vector2d mean(0, 0);
    for (size_t index = 0; index < num_points; index++)
    {

        size_t point_index = indices[index];
        mean = mean + points[point_index];
        // cout << "[ " << pose_index << " " << point_index << " ]";
        // cout << "vector: " << endl << points[pose_index][point_index] << endl;
    }

    mean = mean / num_points;

    // cout << "Mean vector: " << endl << mean << endl;


    float num = 0, den = 0;

    for (size_t index = 0; index < num_points; index++)
    {
        size_t point_index = indices[index];
        Vector2d centered_point = points[point_index] - mean;

        num = num + centered_point.x() * centered_point.y();
        den = den + pow(centered_point.x(), 2);
    }

    
    if(!den) angle = 90 * M_PI / 180; // 90 degrees
    else if(!num) angle = 0; // 0 degrees
    else angle = atan2(-den, num);
    
    return true;

}


// normal of line fitting point cloud
bool estimate_normal(Vector2dVector &points, IntVector& indices, Vector2d &normal){

    size_t num_points = indices.size();

    // Center the data

    // - compute mean
    Vector2d mean(0, 0);
    for (size_t point_index = 0; point_index < num_points; point_index++)
    {
        mean = mean + points[indices[point_index]];
    }
    mean = mean / num_points;

    // cout << endl << mean << endl;

    // - subtract mean from each point
    Eigen::Matrix2Xd data(2, num_points);
    for (size_t point_index = 0; point_index < num_points; point_index++)
    {
        data.block(0, point_index, 2, 1) = points[indices[point_index]] - mean;

    } 

    // cout << endl << data << endl;

    // Calculate the covariance matrix

    Matrix2d covariance;
    covariance.setZero();

    for (size_t point_index = 0; point_index < num_points; point_index++)
    {
        covariance = covariance + (data.block(0, point_index, 2, 1) - mean) * (data.block(0, point_index, 2, 1) - mean).transpose();
    }
    
    // Calculate the covariance eigen values and eigen vectors

    // Eigen decomposition
    Eigen::EigenSolver<Eigen::Matrix2d> evd(covariance);
    float lambda1, lambda2;
    lambda1 = evd.eigenvalues().x().real();
    lambda2 = evd.eigenvalues().y().real();
    Vector2d v1, v2;
    v1 = evd.eigenvectors().col(0).real();
    v2 = evd.eigenvectors().col(1).real();

    if (lambda1 < lambda2) normal = v1;
    else normal = v2;

    // normal = normal + mean;

    
    // cout << "EIGEN " << endl << normal << endl << "(norm: " << normal.norm() << " )"<< endl;
    // Svd decomposition
    // int setting = Eigen::ComputeThinU | Eigen::ComputeThinV;
    int setting = Eigen::ComputeFullU | Eigen::ComputeFullV;
    Eigen::JacobiSVD<Eigen::Matrix2d> svd = covariance.jacobiSvd(setting);

    Eigen::Matrix2d U = svd.matrixU();
    normal = Eigen::Vector2d(U.col(1));

    Eigen::Matrix2d V = svd.matrixV();
    normal = Eigen::Vector2d(V.col(1));

    // normal = normal / normal.norm();
    
    
    // cout << "SVD " << endl << normal << endl << "(norm: " << normal.norm() << " )"<< endl;
    
    return true;

}


    // Matrix2f covariance;
    // compute_covariance(points, points_cloud_indices, covariance);
    
    // int setting = Eigen::ComputeThinU | Eigen::ComputeThinV;
    // Eigen::JacobiSVD<Eigen::Matrix2f> svd = covariance.jacobiSvd(setting);

    // Eigen::Matrix2f U = svd.matrixU();
    // normal = Eigen::Vector2f(U.col(1).x(), U.col(1).y());

    // // Eigen::Matrix2f V = svd.matrixV();
    // // normal = Eigen::Vector2f(V.col(1).x(), V.col(1).y());

    // normal = normal / normal.norm();
    
    // cout << "normal svd " << normal << endl;

    // Eigen::EigenSolver<Eigen::Matrix2f> evd(covariance);
    // float lambda1, lambda2;
    // lambda1 = evd.eigenvalues().x().real();
    // lambda2 = evd.eigenvalues().y().real();
    // Eigen::Vector2f v1, v2;
    // v1 = evd.eigenvectors().col(0).real();
    // v2 = evd.eigenvectors().col(1).real();

    // if (lambda1 < lambda2) normal = v1;
    // else normal = v2;

    // cout << "normal evd " << normal << endl;
    

    // return true;



