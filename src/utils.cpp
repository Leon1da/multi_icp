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




bool read_configuration(
    int argc, char** argv,
    string &dataset_filename,
    int &dataset_from_record_number,
    int &dataset_num_records,
    int &points_kdtree_dim,
    int &poses_kdtree_dim,
    int &min_poses_correspondences,
    int &min_local_correspondences,
    double &kernel_threshold,
    double &damping,
    bool &keep_outliers
){
    
    OptionParser op;
    auto help_option   = op.add<Switch>("h", "help", "The following options are available.");
    
    // data options 
    auto dataset_option = op.add<Value<string>>("p", "dataset_path", "[DATASET] Path of the file containg the map to be optimized.", "../../dataset/dataset_test.txt", &dataset_filename);
    auto dataset_from_record_number_option = op.add<Value<int>>("f", "from_pose", "[DATASET] From which record start to load the dataset", 0, &dataset_from_record_number);
    auto dataset_num_records_option = op.add<Value<int>>("n", "num_pose", "[DATASET] How many record load", 100, &dataset_num_records);
    
    // finder options
    auto points_kdtree_dim_option = op.add<Value<int>>("z", "points_kdtree_dim", "[FINDER] Dimension of the kdtrees containing the points (2 or 4)", 2, &points_kdtree_dim);
    auto poses_kdtree_dim_option = op.add<Value<int>>("y", "poses_kdtree_dim", "[FINDER] Dimension of the kdtrees containing the poses (2 or 4)", 2, &poses_kdtree_dim);
    auto min_poses_correspondences_option = op.add<Value<int>>("c", "min_poses_corr", "[FINDER] Minimum number of correspondences betewenn two poses", 0, &min_poses_correspondences);
    auto min_local_correspondences_option = op.add<Value<int>>("e", "min_local_corr", "[FINDER] Minimum number of point correspondences needed to provide a good estimation of the normal", 0, &min_local_correspondences);

    // auto pose_finder_r_option = op.add<Value<double>>("k", "k", "[FINDER] kernel threshold", 2, &kernel_threshold);
    // auto pose_finder_k_option = op.add<Value<int>>("k", "k", "[FINDER] kernel threshold", 2, &kernel_threshold);
    // auto points_finder_r_option = op.add<Value<double>>("k", "k", "[FINDER] kernel threshold", 2, &kernel_threshold);
    // auto points_finder_k_option = op.add<Value<int>>("k", "k", "[FINDER] kernel threshold", 2, &kernel_threshold);

    // solver options
    auto kernel_threshold_option = op.add<Value<double>>("k", "kernel_threshold", "[SOLVER] Kernel threshold used during optimization.", 1.0, &kernel_threshold);
    auto damping_option = op.add<Value<double>>("d", "damping", "[SOLVER] Damping factor used during optimization", 1.0, &damping);
    auto keep_outliers_option = op.add<Switch>("o", "keep_outliers", "[SOLVER] Consider or not outliers during the computation", &keep_outliers);

    op.parse(argc, argv);

    // show all non option arguments (those without "-o" or "--option")
	for (const auto& non_option_arg: op.non_option_args())
		  cout << "Not option arguments provided: " << non_option_arg << "\n";

    // show unknown options (undefined ones, like "-u" or "--undefined")
    for (const auto& unknown_option: op.unknown_options())
      cout << "Unknown optiont arguments provided: " << unknown_option << "\n";

    if (op.non_option_args().size() > 0 || op.unknown_options().size() > 0) {
      cout << op << "\n";
      return false;
    }

    // print auto-generated help message
    if (help_option->is_set()){
      cout << op << "\n";
      return false;
    
    }

    return true;

}

void print_configuration(
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

){
          
    
    cout << "Configuration: " << endl;
    
    cout << " - [DATASET] dataset_path = " << dataset_filename << endl;
    cout << " - [DATASET] from_record = " << dataset_from_record_number << endl;
    cout << " - [DATASET] num_record = " << dataset_num_records << endl;

    cout << endl;
    cout << " - [FINDER] points_tree_size = " << points_kdtree_dim << endl;
    cout << " - [FINDER] poses_tree_size = " << poses_kdtree_dim << endl;
    cout << " - [FINDER] min_poses_correspondences = " << min_poses_correspondences << endl;
    cout << " - [FINDER] min_local_correspondences = " << min_local_correspondences << endl;

    cout << endl;
    cout << " - [SOLVER] kernel_threshold = " << kernel_threshold << endl;
    cout << " - [SOLVER] damping = " << damping << endl;
    cout << " - [SOLVER] keep_outliers = " << keep_outliers << endl;


    cout << endl;
    

}