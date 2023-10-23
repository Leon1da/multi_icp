# include "defs.h"


bool read_configuration(
    int argc, char** argv,
    string &dataset_filename,
    int &dataset_from_record_number,
    int &dataset_num_records,
    int &points_kdtree_dim,
    int &poses_kdtree_dim,
    int &min_poses_correspondences,
    int &min_local_correspondences,
    int &iterations,
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
    auto min_poses_correspondences_option = op.add<Value<int>>("c", "min_poses_corr", "[FINDER] Minimum number of correspondences betewenn two poses", 1, &min_poses_correspondences);
    auto min_local_correspondences_option = op.add<Value<int>>("e", "min_local_corr", "[FINDER] Minimum number of point correspondences needed to provide a good estimation of the normal", 2, &min_local_correspondences);

    // auto pose_finder_r_option = op.add<Value<double>>("k", "k", "[FINDER] kernel threshold", 2, &kernel_threshold);
    // auto pose_finder_k_option = op.add<Value<int>>("k", "k", "[FINDER] kernel threshold", 2, &kernel_threshold);
    // auto points_finder_r_option = op.add<Value<double>>("k", "k", "[FINDER] kernel threshold", 2, &kernel_threshold);
    // auto points_finder_k_option = op.add<Value<int>>("k", "k", "[FINDER] kernel threshold", 2, &kernel_threshold);

    // solver options
    auto iterations_option = op.add<Value<int>>("i", "iterations", "[SOLVER] number of iterations.", 10, &iterations);
    auto kernel_threshold_option = op.add<Value<double>>("k", "kernel_threshold", "[SOLVER] Kernel threshold used during optimization.", 0.025, &kernel_threshold);
    auto damping_option = op.add<Value<double>>("d", "damping", "[SOLVER] Damping factor used during optimization", 1000.0, &damping);
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
    int& iterations,
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
    cout << " - [SOLVER] iterations = " << iterations << endl;
    cout << " - [SOLVER] kernel_threshold = " << kernel_threshold << endl;
    cout << " - [SOLVER] damping = " << damping << endl;
    cout << " - [SOLVER] keep_outliers = " << keep_outliers << endl;


    cout << endl;
    

}

string conf_to_string(
    string& label,
    string& dataset_filename,
    int& dataset_from_record_number,
    int& dataset_num_records,
    int& points_kdtree_dim,
    int& poses_kdtree_dim,
    int& min_poses_correspondences,
    int& min_local_correspondences,
    int& iterations,
    double& kernel_threshold,
    double& damping,
    bool& keep_outliers
){
          
    cout << "ok" << endl;
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);

    
    cout << "ok 1" << endl;
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y_%m_%d_%H_%M_%S") << "_conf_it_" << iterations << "_kt_" << kernel_threshold << "_dm_" << damping;
    
    cout << "ok 2" << endl;
    label = oss.str();

    cout << label << endl; 
    cout << "ok 3" << endl;


    

}

    