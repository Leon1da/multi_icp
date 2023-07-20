#include "defs.h"

#include "dataset.h"
#include "kdtree.h"
#include "utils.h"
#include "multi_icp_solver.h"

#include "drawer.h"



int main (int argc, char** argv) {
    
    cout << "Multi Iterative Closest Points (ICP)." << endl;
    // parsing aruments

    OptionParser op;
    
    auto help_option   = op.add<Switch>("h", "help", "The following options are available.");
    
    // absolute dataset path
    // "/home/leonardo/multi_icp/dataset/dataset_test.txt"
    
    // data options 
    string dataset_filename;
    auto dataset_option = op.add<Value<string>>("p", "dataset_path", "[DATASET] Path of the file containg the map to be optimized.", "../../dataset/dataset_test.txt", &dataset_filename);
    
    int dataset_from_record_number;
    auto dataset_from_record_number_option = op.add<Value<int>>("f", "from_pose", "[DATASET] From which record start to load the dataset", 0, &dataset_from_record_number);
    
    int dataset_num_records;
    auto dataset_num_records_option = op.add<Value<int>>("n", "num_pose", "[DATASET] How many record load", 100, &dataset_num_records);
    
    // finder options
    int points_kdtree_dim;
    auto points_kdtree_dim_option = op.add<Value<int>>("z", "points_kdtree_dim", "[FINDER] Dimension of the kdtrees containing the points (2 or 4)", 2, &points_kdtree_dim);
    
    int poses_kdtree_dim;
    auto poses_kdtree_dim_option = op.add<Value<int>>("y", "poses_kdtree_dim", "[FINDER] Dimension of the kdtrees containing the poses (2 or 4)", 2, &poses_kdtree_dim);
    
    int min_poses_correspondences;
    auto min_poses_correspondences_option = op.add<Value<int>>("c", "min_poses_corr", "[FINDER] Minimum number of correspondences betewenn two poses", 0, &min_poses_correspondences);

    int min_local_correspondences;
    auto min_local_correspondences_option = op.add<Value<int>>("e", "min_local_corr", "[FINDER] Minimum number of point correspondences needed to provide a good estimation of the normal", 0, &min_local_correspondences);

    // auto pose_finder_r_option = op.add<Value<double>>("k", "k", "[FINDER] kernel threshold", 2, &kernel_threshold);
    // auto pose_finder_k_option = op.add<Value<int>>("k", "k", "[FINDER] kernel threshold", 2, &kernel_threshold);
    // auto points_finder_r_option = op.add<Value<double>>("k", "k", "[FINDER] kernel threshold", 2, &kernel_threshold);
    // auto points_finder_k_option = op.add<Value<int>>("k", "k", "[FINDER] kernel threshold", 2, &kernel_threshold);

    // solver options
    double kernel_threshold;
    auto kernel_threshold_option = op.add<Value<double>>("k", "kernel_threshold", "[SOLVER] Kernel threshold used during optimization.", 1.0, &kernel_threshold);
    
    double damping;
    auto damping_option = op.add<Value<double>>("d", "damping", "[SOLVER] Damping factor used during optimization", 1.0, &damping);

    bool keep_outliers;
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
      return 0;
    }

    // print auto-generated help message
    if (help_option->is_set()){
      cout << op << "\n";
      return 0;
    
    }
      
    
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

    
    int width = 1000;
    int height = 1000;
    Drawer drawer(width, height, "test_multi_icp_solver");
    // DrawerController drawer_controller(5.0, 5.0, 0.5);
    DrawerController drawer_controller(height/4, width/4, 5.0, 5.0, 5.0, 5.0);
    
    Scalar blue(255, 0, 0);
    Scalar red(0, 0, 255);
    Scalar green(0, 255, 0);
    Scalar pink(255, 0, 255);
    Scalar black(0, 0, 0);
    Scalar white(255, 255, 255);
    
    
    Dataset dataset(dataset_filename);

    vector<vector<MapPoint>> map;

    Vector2dVector points;
    Vector3dVector poses;
    Vector3dVector sensor_poses;
    
    
    cout << "Loading data.." << endl;

    dataset.load_data(poses, sensor_poses, points, map, dataset_from_record_number, dataset_num_records);


    // for(auto pose : poses) cout << pose << endl << endl;
    // for(auto point : points) cout << point << endl << endl;

    size_t num_poses = poses.size();
    size_t num_points = points.size();

    cout << num_poses << " poses have been loaded." << endl;

    cout << num_points << " points have been loaded." << endl;

    // apply sensor offset to pose (it is a semplification not strictly needed).
    for (size_t pose_index = 0; pose_index < num_poses; pose_index++)
    {
        poses[pose_index] = t2v(v2t(poses[pose_index]) * v2t(sensor_poses[pose_index]));
    }
    

    cout << "Loading data complete." << endl << endl;
    
    cout << "Loading correspondence finder." << endl;

    kdt::CorrespondenceFinder finder;
    
    finder.init(poses, points);

    switch (poses_kdtree_dim)
    {
      case 2:
        cout << "Loading poses 2dtree.." << endl;
        finder.load_poses_2dtree();
        break;
      case 4:
        cout << "Loading poses 4dtree.." << endl;
        finder.load_poses_4dtree();
        break;
      default:
        cout << "Error during load poses kdtree, supported kdtree dimension is 2 or 4." << endl;
        throw runtime_error("Not implemented exception");
        
    }
 
    cout << "Loading points 2dtree.." << endl;
    for (size_t pose_index = 0; pose_index < map.size(); pose_index++)
    {
      IntVector points_indices;
      for (size_t point_index = 0; point_index < map[pose_index].size(); point_index++)
      {
        MapPoint map_point = map[pose_index][point_index];
        points_indices.push_back(map_point.point_index());
      }
      
      // loading kdtree
      finder.load_points_2dtree(pose_index, points_indices);

    }
    
    cout << "Loading correspondence finder complete." << endl << endl;
    
    cout << "Estimating surface normals.." << endl;
    
    Vector2dVector normals;
    
    for (size_t pose_index = 0; pose_index < map.size(); pose_index++)
    {
      for (size_t point_index = 0; point_index < map[pose_index].size(); point_index++)
      {
        MapPoint& map_point = map[pose_index][point_index];

        vector<pair<double, int>> point_neighbors;
        if (finder.find_point_neighbors(pose_index, map_point.pose_index(), map_point.point_index(), 0.05, point_neighbors))
        // if (finder.find_point_neighbors(pose_index, map_point.pose_index(), map_point.point_index(), 20, point_neighbors))
        {
          if (point_neighbors.size() < (size_t) min_local_correspondences) continue;
          IntVector indices;
          for (size_t i = 0; i < point_neighbors.size(); i++){
            MapPoint p = map[pose_index][point_neighbors[i].second];
            indices.push_back(p.point_index());
          } 

          double normal_vector_angle;
          if (estimate_normal(points, indices, normal_vector_angle)) {
            Vector2d normal;
            normal << cos(normal_vector_angle), sin(normal_vector_angle);
            normals.push_back(normal);
            map_point.set_normal_index(normals.size() - 1);
            // cout << "classic normal " << endl << normal << endl << "(norm: " << normal.norm() << " )"<< endl;
          }
          
          // Vector2f normal;
          // if (estimate_normal(points, indices, normal)) {
          //   normals.push_back(normal);
          //   map_point.set_normal_index(normals.size() - 1);

          // }
          
           
        }

      
      }
      
    }
    
    cout << normals.size() << " normals have been estimated." << endl;
    
    cout << "Estimating surface normals complete." << endl;

    
    vector<IntVector> index_map;
    if (points_kdtree_dim == 4)
    {
      
      finder.set_normals(&normals);

      cout << "Loading points 4dtree.." << endl;
      for (size_t pose_index = 0; pose_index < map.size(); pose_index++)
      {
        IntVector index_map_record;
        IntVector points_indices;
        IntVector normals_indices;
        for (size_t point_index = 0; point_index < map[pose_index].size(); point_index++)
        {
          MapPoint map_point = map[pose_index][point_index];

          // points without normals will be no more considered
          if (map_point.has_normal())
          {
            index_map_record.push_back(point_index); // IMPORTANT! map coordinates
            points_indices.push_back(map_point.point_index());  // points structure coords
            normals_indices.push_back(map_point.normal_index()); // normals structure coords
          }
        }

        assert(points_indices.size() == normals_indices.size());
        
        finder.load_points_4dtree(pose_index, points_indices, normals_indices);
        
        // contains the association between the indices contained into the kdtree and the map positions
        // required since some points normals could be not esimated so there is no 1:1 association 
        index_map.push_back(index_map_record);


      }
      
    }
        



    cout << "Init Multi ICP Solver.." << endl;
    MultiICPSolver solver;

    cout << "Initializing state.." << endl;
    Isometry2dVector state(num_poses);
    for (size_t pose_index = 0; pose_index < num_poses; pose_index++)
    {
        Eigen::Isometry2d state_pose = Eigen::Isometry2d::Identity();
        state[pose_index] = state_pose;
        
    }

    cout << "Initializing state ok." << endl;

    solver.init(state, poses, points, normals, kernel_threshold, damping);
    
    cout << "Init Multi ICP Solver ok." << endl;

    Eigen::MatrixXi num_correspondences(num_poses, num_poses);
          
    TriplePairVector pose_point_normal_correspondences;
    TriplePairVector cleaned_pose_point_normal_correspondences; 
    IntPairVector poses_correspondences;
    

    bool drawing_poses = true;
    bool drawing_points = true;
    bool drawing_normals = false;
    bool drawing_poses_correspondences = false;
    bool drawing_points_correspondences = false;


    char key=0;
    const char ESC_key=27;
    while (key!=ESC_key) {

        drawer.clear();

        // draw things
        for (size_t pose_index = 0; pose_index < num_poses; pose_index++)
        { 
          
          Vector3d world_pose = poses[pose_index];
          if (drawing_poses)
          {
            drawer.drawPose(world_pose, drawer_controller, red);
          }
          
          for (size_t point_index = 0; point_index < map[pose_index].size(); point_index++)
          {
            MapPoint map_point = map[pose_index][point_index];
            Vector2d world_point = v2t(poses[map_point.pose_index()]) * points[map_point.point_index()];
              
            if (drawing_points)
            {
              drawer.drawPoint(world_point, drawer_controller, black);  
            }

            if (map_point.has_normal() && drawing_normals)
            {
              Vector2d world_normal = world_point + v2t(poses[pose_index]).rotation() * normals[map_point.normal_index()] * 0.05; // It's ok
              drawer.drawLine(world_point, world_normal, drawer_controller, green);
              
            }
              
          }
          
        }

        if (drawing_points_correspondences)
        { 
          for (size_t i = 0; i < cleaned_pose_point_normal_correspondences.size(); i++)
          {
              TriplePair c = cleaned_pose_point_normal_correspondences[i];
            
              Vector2d start_point = v2t(poses[get<0>(c.first)]) * points[get<1>(c.first)];
              Vector2d end_point = v2t(poses[get<0>(c.second)]) * points[get<1>(c.second)];
              drawer.drawLine(start_point, end_point, drawer_controller, green);

          }
          
        }

        if (drawing_poses_correspondences)
        {
          for (size_t i = 0; i < poses_correspondences.size(); i++)
          {
            IntPair c = poses_correspondences[i];
            Vector2d start_point = poses[c.first].block(0, 0, 2, 1);
            Vector2d end_point = poses[c.second].block(0, 0, 2, 1);
            drawer.drawLine(start_point, end_point, drawer_controller, pink);

          }
          
        }
        
        
        drawer.show();

        key=cv::waitKey(0);

        // update the controller (w, a, s, d)
        drawer_controller.update(key);

        // update what drawer should display (points, normal, poses, ..)
        switch (key)
        {
          case '1':
            // points
            if (drawing_points) drawing_points = false;
            else drawing_points = true;

            break;
          case '2':
            // poses
            if (drawing_poses) drawing_poses = false;
            else drawing_poses = true;
            
            break;
          case '3':
            // normals
            if (drawing_normals) drawing_normals = false;
            else drawing_normals = true;
            
            break;
          case '4':
            // points correspondences
            if (drawing_points_correspondences) drawing_points_correspondences = false;
            else drawing_points_correspondences = true;
            
            break;
          case '5':
            // poses correspondences
            if (drawing_poses_correspondences) drawing_poses_correspondences = false;
            else drawing_poses_correspondences = true;
            
            break;
          
          default:
            break;
        }

        if (key == ' '){
          
          // update correspondences
          cout << "Updating global correspondences." << endl;
          
          num_correspondences.setZero();
          pose_point_normal_correspondences.clear();
          cleaned_pose_point_normal_correspondences.clear();
          poses_correspondences.clear();

          

          for (size_t pose_index = 0; pose_index < map.size(); pose_index++)
          {
            IntVector pose_neighbors;

            if (!finder.find_pose_neighbors(pose_index, 0.5, pose_neighbors, poses_kdtree_dim)) continue;

            for (size_t point_index = 0; point_index < map[pose_index].size(); point_index++)
            {
              MapPoint& map_src_point = map[pose_index][point_index];
              if (!map_src_point.has_normal()) continue;

              IntTriple src(map_src_point.pose_index(), map_src_point.point_index(), map_src_point.normal_index());
                
              // double min_distance = std::numeric_limits<double>::max();
              double min_distance = DBL_MAX;
              IntTriple dst;
              for (size_t i = 0; i < pose_neighbors.size(); i++)
              {
                size_t tree_index = pose_neighbors[i];
                if (tree_index != pose_index){
                  
                  pair<double, int> point_neighbor;
                  bool point_neighbor_exist = false;
                  switch (points_kdtree_dim)
                  {
                    case 2:
                      point_neighbor_exist = finder.find_point_neighbor(tree_index, map_src_point.pose_index(), map_src_point.point_index(), point_neighbor, 0.1);
                      // point_neighbor_exist = finder.find_point_neighbor(tree_index, map_src_point.pose_index(), map_src_point.point_index(), point_neighbor, 1);
                      break;
                    case 4:
                      point_neighbor_exist = finder.find_point_neighbor(tree_index, map_src_point.pose_index(), map_src_point.point_index(), map_src_point.normal_index(), point_neighbor, 0.05);
                      // point_neighbor_exist = finder.find_point_neighbor(tree_index, map_src_point.pose_index(), map_src_point.point_index(), map_src_point.normal_index(), point_neighbor, 1);
                      point_neighbor.second = index_map[tree_index][point_neighbor.second]; // IMPORTANT
                      break;
                    default:
                      cout << "Error during finding point correspondences, supported kdtree dimension is 2 or 4." << endl;
                      throw runtime_error("Not implemented exception");
                      break;
                  }
                  if (point_neighbor_exist)
                  {
                    // TODO bug tree_point_index could be wrong if not all the normals have been estimated (corrected pay attention)
                    int tree_point_index = point_neighbor.second;
                    double tree_point_distance = point_neighbor.first;
                    MapPoint& map_dst_point = map[tree_index][tree_point_index];
                    if (!map_dst_point.has_normal()) continue;
               
                    if (tree_point_distance < min_distance)
                    {
                      min_distance = tree_point_distance;
                      get<0>(dst) = map_dst_point.pose_index();
                      get<1>(dst) = map_dst_point.point_index();
                      get<2>(dst) = map_dst_point.normal_index();
                      
                    }
                  }
                }
              }

              if (min_distance != DBL_MAX)
              {
                // cout << "min_distance " << min_distance << endl;
                pose_point_normal_correspondences.push_back(TriplePair(src, dst));
                map_src_point.set_global_correspondences_index(pose_point_normal_correspondences.size() - 1, 1);
                num_correspondences(get<0>(src), get<0>(dst))++;
                // cout << "[ " << pose_index << " " << point_index << " ] global_neighbor found ( " << global_neighbor.first << " " << global_neighbor.second << " )." << endl;
                    
              } 
            }             

            for (size_t i = 0; i < pose_neighbors.size(); i++)
            {
              if (i != pose_neighbors[i]) poses_correspondences.push_back(IntPair(pose_index, pose_neighbors[i]));
            }
            
          }

          // cout << num_correspondences << endl;
          cout << "Updating global correspondences ok." << endl;

          cout << "Removing correspondences below the threshold. (threshold="<< min_poses_correspondences << " )." << endl;

          // removing some useless correspondences 
          for (size_t i = 0; i < pose_point_normal_correspondences.size(); i++)   
          {
            TriplePair c = pose_point_normal_correspondences[i];
            int pose_i = get<0>(c.first);
            int pose_j = get<0>(c.second);
            // if (pose_j < pose_i) continue;
            if (num_correspondences(pose_i, pose_j) > min_poses_correspondences) {
              // cout << pose_i << " " << pose_j << " "<< num_correspondences(pose_i, pose_j) << " ok" << endl;
              cleaned_pose_point_normal_correspondences.push_back(c);
            } else{
              // cout << num_correspondences(pose_i, pose_j) << " ko" << endl;
            }
            
          }

          cout << "ok." << endl << endl;
          
            
          cout << "Total correspondences: " << pose_point_normal_correspondences.size() << endl;
          cout << "Removed correspondences: " << pose_point_normal_correspondences.size() - cleaned_pose_point_normal_correspondences.size() << endl;
          
          cout << "# Multi-ICP optimization .." << endl;
          solver.oneRound(cleaned_pose_point_normal_correspondences, keep_outliers);
          cout << "# inliers: " << solver.numInliers() << " ( error: " << solver.chiInliers()<< " )" << " outliers: " << solver.numOutliers() << " ( error: " << solver.chiOutliers() << " ). " << endl;
          // update poses
          for (size_t pose_index = 0; pose_index < state.size(); pose_index++)
          {
            poses[pose_index] = t2v(v2t(poses[pose_index]) * Eigen::Isometry2d(state[pose_index]));

          }

          cout << "#################################################################" << endl;
          
          // update poses tree
          switch (poses_kdtree_dim)
          {
            case 2:
              cout << "Loading poses 2dtree.." << endl;
              finder.load_poses_2dtree();
              cout << "ok." << endl;
              break;
            case 4:
              cout << "Loading poses 4dtree.." << endl;
              finder.load_poses_4dtree();
              cout << "ok." << endl;
              break;
            default:
              throw runtime_error("Not implemented exception");
              cout << "Error during load poses kdtree, supported dimensions are 2 and 4." << endl;
              
          }
            
        }

    }


  return 0;
}
    
    