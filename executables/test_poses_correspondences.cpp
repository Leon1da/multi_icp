#include "defs.h"

#include "dataset.h"
#include "drawer.h"

#include "utils.h"
#include "kdtree.h"


int main (int argc, char** argv) {
    
  
    cout << "Test Correspondence Finder using poses." << endl << endl;

    OptionParser op;
    
    auto help_option   = op.add<Switch>("h", "help", "The following options are available.");
    
    // absolute dataset path
    // "/home/leonardo/multi_icp/dataset/dataset_test.txt"
    
    // data options 
    string dataset_filename;
    auto dataset_option = op.add<Value<string>>("p", "dataset_path", "[DATASET] Path of the file containg the map to be optimized.", "../../dataset/dataset_test.txt", &dataset_filename);
    
    // int dataset_from_pose_number;
    // auto dataset_from_pose_number_option = op.add<Value<int>>("f", "from_pose", "[DATASET] From which record start to load the dataset", 0, &dataset_from_pose_number);
    
    // int dataset_num_pose;
    // auto dataset_num_pose_option = op.add<Value<int>>("n", "num_pose", "[DATASET] How many record load", 10, &dataset_num_pose);
    
    // finder options
    
    int poses_kdtree_dim;
    auto poses_kdtree_dim_option = op.add<Value<int>>("y", "poses_kdtree_dim", "[FINDER] Dimension of the kdtrees containing the poses (2 or 4)", 2, &poses_kdtree_dim);

    op.parse(argc, argv);

    // print auto-generated help message
    if (help_option->is_set()){
      cout << op << "\n";
      return 0;
    
    }
      
    
    cout << "Configuration: " << endl;
    
    cout << " - [DATASET] dataset_path = " << dataset_filename << endl;
    cout << " - [FINDER] poses_tree_size = " << poses_kdtree_dim << endl;
    cout << endl;

    
    Dataset dataset(dataset_filename);

    vector<vector<MapPoint>> map;

    Vector2dVector points;
    Vector3dVector poses;
    Vector3dVector sensor_poses;
    

    cout << "Loading data.." << endl;

    dataset.load_data(poses, sensor_poses, points, map, 0, 500);


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
        throw runtime_error("Not implemented exception");
        cout << "Error during load poses kdtree, supported dimensions are 2 and 4." << endl;
        
    }
    
    cout << "Loading correspondence finder complete." << endl;
    
    cout << "Finding for global correspondences." << endl;
    
    IntPairVector poses_correspondences;

    for (size_t pose_index = 0; pose_index < map.size(); pose_index++)
    {

      IntVector pose_neighbors;
      // bool pose_found = finder.find_pose_neighbors(pose_index, 3, pose_neighbors, poses_kdtree_dim);
      bool pose_found = finder.find_pose_neighbors(pose_index, 1.5, pose_neighbors, poses_kdtree_dim);
      if (!pose_found) continue;

      for (size_t i = 0; i < pose_neighbors.size(); i++)
      {
        poses_correspondences.push_back(IntPair(pose_index, pose_neighbors[i]));
  
      }
      
      
    }

    cout << "Finding for global correspondences ok." << endl;
    
    cout << "Finding correspondences ok." << endl;


    int width = 800;
    int height = 800;
  
    Drawer drawer(width, height, "test_poses_correspondences");

    // DrawerController drawer_controller(5.0, 5.0, 0.5);
    DrawerController drawer_controller(height/4, width/4, 5.0, 5.0, 5.0, 5.0);
    
    Scalar blue(255, 0, 0);
    Scalar red(0, 0, 255);
    Scalar green(0, 255, 0);
    Scalar pink(255, 0, 255);
    Scalar black(0, 0, 0);
    Scalar white(255, 255, 255);
    


    char key=0;
    const char ESC_key=27;
    while (key!=ESC_key) {

        drawer.clear();
        
        for (size_t pose_index = 0; pose_index < num_poses; pose_index++)
        {
          Vector3d world_pose = poses[pose_index];
          drawer.drawPose(world_pose, drawer_controller, red);

          for (size_t point_index = 0; point_index < map[pose_index].size(); point_index++)
          {

            MapPoint map_point = map[pose_index][point_index];

            Vector2d world_point = v2t(poses[map_point.pose_index()]) * points[map_point.point_index()];
            
            drawer.drawPoint(world_point, drawer_controller, black);

    
          }
          
        }
        
        for (size_t i = 0; i < poses_correspondences.size(); i++)
        {
          IntPair pose_corr = poses_correspondences[i];
          Vector3d start_pose = poses[pose_corr.first];
          Vector3d end_pose = poses[pose_corr.second];
          // cout << "[ " << c._src_pose << " " << c._src_point << " ] [ " << c._dst_pose << " " << c._dst_point << " ]" << endl;
          Vector2d start_point = Vector2d(start_pose.x(), start_pose.y());
          Vector2d end_point = Vector2d(end_pose.x(), end_pose.y());
          drawer.drawLine(start_point, end_point, drawer_controller, green);

        }
        

        drawer.show();

        key=cv::waitKey(0);

        drawer_controller.update(key);

    }


  return 0;
}