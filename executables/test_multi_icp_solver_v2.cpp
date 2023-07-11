#include "defs.h"

#include "dataset.h"
#include "kdtree.h"
#include "utils.h"
#include "multi_icp_solver.h"

#include "drawer.h"

int main (int argc, char** argv) {
    

  
    cout << "Multi Iterative Closest Points (ICP)." << endl << endl;

    int width = 800;
    int height = 800;
  
    Drawer drawer(width, height, "test_multi_icp_solver");
    
    
    // DrawerController drawer_controller(5.0, 5.0, 0.5);
    DrawerController drawer_controller(height/4, width/4, 5.0, 5.0, 5.0, 5.0);
    
    Scalar blue(255, 0, 0);
    Scalar red(0, 0, 255);
    Scalar green(0, 255, 0);
    Scalar pink(255, 0, 255);
    Scalar black(0, 0, 0);
    Scalar white(255, 255, 255);
    
    
    std::string dataset_filename = "/home/leonardo/multi_icp/dataset/dataset_test.txt";

    Dataset dataset(dataset_filename);

    vector<vector<MapPoint>> map;

    Vector2dVector points;
    Vector3dVector poses;
    Vector3dVector sensor_poses;
    
    
    cout << "Loading data.." << endl;

    dataset.load_data(poses, sensor_poses, points, map, 0, 500);


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

    cout << "Loading poses 2dtree.." << endl;
    finder.load_poses_2dtree();

    
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
    
    cout << "Finding for local correspondences." << endl;

    vector<vector<Correspondence>> local_correspondences;

    for (size_t pose_index = 0; pose_index < map.size(); pose_index++)
    {
      for (size_t point_index = 0; point_index < map[pose_index].size(); point_index++)
      {
        MapPoint& map_point = map[pose_index][point_index];

        vector<Correspondence> local_corr;

        vector<pair<double, int>> point_neighbors;
        if (finder.find_point_neighbors(pose_index, map_point.pose_index(), map_point.point_index(), 0.1, point_neighbors))
        {
          for (size_t i = 0; i < point_neighbors.size(); i++)
          {
            int found_index = point_neighbors[i].second;
            if ( found_index != point_index)
            {
              local_corr.push_back(Correspondence(pose_index, point_index, pose_index, found_index));
            }
          } 
        }

        local_correspondences.push_back(local_corr);
        map_point.set_local_correspondences_index(local_correspondences.size() - 1, local_corr.size());
      
      }
      
    }
    
    cout << "Finding for local correspondences ok." << endl;

    cout << "Estimating surface normals" << endl;
    
    Vector2dVector normals;
    
    for (size_t pose_index = 0; pose_index < num_poses; pose_index++)
    {
      for (size_t point_index = 0; point_index < map[pose_index].size(); point_index++)
      {
        MapPoint& map_point = map[pose_index][point_index];
        if (map_point.num_local_correspondences())
        {
          vector<Correspondence> correspondences = local_correspondences[map_point.local_correspondences_index()];
          IntVector indices;
          for (auto correspondence : correspondences) indices.push_back(correspondence._dst_point);

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

    cout << "Init Multi ICP Solver.." << endl;
    MultiICPSolver solver;

    cout << "Initializing state.." << endl;
    Isometry2fVector state(num_poses);
    for (size_t pose_index = 0; pose_index < num_poses; pose_index++)
    {
        Eigen::Isometry2f state_pose = Eigen::Isometry2f::Identity();
        state[pose_index] = state_pose;
        
    }

    cout << "Initializing state ok." << endl;

    solver.init(state, poses, points, normals, 0.001, 1.0);
    
    cout << "Init Multi ICP Solver ok." << endl;


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

        // for (size_t correspondence_index = 0; correspondence_index < global_correspondences.size(); correspondence_index++)
        // {
        //   Correspondence c = global_correspondences[correspondence_index];
          
        //   // cout << "[ " << c._src_pose << " " << c._src_point << " ] [ " << c._dst_pose << " " << c._dst_point << " ]" << endl;
        //   Vector2f start_point = v2t(poses[map[c._src_pose][c._src_point].pose_index()]) * points[map[c._src_pose][c._src_point].point_index()];
        //   Vector2f end_point = v2t(poses[map[c._dst_pose][c._dst_point].pose_index()]) * points[map[c._dst_pose][c._dst_point].point_index()];
        //   drawer.drawLine(start_point, end_point, drawer_controller, green);

        // }
        
        drawer.show();

        key=cv::waitKey(0);

        drawer_controller.update(key);

        if (key == ' '){
          
          // update correspondences
          cout << "Finding for global correspondences." << endl;
          
          Eigen::MatrixXi num_correspondences(num_poses, num_poses);
          num_correspondences.setZero();

          TriplePairVector pose_point_normal_correspondences;
          TriplePairVector cleaned_pose_point_normal_correspondences; 
          pose_point_normal_correspondences.clear();
          cleaned_pose_point_normal_correspondences.clear();
          

          for (size_t pose_index = 0; pose_index < map.size(); pose_index++)
          {
            IntVector pose_neighbors;
            if (!finder.find_pose_neighbors(pose_index, 1.5, pose_neighbors)) continue;
            // if (!finder.find_pose_neighbors(pose_index, 0.5, pose_neighbors)) continue;

            for (size_t point_index = 0; point_index < map[pose_index].size(); point_index++)
            {
              MapPoint& map_src_point = map[pose_index][point_index];
              if (!map_src_point.has_normal()) continue;

              IntTriple src(map_src_point.pose_index(), map_src_point.point_index(), map_src_point.normal_index());
                
              // double min_distance = std::numeric_limits<double>::max();
              double min_distance = MAXFLOAT;
              IntTriple dst;
              for (size_t i = 0; i < pose_neighbors.size(); i++)
              {
                int tree_index = pose_neighbors[i];
                if (tree_index != pose_index){
                  pair<double, int> point_neighbor;
                  if (finder.find_point_neighbor(tree_index, map_src_point.pose_index(), map_src_point.point_index(), point_neighbor, 0.5))
                  {
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

              if (min_distance != MAXFLOAT)
              {
                
                pose_point_normal_correspondences.push_back(TriplePair(src, dst));
                map_src_point.set_global_correspondences_index(pose_point_normal_correspondences.size() - 1, 1);
                num_correspondences(get<0>(src), get<0>(dst))++;
                // cout << "[ " << pose_index << " " << point_index << " ] global_neighbor found ( " << global_neighbor.first << " " << global_neighbor.second << " )." << endl;
                    
              } 
            }             
          }

          // cout << num_correspondences << endl;
          cout << "Finding for global correspondences ok." << endl;

          
          // int min_num_corr = 54;
          // int min_num_corr = 27;
          // int min_num_corr = 20;
          int min_num_corr = 10;
          
          // int min_num_corr = 13.5;
          // int min_num_corr = 0;
          for (size_t i = 0; i < num_poses; i++)
          {
            for (size_t j = 0; j < num_poses; j++)
            {
              if (!num_correspondences(i, j)) continue;
              // if (num_correspondences(i, j) < min_num_corr)
              //   cout << num_correspondences(i, j) << " will be skipped between pose " << i << " " << j << " ( threshold: "<< min_num_corr << " )."<< endl;
            }
              
          }

          // removing some useless correspondences 
          for (size_t i = 0; i < pose_point_normal_correspondences.size(); i++)   
          {
            TriplePair c = pose_point_normal_correspondences[i];
            int pose_i = get<0>(c.first);
            int pose_j = get<0>(c.second);
            // if (pose_j < pose_i) continue;
            if (num_correspondences(pose_i, pose_j) > min_num_corr) {
              // cout << pose_i << " " << pose_j << " "<< num_correspondences(pose_i, pose_j) << " ok" << endl;
              cleaned_pose_point_normal_correspondences.push_back(c);
            } else{
              // cout << num_correspondences(pose_i, pose_j) << " ko" << endl;
            }
            
          }
          
            
          cout << "Total correspondences: " << pose_point_normal_correspondences.size() << endl;
          cout << "Removed correspondences: " << pose_point_normal_correspondences.size() - cleaned_pose_point_normal_correspondences.size() << endl;
          
          cout << "# Multi-ICP optimization .." << endl;
          solver.oneRound(cleaned_pose_point_normal_correspondences, false);
          cout << "# inliers: " << solver.numInliers() << "(error: " << solver.chiInliers()<< " )" << " outliers: " << solver.numOutliers() <<"(error: " << solver.chiOutliers() << " )" << endl;
          // update poses
          for (size_t pose_index = 0; pose_index < state.size(); pose_index++)
          {
            poses[pose_index] = t2v(v2t(poses[pose_index]) * Eigen::Isometry2d(state[pose_index]));

          }

          cout << "#################################################################" << endl;
          
          // update poses tree
          finder.load_poses_2dtree();
            
        }

    }


  return 0;
}
    
    