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

    Vector2fVector points;
    Vector3fVector poses;
    Vector3fVector sensor_poses;
    
    
    cout << "Loading data.." << endl;

    dataset.load_data(poses, sensor_poses, points, map, 0, 1000);


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

    finder.load_poses_2dtree();

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
        if (finder.find_point_neighbors(pose_index, map_point.pose_index(), map_point.point_index(), 0.05, point_neighbors))
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
    
    Vector2fVector normals;
    
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

          float normal_vector_angle;
          if (estimate_normal(points, indices, normal_vector_angle)) {
            Vector2f normal;
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

    
    cout << "Finding for global correspondences." << endl;
    
    vector<Correspondence> global_correspondences;
    for (size_t pose_index = 0; pose_index < map.size(); pose_index++)
    {

      IntVector pose_neighbors;
      bool pose_found = finder.find_pose_neighbors(pose_index, 5, pose_neighbors);
      // bool pose_found = finder.find_pose_neighbors(pose_index, 0.1, pose_neighbors);

      for (size_t point_index = 0; point_index < map[pose_index].size(); point_index++)
      {

        MapPoint& map_point = map[pose_index][point_index];
        if (pose_found) 
        {

          // double min_distance = std::numeric_limits<double>::max();
          double min_distance = MAXFLOAT;
          IntPair nearest_correspondence;
          for (size_t i = 0; i < pose_neighbors.size(); i++)
          {
              int tree_index = pose_neighbors[i];
              if (tree_index != pose_index){
                pair<double, int> point_neighbor;
                if (finder.find_point_neighbor(tree_index, map_point.pose_index(), map_point.point_index(), point_neighbor, 0.05))
                {

                  double point_distance = point_neighbor.first;
                  if (point_distance < min_distance)
                  {
                    min_distance = point_distance;
                    nearest_correspondence = IntPair(tree_index, point_neighbor.second);
                  }
                }
              }
          }

          if (min_distance != MAXFLOAT)
          {
            global_correspondences.push_back(Correspondence(IntPair(pose_index, point_index), nearest_correspondence));
            map_point.set_global_correspondences_index(global_correspondences.size() - 1, 1);
            // cout << "[ " << pose_index << " " << point_index << " ] global_neighbor found ( " << global_neighbor.first << " " << global_neighbor.second << " )." << endl;
                
          }
          
        }
        
      }
      
    }

    cout << "Finding for global correspondences ok." << endl;
    

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

    solver.init(state, poses, points, normals);
    
    cout << "Init Multi ICP Solver ok." << endl;


    
    // will be passed to the optimization round
    vector<Correspondence> total_correspondeces;
    IntPairVector total_normals;

    for (size_t correspondence_index = 0; correspondence_index < global_correspondences.size(); correspondence_index++)
    {
      Correspondence correspondence = global_correspondences[correspondence_index];
      MapPoint src_map_point = map[correspondence._src_pose][correspondence._src_point];
      MapPoint dst_map_point = map[correspondence._dst_pose][correspondence._dst_point];
      if (src_map_point.has_normal() && dst_map_point.has_normal())   
      {
        Correspondence total_correspondece = Correspondence(src_map_point.pose_index(), src_map_point.point_index(), dst_map_point.pose_index(), dst_map_point.point_index());
        total_correspondeces.push_back(total_correspondece);
        total_normals.push_back(IntPair(src_map_point.normal_index(), dst_map_point.normal_index()));
      }
    }

    cout << "#################################################################" << endl;
            
    cout << global_correspondences.size() << " points will be used for otpimization [total points: " << num_points << " ]" << endl;

    


    char key=0;
    const char ESC_key=27;
    while (key!=ESC_key) {

        drawer.clear();
        
        for (size_t pose_index = 0; pose_index < num_poses; pose_index++)
        {
          Vector3f world_pose = poses[pose_index];
          drawer.drawPose(world_pose, drawer_controller, red);

          for (size_t point_index = 0; point_index < map[pose_index].size(); point_index++)
          {
            MapPoint map_point = map[pose_index][point_index];
            Vector2f world_point = v2t(poses[map_point.pose_index()]) * points[map_point.point_index()];
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

            cout << "# Multi-ICP optimization .." << endl;
            solver.oneRound(total_correspondeces, total_normals, false);
            cout << "# Multi-ICP optimization complete." << endl;
            
            cout << "####### Multi-ICP iteration #######" << endl; 
            cout << "# inliers: " << solver.numInliers() << " outliers: " << solver.numOutliers() << endl;
            cout << "# error (inliers): " << solver.chiInliers() << " (outliers): " << solver.chiOutliers() << endl;
            cout << "#################################################################" << endl;
            
            // update poses
            for (size_t pose_index = 0; pose_index < state.size(); pose_index++)
            {
              poses[pose_index] = t2v(v2t(poses[pose_index]) * state[pose_index]);

            }

            cout << "Finding for new global correspondences." << endl;
            
            global_correspondences.clear();
            
            for (size_t pose_index = 0; pose_index < map.size(); pose_index++)
            {

              IntVector pose_neighbors;
              bool pose_found = finder.find_pose_neighbors(pose_index, 5, pose_neighbors);
              // bool pose_found = finder.find_pose_neighbors(pose_index, 0.1, pose_neighbors);

              for (size_t point_index = 0; point_index < map[pose_index].size(); point_index++)
              {

                MapPoint& map_point = map[pose_index][point_index];
                if (pose_found) 
                {

                  // double min_distance = std::numeric_limits<double>::max();
                  double min_distance = MAXFLOAT;
                  IntPair nearest_correspondence;
                  for (size_t i = 0; i < pose_neighbors.size(); i++)
                  {
                      int tree_index = pose_neighbors[i];
                      if (tree_index != pose_index){
                        pair<double, int> point_neighbor;
                        if (finder.find_point_neighbor(tree_index, map_point.pose_index(), map_point.point_index(), point_neighbor, 0.05))
                        {

                          double point_distance = point_neighbor.first;
                          if (point_distance < min_distance)
                          {
                            min_distance = point_distance;
                            nearest_correspondence = IntPair(tree_index, point_neighbor.second);
                          }
                        }
                      }
                  }

                  if (min_distance != MAXFLOAT)
                  {
                    global_correspondences.push_back(Correspondence(IntPair(pose_index, point_index), nearest_correspondence));
                    map_point.set_global_correspondences_index(global_correspondences.size() - 1, 1);
                    // cout << "[ " << pose_index << " " << point_index << " ] global_neighbor found ( " << global_neighbor.first << " " << global_neighbor.second << " )." << endl;
                        
                  }
                  
                }
                
              }
              
            }

            
            cout << "Finding for new global correspondences ok." << endl;

            total_correspondeces.clear();
            total_normals.clear();
            // will be passed to the optimization round

            for (size_t correspondence_index = 0; correspondence_index < global_correspondences.size(); correspondence_index++)
            {
              Correspondence correspondence = global_correspondences[correspondence_index];
              MapPoint src_map_point = map[correspondence._src_pose][correspondence._src_point];
              MapPoint dst_map_point = map[correspondence._dst_pose][correspondence._dst_point];
              if (src_map_point.has_normal() && dst_map_point.has_normal())   
              {
                Correspondence total_correspondece = Correspondence(src_map_point.pose_index(), src_map_point.point_index(), dst_map_point.pose_index(), dst_map_point.point_index());
                total_correspondeces.push_back(total_correspondece);
                total_normals.push_back(IntPair(src_map_point.normal_index(), dst_map_point.normal_index()));
              }
            }

            
            cout << global_correspondences.size() << " points will be used for otpimization [total points: " << num_points << " ]" << endl;

            
            

        
        }

    }


  return 0;
}
    
    