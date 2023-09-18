#include "defs.h"

#include "dataset.h"
#include "kdtree.h"
#include "utils.h"
#include "multi_icp_solver.h"

#include "drawer.h"



int main (int argc, char** argv) {
    
    cout << "Multi Iterative Closest Points (ICP)." << endl;
    
    // Read configuration from arguments
    
    string dataset_filename;
    int dataset_from_record_number;
    int dataset_num_records;
    int points_kdtree_dim;
    int poses_kdtree_dim;
    int min_poses_correspondences;
    int min_local_correspondences;
    int iterations;
    double kernel_threshold;
    double damping;
    bool keep_outliers;
    
    bool out = read_configuration(
      argc, argv, 
      dataset_filename, dataset_from_record_number, dataset_num_records,
      points_kdtree_dim, poses_kdtree_dim, min_poses_correspondences, min_local_correspondences, 
      iterations, kernel_threshold, damping, keep_outliers);

    if (!out) return 0;
    

    print_configuration(dataset_filename, dataset_from_record_number, dataset_num_records,
      points_kdtree_dim, poses_kdtree_dim, min_poses_correspondences, min_local_correspondences,
      iterations, kernel_threshold, damping, keep_outliers);
    
    
    Dataset dataset(dataset_filename);

    vector<vector<MapPoint>> map;

    Vector2dVector points;
    Vector3dVector poses;
    Vector3dVector sensor_poses;
    
    cout << "Loading data.." << endl;

    dataset.load_data(poses, sensor_poses, points, map, dataset_from_record_number, dataset_num_records, 0.3, 0.1);
    // dataset.load_data(poses, sensor_poses, points, map, dataset_from_record_number, dataset_num_records, 0.1, 0.02);
    // dataset.load_data(poses, sensor_poses, points, map, dataset_from_record_number, dataset_num_records, DBL_MIN, DBL_MIN);


    size_t num_poses = poses.size();
    size_t num_points = points.size();

    cout << num_poses << " poses have been loaded." << endl;
    cout << num_points << " points have been loaded." << endl;


    cout << "Loading data complete." << endl << endl;

    cout << "Computing data mean.." << endl;
    Vector2d data_mean;
    Matrix2d data_covariance;
    data_mean.setZero();
    data_covariance.setZero();

    for (size_t pose_index = 0; pose_index < map.size(); pose_index++)
    {
      for (size_t point_index = 0; point_index < map[pose_index].size(); point_index++)
      {
        MapPoint map_point = map[pose_index][point_index];
        Vector2d world_point = v2t(poses[map_point.pose_index()]) * points[map_point.point_index()];
        data_mean += world_point;
        data_covariance += world_point * world_point.transpose();

      }
      
    }

    data_mean *= (1./num_points);
    data_covariance *= (1./num_points);
    data_covariance -= data_mean * data_mean.transpose();
    data_covariance *= num_points/(num_points-1);
    cout << "Data mean: " << data_mean.transpose() << endl;
    cout << "Data covariance: " << endl << data_covariance << endl;

    
    // apply sensor offset to pose (it is a semplification not strictly needed).
    for (size_t pose_index = 0; pose_index < num_poses; pose_index++)
    {
        poses[pose_index] = t2v(v2t(poses[pose_index]) * v2t(sensor_poses[pose_index]));  // aply sensor offset
        poses[pose_index].block(0, 0, 2, 1) -= data_mean; // apply standardization
        // poses[pose_index].block(0, 0, 1, 1) /= data_covariance(0, 0); // apply standardization
        // poses[pose_index].block(1, 0, 1, 1) /= data_covariance(1, 1); // apply standardization
        
    }
    
    

    cout << "Writing on file.." << endl;
    
    ofstream points_file, poses_file;
    points_file.open ("points.dat");
    poses_file.open ("poses.dat");

    for (size_t pose_index = 0; pose_index < map.size(); pose_index++)
    {
        for (size_t point_index = 0; point_index < map[pose_index].size(); point_index++)
        {
            
            MapPoint map_point = map[pose_index][point_index];

            Vector2d point = points[map_point.point_index()];
            Vector3d pose = poses[map_point.pose_index()];

            Vector2d world_point = v2t(pose) * point;
            Vector2d world_pose = pose.block(0, 0, 2, 1);
            
            points_file << world_point.transpose() << endl;
            poses_file << world_pose.transpose() << endl;

        }
        
    }
    
    
    points_file.close();
    poses_file.close();

    cout << "Writing on file terminated" << endl;
    
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
        if (finder.find_point_neighbors(pose_index, map_point.pose_index(), map_point.point_index(), 0.3, point_neighbors))
        // if (finder.find_point_neighbors(pose_index, map_point.pose_index(), map_point.point_index(), 10, point_neighbors))
        {
          if (point_neighbors.size() < (size_t) min_local_correspondences) continue;
          IntVector indices;
          for (size_t i = 0; i < point_neighbors.size(); i++){
            MapPoint p = map[pose_index][point_neighbors[i].second];
            indices.push_back(p.point_index());
          } 

          // double normal_vector_angle;
          // if (estimate_normal(points, indices, normal_vector_angle)) {
          //   Vector2d normal;
          //   normal << cos(normal_vector_angle), sin(normal_vector_angle);
          //   normals.push_back(normal);
          //   map_point.set_normal_index(normals.size() - 1);
          //   // cout << "classic normal " << endl << normal << endl << "(norm: " << normal.norm() << " )"<< endl;
          // }
          
          Vector2d normal;
          if (estimate_normal(points, indices, normal)) {
            normals.push_back(normal);
            map_point.set_normal_index(normals.size() - 1);

          }
          
        }
      }
    }
    
    cout << normals.size() << " normals have been estimated." << endl;
    
    cout << "Estimating surface normals complete." << endl;
 
    
    cout << "Init Multi ICP Solver.." << endl;
    
    MultiICPSolver solver;
    Eigen::VectorXd state;
    solver.init(state, poses, points, normals, kernel_threshold, damping);
    
    cout << "Init Multi ICP Solver ok." << endl;

    vector<TriplePairVector> correspondences;
    IntPairVector poses_correspondences;
    Eigen::MatrixXi fill_in(num_poses, num_poses);
    
    char key=0;
    const char ESC_key=27;
    while (key!=ESC_key) {
      
        cout << "Press Enter to run an optimization step.." << endl;
        key = getchar();

        if (key == ESC_key) continue;
          
        // update correspondences
        cout << "Updating global correspondences." << endl;
        
        correspondences.clear();

        poses_correspondences.clear();

        fill_in.setZero();

        for (size_t pose_index = 0; pose_index < map.size(); pose_index++)
        { 
          IntVector pose_neighbors;
          if (!finder.find_pose_neighbors(pose_index, 4.0, pose_neighbors)) continue;
          // if (!finder.find_pose_neighbors(pose_index, 5, pose_neighbors)) continue;

          for (size_t index = 0; index < pose_neighbors.size(); index++)
          {
            int pose_neighbor_index = pose_neighbors[index];
            TriplePairVector pose_pose_correspondences;
            
            // if (pose_index >= pose_neighbor_index) continue;
            if (pose_index == pose_neighbor_index) continue;
            
            Vector3d pose = poses[pose_index];
            Vector3d pose_neighbor = poses[pose_neighbor_index];
            Vector2d pose_dir = Vector2d(cos(pose.z()), sin(pose.z()));
            Vector2d pose_neighbor_dir = Vector2d(cos(pose_neighbor.z()), sin(pose_neighbor.z()));
            // cout << acos(pose_dir.dot(pose_neighbor_dir)) << endl;
            // if the difference in the orientation between two nearest poses is greater than 90 degrees go to next iteration
            if (pose_dir.dot(pose_neighbor_dir) < 0) continue; // angle greter than 90 degrees
            poses_correspondences.push_back(IntPair(pose_index, pose_neighbor_index));
            
            for (size_t point_index = 0; point_index < map[pose_index].size(); point_index++)
            {
              MapPoint map_point = map[pose_index][point_index];
              if (!map_point.has_normal()) continue;
              
              pair<double, int> point_global_neighbor;
              if (!finder.find_point_neighbor(pose_neighbor_index, map_point.pose_index(), map_point.point_index(), point_global_neighbor, 0.3)) continue;

              MapPoint map_neighbor_point = map[pose_neighbor_index][point_global_neighbor.second];
              if (!map_neighbor_point.has_normal()) continue;

              Vector2d point_normal = v2t(poses[map_point.pose_index()]).rotation() * normals[map_point.normal_index()];
              Vector2d neighbor_point_normal = v2t(poses[map_neighbor_point.pose_index()]).rotation() * normals[map_neighbor_point.normal_index()]; 
              
              if (point_normal.dot(neighbor_point_normal) < 0.9) continue; // 20 degrees
              IntTriple src = IntTriple(map_point.pose_index(), map_point.point_index(), map_point.normal_index());
              IntTriple dst = IntTriple(map_neighbor_point.pose_index(), map_neighbor_point.point_index(), map_neighbor_point.normal_index());
              
              pose_pose_correspondences.push_back(TriplePair(make_pair(src, dst)));
              // pose_pose_correspondences.push_back(TriplePair(make_pair(dst, src)));

            }

            // size_t num_pose_pose_correspondences = pose_pose_correspondences.size() / 2;
            size_t num_pose_pose_correspondences = pose_pose_correspondences.size();
            if (num_pose_pose_correspondences < min_poses_correspondences) continue;

            fill_in(pose_index, pose_neighbor_index) = num_pose_pose_correspondences;
            // fill_in(pose_neighbor_index, pose_index) = num_pose_pose_correspondences;
            
            // if (pose_pose_correspondences.size() < min_poses_correspondences) continue;
            // fill_in(pose_index, pose_neighbor_index) = pose_pose_correspondences.size();
            // each block correspondes to  pose-i pose-j correspondences
            correspondences.push_back(pose_pose_correspondences); // put the block of correspondences in the array 

            
          }
          
        }
                  
        // cout << num_correspondences << endl;
        cout << "Updating global correspondences ok." << endl;
        cout << "ok." << endl << endl;
        
        cout << "Total correspondences: " << correspondences.size() << endl;
        
        cout << "# Multi-ICP optimization .." << endl;
        solver.oneRound(correspondences, keep_outliers);
        cout << "# inliers: " << solver.numInliers() << " ( error: " << solver.chiInliers()<< " )" << " outliers: " << solver.numOutliers() << " ( error: " << solver.chiOutliers() << " ). " << endl;
        
        // cout << state << endl;

        // update poses
        for (size_t pose_index = 0; pose_index < num_poses; pose_index++)
        {
          Vector3d pose = poses[pose_index];
          Vector3d pert = state.segment<3>(pose_index*3);

          poses[pose_index] = t2v(v2t(pert) * v2t(pose));
          // poses[pose_index] = t2v(v2t(pose) * v2t(pert));
        }

        cout << "Writing on file.." << endl;
  
        ofstream points_file, poses_file;
        points_file.open ("points_opt.dat");
        poses_file.open ("poses_opt.dat");

        for (size_t pose_index = 0; pose_index < map.size(); pose_index++)
        {
            for (size_t point_index = 0; point_index < map[pose_index].size(); point_index++)
            {
                
                MapPoint map_point = map[pose_index][point_index];

                Vector2d point = points[map_point.point_index()];
                Vector2d world_point = v2t(poses[map_point.pose_index()]) * point;
                points_file << world_point.transpose() << endl;
                poses_file << poses[map_point.pose_index()].block(0, 0, 2, 1).transpose() << endl;

            }
            
        }
        
        points_file.close();
        poses_file.close();

        cout << "Writing on file complete." << endl;
  
      
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


  return 0;
}
    
    