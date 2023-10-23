#include "defs.h"

#include "dataset.h"
#include "utils.h"
#include "drawer.h"
#include "solver.h"
#include "eigen_kdtree.h"


using ScalarType = double;
using PoseType = Eigen::Matrix<ScalarType, 3, 1>;
using PointType = Eigen::Matrix<ScalarType, 2, 1>;

using PointContainerType = std::vector<PointType, Eigen::aligned_allocator<PointType>>;
using PoseContainerType = std::vector<PoseType, Eigen::aligned_allocator<PoseType>>;

using TreeContainerType = PointContainerType;
using TreeNodeType = TreeNode_<TreeContainerType::iterator>;
using AnswerType = TreeNodeType::AnswerType;

using VectorTreeNodeType = std::vector<TreeNodeType>;


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

    string file_label;
    string file_dir = "results/";

    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y_%m_%d_%H_%M_%S") << "_conf_it_" << iterations << "_kt_" << kernel_threshold << "_dm_" << damping << "_";
    file_label = oss.str();

    
    Dataset dataset(dataset_filename);

    vector<vector<MapPoint>> map;

    PointContainerType points;
    PoseContainerType poses;
    PoseContainerType sensor_poses;

    
    cout << "Loading data.." << endl;

    // dataset.load_data(poses, sensor_poses, points, map, dataset_from_record_number, dataset_num_records, 0.1);
    // dataset.load_data(poses, sensor_poses, points, map, dataset_from_record_number, dataset_num_records, DBL_MIN, DBL_MIN);
    dataset.load_data(poses, sensor_poses, points, map, dataset_from_record_number, dataset_num_records, 0.3, 0.1);
    

    size_t num_poses = poses.size();
    size_t num_points = points.size();

    cout << num_poses << " poses have been loaded." << endl;
    cout << num_points << " points have been loaded." << endl;

    // apply sensor offset to each pose
    for (size_t pose_index = 0; pose_index < num_poses; pose_index++) poses[pose_index] = t2v(v2t(poses[pose_index]) * v2t(sensor_poses[pose_index]));
    

    cout << "Loading data complete." << endl << endl;

    cout << "Writing on file.." << endl;
    
    ofstream points_file, poses_file, points_opt_file, poses_opt_file, poses_correspondences_file, points_correspondences_file, statistics_file;
    points_file.open (file_dir + file_label + "points.dat");
    poses_file.open (file_dir + file_label + "poses.dat");
    
    for (size_t pose_index = 0; pose_index < map.size(); pose_index++)
    {
        for (size_t point_index = 0; point_index < map[pose_index].size(); point_index++)
        {
            
            MapPoint map_point = map[pose_index][point_index];
            
            PointType point = points[map_point.point_index()];
            PoseType pose = poses[map_point.pose_index()];

            PointType world_point = v2t(pose) * point;
            PointType world_pose = pose.block(0, 0, 2, 1);
            
            points_file << world_point.transpose() << endl;
            poses_file << world_pose.transpose() << endl;

        }
        
    }
    
    points_file.close();
    poses_file.close();

    cout << "Writing on file terminated" << endl;
   
    cout << "Loading poses kdtree." << endl;
    
    TreeContainerType poses_container;
    IntVector poses_indices;
    for (size_t pose_index = 0; pose_index < map.size(); pose_index++)
    {
        poses_container.push_back(poses[pose_index].block(0, 0, 2, 1)); 
        poses_indices.push_back(pose_index);
   
    }
    

    TreeNodeType kdtree_poses(poses_container.begin(), poses_container.end(), poses_indices.begin(), poses_indices.end(), 5);

    cout << "Loading poses kdtree ok." << endl;

    cout << "Loading points kdtree.." << endl;

    VectorTreeNodeType kdtree_points_vector;

    vector<TreeContainerType> points_container_vector;
    vector<IntVector> points_indices_vector;
    for (size_t pose_index = 0; pose_index < map.size(); pose_index++)
    {
      IntVector points_indices;
      TreeContainerType points_container;
      for (size_t point_index = 0; point_index < map[pose_index].size(); point_index++)
      {
        points_container.push_back(points[map[pose_index][point_index].point_index()]);
        points_indices.push_back(point_index);
      }

      points_container_vector.push_back(points_container);
      points_indices_vector.push_back(points_indices);

      TreeNodeType kdtree_points(points_container_vector[pose_index].begin(), points_container_vector[pose_index].end(), points_indices_vector[pose_index].begin(), points_indices_vector[pose_index].end(), 10);
      kdtree_points_vector.push_back(move(kdtree_points));
    }

    cout << "Loading points kdtree ok." << endl;

    cout << "Estimating surface normals.." << endl;
    
    PointContainerType normals;
    
    for (size_t pose_index = 0; pose_index < map.size(); pose_index++)
    {
      for (size_t point_index = 0; point_index < map[pose_index].size(); point_index++)
      {
        MapPoint& map_point = map[pose_index][point_index];

        TreeNodeType::AnswerType leaf_points;
        PointType query_point = points[map_point.point_index()];
        
        kdtree_points_vector[pose_index].getLeafPoints(leaf_points, query_point);
        size_t num_leaf_nodes = leaf_points.size();

        if (num_leaf_nodes < (size_t) min_local_correspondences) continue;

        PointContainerType neighbor_container(leaf_points.size());
        for (size_t neighbor_index = 0; neighbor_index < leaf_points.size(); neighbor_index++) neighbor_container[neighbor_index] = *leaf_points[neighbor_index];
        CovarianceType cov;
        PointType mean, normal;
        computeMeanAndCovariance(mean, cov, neighbor_container.begin(), neighbor_container.end());
        normal = smallestEigenVector(cov);
        
        normals.push_back(move(normal));
        map_point.set_normal_index(normals.size() - 1);

        
        // PointType query_point = points[map_point.point_index()];
        // AnswerType point_neighbors;
        // kdtree_points_vector[pose_index].fullSearch(point_neighbors, query_point, 0.3);
        // size_t num_neighbors = point_neighbors.size();

        // if (num_neighbors < (size_t) min_local_correspondences) continue;

        // PointContainerType neighbor_container(num_neighbors);
        // for (size_t neighbor_index = 0; neighbor_index < num_neighbors; neighbor_index++) neighbor_container[neighbor_index] = *point_neighbors[neighbor_index];
        // CovarianceType cov;
        // PointType mean, normal;
        // computeMeanAndCovariance(mean, cov, neighbor_container.begin(), neighbor_container.end());
        // normal = smallestEigenVector(cov);
        // normals.push_back(normal);
        // map_point.set_normal_index(normals.size() - 1);

      }
    }
    
    cout << normals.size() << " normals have been estimated." << endl;
    
    cout << "Estimating surface normals complete." << endl;

    cout << "Init Multi ICP Solver.." << endl;
    
    NICP2dSolver solver;
    Eigen::VectorXd state;
    solver.init(state, poses, points, normals, kernel_threshold, damping);
    
    statistics_file.open (file_dir + file_label + "statistics.dat");
    
    statistics_file << "iteration, max_inliers, max_chi_inliers, max_outliers, max_chi_outliers, tot_inliers, tot_chi_inliers, tot_outliers, tot_chi_outliers" << endl;

    cout << "Init Multi ICP Solver ok." << endl;

    vector<TriplePairVector> correspondences;
    IntPairVector poses_correspondences;
    Eigen::MatrixXi fill_in(num_poses, num_poses);

    cout << setw(15) << fixed  << setprecision(2) << "iteration";
    cout << setw(15) << fixed  << setprecision(2) << "inliers [max/tot]";
    cout << setw(15) << fixed  << setprecision(2) << "error [max/tot]";
    cout << setw(15) << fixed  << setprecision(2) << "outliers [max/tot]";
    cout << setw(15) << fixed  << setprecision(2) << "error [max/tot]";
    cout << endl;
    
    for (size_t iteration = 0; iteration < iterations; iteration++)
    {
      correspondences.clear();
      poses_correspondences.clear();
      fill_in.setZero();

      for (size_t pose_index = 0; pose_index < map.size(); pose_index++)
      { 

        TreeNodeType::AnswerType pose_neighbors;
        PointType query_pose = poses[pose_index].block(0, 0, 2, 1);
        kdtree_poses.fullSearch(pose_neighbors, query_pose, 4.0);
        
        size_t num_pose_neighbors = pose_neighbors.size();
        IntVector pose_neighbors_indices(num_pose_neighbors);
        for (size_t npose = 0; npose < num_pose_neighbors; npose++)
        {
          int index = poses_indices[pose_neighbors[npose] - &poses_container[0]];
          pose_neighbors_indices[npose] = index;
        }
        
        for (size_t index = 0; index < pose_neighbors.size(); index++)
        {
          int pose_neighbor_index = pose_neighbors_indices[index];
          TriplePairVector pose_pose_correspondences;
          
          // if (pose_index >= pose_neighbor_index) continue;
          if (pose_index == pose_neighbor_index) continue;
          
          PoseType pose = poses[pose_index];
          PoseType pose_neighbor = poses[pose_neighbor_index];
          PointType pose_dir = PointType(cos(pose.z()), sin(pose.z()));
          PointType pose_neighbor_dir = PointType(cos(pose_neighbor.z()), sin(pose_neighbor.z()));
          // if the difference in the orientation between two nearest poses is greater than 90 degrees go to next iteration
          if (pose_dir.dot(pose_neighbor_dir) < 0) continue; // angle greter than 90 degrees
          poses_correspondences.push_back(IntPair(pose_index, pose_neighbor_index));
          
          for (size_t point_index = 0; point_index < map[pose_index].size(); point_index++)
          {
            MapPoint map_point = map[pose_index][point_index];
            if (!map_point.has_normal()) continue;

            
            PointType query_point = v2t(poses[pose_neighbor_index]).inverse() * v2t(poses[map_point.pose_index()]) * points[map_point.point_index()];
            PointType* neighbor_point = kdtree_points_vector[pose_neighbor_index].bestMatchFull(query_point, 0.3);
            // PointType* neighbor_point = kdtree_points_vector[pose_neighbor_index].bestMatchFull(query_point, 0.1);
            if (!neighbor_point) continue;

            int neighbor_point_index = points_indices_vector[pose_neighbor_index][neighbor_point - &points_container_vector[pose_neighbor_index][0]];
            
            MapPoint map_neighbor_point = map[pose_neighbor_index][neighbor_point_index];
            if (!map_neighbor_point.has_normal()) continue;

            PointType point_normal = v2t(poses[map_point.pose_index()]).rotation() * normals[map_point.normal_index()];
            PointType neighbor_point_normal = v2t(poses[map_neighbor_point.pose_index()]).rotation() * normals[map_neighbor_point.normal_index()]; 
            
            double prod = point_normal.dot(neighbor_point_normal);
            if (prod < 0.9 ) continue; 
            
            IntTriple src = IntTriple(map_point.pose_index(), map_point.point_index(), map_point.normal_index());
            IntTriple dst = IntTriple(map_neighbor_point.pose_index(), map_neighbor_point.point_index(), map_neighbor_point.normal_index());
            
            pose_pose_correspondences.push_back(TriplePair(make_pair(src, dst)));
            // pose_pose_correspondences.push_back(TriplePair(make_pair(dst, src)));

          }

          size_t num_pose_pose_correspondences = pose_pose_correspondences.size();
          if (num_pose_pose_correspondences < min_poses_correspondences) continue;
          fill_in(pose_index, pose_neighbor_index) = num_pose_pose_correspondences;
          
          // each block correspondes to  pose-i pose-j correspondences
          correspondences.push_back(pose_pose_correspondences); // put the block of correspondences in the array 

          
        }
        
      }

      solver.oneRound(correspondences, keep_outliers);
      
      // log on shell
      cout << setw(15) << fixed  << setprecision(2) << iteration;
      cout << setw(20) << fixed  << setprecision(2) << solver.numMaxInliers();
      cout << setw(20) << fixed  << setprecision(4) << solver.chiMaxInliers();
      cout << setw(20) << fixed  << setprecision(2) << solver.numMaxOutliers();
      cout << setw(20) << fixed  << setprecision(4) << solver.chiMaxOutliers();
      cout << endl;
      
      cout << setw(15) << fixed  << setprecision(2) << "";
      cout << setw(20) << fixed  << setprecision(2) << solver.numTotInliers();
      cout << setw(20) << fixed  << setprecision(4) << solver.chiTotInliers();
      cout << setw(20) << fixed  << setprecision(2) << solver.numTotOutliers();
      cout << setw(20) << fixed  << setprecision(4) << solver.chiTotOutliers();
      cout << endl;

      statistics_file << iteration << ", " << solver.numMaxInliers() << ", " << solver.chiMaxInliers() << ", " << solver.numMaxOutliers() << ", " << solver.chiMaxOutliers()
      << ", " << solver.numTotInliers() << ", " << solver.chiTotInliers() << ", " << solver.numTotOutliers() << ", " << solver.chiTotOutliers() << endl;
      
      
      // update poses
      for (size_t pose_index = 0; pose_index < num_poses; pose_index++)
      {
        PoseType pose = poses[pose_index];
        PoseType pert = state.segment<3>(pose_index*3);
        poses[pose_index] = t2v(v2t(pert) * v2t(pose));

      }

      // updating pose kdtree 
      poses_container.clear();
      poses_indices.clear();
      for (size_t pose_index = 0; pose_index < map.size(); pose_index++)
      {
        poses_container.push_back(poses[pose_index].block(0, 0, 2, 1)); 
        poses_indices.push_back(pose_index);
      }
        
      TreeNodeType kdtree_poses(poses_container.begin(), poses_container.end(), poses_indices.begin(), poses_indices.end());

    }

    cout << "Writing on file.." << endl;
  
    points_opt_file.open (file_dir + file_label + "points_opt.dat");
    poses_opt_file.open (file_dir + file_label + "poses_opt.dat");
    
    for (size_t pose_index = 0; pose_index < map.size(); pose_index++)
    {
        for (size_t point_index = 0; point_index < map[pose_index].size(); point_index++)
        {
            
            MapPoint map_point = map[pose_index][point_index];

            PointType point = points[map_point.point_index()];
            PoseType pose = poses[map_point.pose_index()];

            // apply normalization
            PointType world_point = v2t(pose) * point;
            PointType world_pose = pose.block(0, 0, 2, 1);
            
            points_opt_file << world_point.transpose() << endl;
            poses_opt_file << world_pose.transpose() << endl;

        }    
    }

    
    points_opt_file.close();
    points_opt_file.close();

    cout << "Writing on file complete." << endl;
    
    
    return 1;



}