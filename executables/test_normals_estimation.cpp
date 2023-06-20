#include "defs.h"

#include "dataset.h"
#include "drawer.h"

#include "utils.h"
#include "kdtree.h"
#include "icp_solver.h"
#include "utils.h"


int main (int argc, char** argv) {
    
  
    cout << "Multi Iterative Closest Points (IPC)." << endl << endl;
    
    std::string dataset_filename = "/home/leonardo/multi_icp/dataset/dataset_test.txt";

    Dataset dataset(dataset_filename);

    IntPairVector pose_point_correspondences;
    vector<Vector2fVector> points;
    vector<vector<int>> valid_points;
    Vector3fVector poses;
    Vector3fVector sensor_poses;
    
    cout << "Loading data.." << endl;

    dataset.load_data(poses, sensor_poses, points, pose_point_correspondences, valid_points, 0, 10);

    for (size_t pose_index = 0; pose_index < poses.size(); pose_index++)
    {
        poses[pose_index] = t2v(v2t(poses[pose_index]) * v2t(sensor_poses[pose_index]));
    }
    
    cout << poses.size() << " poses have been loaded." << endl;

    size_t num_points = 0;
    for (size_t pose_index = 0; pose_index < poses.size(); pose_index++)
      for (size_t point_index = 0; point_index < points[pose_index].size(); point_index++)
        if (valid_points[pose_index][point_index]) num_points++;

    cout << num_points << " valid points have been loaded." << endl;


    cout << "Loading data complete." << endl << endl;
    
    
    cout << "Loading correspondence finder." << endl;

    kdt::CorrespondenceFinder finder;
    finder.init(poses, points, pose_point_correspondences);

    cout << "Loading correspondence finder complete." << endl << endl;

    vector<Vector2fVector> normals(valid_points.size());
    for (size_t i = 0; i < valid_points.size(); i++)
    {
        normals[i] = Vector2fVector(valid_points[i].size());
    }
    
    cout << "Finding for local correspondences." << endl;
    
    FloatPair sv;

    for (size_t index = 0; index < pose_point_correspondences.size(); index++)
    {
      IntPair pose_point_correspondence = pose_point_correspondences[index];
      IntPairVector local_neighbors;
      if (finder.find_local_neighbors(pose_point_correspondence, 11, local_neighbors))
      {

        Eigen::Matrix2f covariance;
        compute_covariance(points, local_neighbors, covariance);
        

        // Eigen::JacobiSVD<Matrix2f, Eigen::ComputeFullU | Eigen::ComputeFullV> svd(covariance);
        // Eigen::Vector2f principal_axis = svd.matrixV().block(0, 1, 2, 1);
        // // principal_axis.normalize();
        // cout << principal_axis / principal_axis.norm() << endl << endl;


        // cout << svd.matrixV() << endl << endl;

        // int pose_index = pose_point_correspondence.first;
        // int point_index = pose_point_correspondence.second;
        // normals[pose_index][point_index] = principal_axis;
        
        // cout << "Its singular values are:" << endl << svd.singularValues().y() << endl;

        // break;
        // // estimate normals EIGEN DECOMPOSITION
        // Vector2f normal;
        // if (normal_of_points(points, local_neighbors, normal)){
            
        //     int pose_index = pose_point_correspondence.first;
        //     int point_index = pose_point_correspondence.second;
        //     normals[pose_index][point_index] = normal;
        // }
        
        // estimate normals LEAST SQUARES
        float normal_angle;
        if (normal_of_points(points, local_neighbors, normal_angle)){
            
            int pose_index = pose_point_correspondence.first;
            int point_index = pose_point_correspondence.second;
            Vector2f normal;
            normal << cos(normal_angle), sin(normal_angle);
        
            normals[pose_index][point_index] = normal;
        }
      }
    

      // cout << '\r' << "Correspondence " << index << " / " << pose_point_correspondences.size() << "." << flush;
      
      // cout << "Local neighbors" << endl;
      // for (auto n : local_neighbors) cout << " [ " << n.first << " " << n.second << " ]" << endl;
      
    }



 

    cout << endl;
    
    cout << "Finding for local correspondences ok." << endl;
    
    cout << "Finding for global correspondences." << endl;
    
    // finding for global correspondences
   
    vector<Correspondence> correspondences;
   
    for (size_t index = 0; index < pose_point_correspondences.size(); index++)
    {
      IntPair pose_point_correspondence = pose_point_correspondences[index];
      IntPair neighbor;
      if (finder.find_global_neighbor(pose_point_correspondence, neighbor))
      {
        Correspondence c(pose_point_correspondence, neighbor);
        correspondences.push_back(c);
      }

      // cout << '\r' << "Correspondence " << index << " / " << pose_point_correspondences.size() << "." << flush;
      
      // cout << "Global neighbors" << endl;
      // for (auto n : global_neighbors) cout << " [ " << n.first << " " << n.second << " ]" << endl;
      
    }

    cout << endl;

    cout << "Finding for global correspondences ok." << endl;

    
    // for (auto c : correspondences) cout << "[ " << c.first.first << " " << c.first.second << " ][ " << c.second.first << " " << c.second.second << " ]" << endl;
    


    int width = 800;
    int height = 800;
  
    Drawer drawer(width, height, "test_map_correspondence");

    
    
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
        
        

        drawer.drawPoses(poses, drawer_controller, red);
        
        for (size_t pose_index = 0; pose_index < poses.size(); pose_index++)
        {
          // draw pose
          drawer.drawPose(poses[pose_index], drawer_controller, red);

          
          // draw point and normals
          for (size_t point_index = 0; point_index < points[pose_index].size(); point_index++)
          { 
            if (valid_points[pose_index][point_index]) {
              
              Vector2f point_world = points[pose_index][point_index];
              point_world = v2t(poses[pose_index]) * point_world;
              drawer.drawPoint(point_world, drawer_controller, black); 

              Vector2f point_normal = normals[pose_index][point_index];
              point_normal = v2t(poses[pose_index]).rotation() * point_normal;
            //   cout << endl << point_normal << endl << endl;

              Vector2f start_point, end_point;
              start_point = point_world; // normal vector start from the point
              end_point = point_world + point_normal * 0.05;
            //   end_point = point_world + point_normal;
              drawer.drawLine(start_point, end_point, drawer_controller, green);
            
            
            }
          }
          
        }

        // // draw correspondences
        // for (size_t correspondece_index = 0; correspondece_index < correspondences.size(); correspondece_index++)
        // {
        //   Correspondence c = correspondences[correspondece_index];
        //   Vector2f start_point, end_point;
        //   start_point = v2t(poses[c.first.first]) * v2t(sensor_poses[c.first.first]) * points[c.first.first][c.first.second];
        //   end_point = v2t(poses[c.second.first]) * v2t(sensor_poses[c.second.first]) * points[c.second.first][c.second.second];
        //   drawer.drawLine(start_point, end_point, drawer_controller, green);
        
        // }
        

        drawer.show();

        key=cv::waitKey(0);

        drawer_controller.update(key);

    }


  return 0;
}
    
    