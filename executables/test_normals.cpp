#include "defs.h"

#include "dataset.h"
#include "kdtree.h"
#include "utils.h"

#include "drawer.h"


int main (int argc, char** argv) {
    
  
    cout << "Normal Estimation." << endl << endl;
    
    std::string dataset_filename = "/home/leonardo/multi_icp/dataset/dataset_test.txt";

    Dataset dataset(dataset_filename);

    vector<vector<MapPoint>> map;

    Vector2fVector points;
    Vector3fVector poses;
    Vector3fVector sensor_poses;
    
    
    cout << "Loading data.." << endl;

    dataset.load_data(poses, sensor_poses, points, map, 10, 1);


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
    finder.init(poses, points, map);

    cout << "Loading correspondence finder complete." << endl << endl;
    
    cout << "Finding correspondences." << endl;
    
    cout << "Finding for local correspondences." << endl;

    vector<vector<Correspondence>> local_correspondences;
    for (size_t pose_index = 0; pose_index < map.size(); pose_index++)
    {
      for (size_t point_index = 0; point_index < map[pose_index].size(); point_index++)
      {
        MapPoint& map_point = map[pose_index][point_index];       
        // Finding for local correspondences 
        IntPairVector local_neighbors;
        // if (finder.find_local_neighbors(map_point.pose_index(), map_point.point_index(), 10, local_neighbors))
        if (finder.find_local_neighbors(map_point.pose_index(), map_point.point_index(), 0.05, local_neighbors))
        {
          size_t num_neighbors = local_neighbors.size();
          if (num_neighbors){
            vector<Correspondence> cs;
            for (size_t local_neighbor_index = 0; local_neighbor_index < num_neighbors; local_neighbor_index++)
            {
              cs.push_back(Correspondence(IntPair(map_point.pose_index(), map_point.point_index()), local_neighbors[local_neighbor_index]));
            }
            local_correspondences.push_back(cs);
            map_point.set_local_correspondences_index(local_correspondences.size() - 1, num_neighbors);
          }
          // cout << "[ " << pose_index << " " << point_index << " ] " << local_neighbors.size() << " local_neighbors found." << endl;
        }
      }
    }

    cout << "Finding for local correspondences ok." << endl;
       
    cout << "Finding correspondences ok." << endl;

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
    
    cout << "Estimating surface normals complete." << endl;
    



    int width = 800;
    int height = 800;
  
    Drawer drawer(width, height, "test_normals");

    
    
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
          Vector3f world_pose = poses[pose_index];
          drawer.drawPose(world_pose, drawer_controller, red);

          for (size_t point_index = 0; point_index < map[pose_index].size(); point_index++)
          {

            MapPoint map_point = map[pose_index][point_index];

            Vector2f world_point = v2t(poses[map_point.pose_index()]) * points[map_point.point_index()];
            
            drawer.drawPoint(world_point, drawer_controller, black);

            // draw normal
            if (map_point.has_normal())
            {
              Vector2f world_normal = world_point + v2t(poses[map_point.pose_index()]).rotation() * normals[map_point.normal_index()] * 0.05; // It's ok
              drawer.drawLine(world_point, world_normal, drawer_controller, green);
              
            }
            
    
          }
          
        }       

        drawer.show();

        key=cv::waitKey(0);

        drawer_controller.update(key);

    }


  return 0;
}
    
    