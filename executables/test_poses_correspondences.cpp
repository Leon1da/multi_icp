#include "defs.h"

#include "dataset.h"
#include "drawer.h"

#include "utils.h"
#include "kdtree.h"


int main (int argc, char** argv) {
    
  
    cout << "Correspondence Finder." << endl << endl;
    
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

    finder.load_poses_2dtree();
    finder.load_poses_4dtree();

    cout << "Loading correspondence finder complete." << endl;
    
    cout << "Finding for global correspondences." << endl;
    
    IntPairVector poses_correspondences;

    for (size_t pose_index = 0; pose_index < map.size(); pose_index++)
    {

      IntVector pose_neighbors;
      // bool pose_found = finder.find_pose_neighbors(pose_index, 5, pose_neighbors);
      bool pose_found = finder.find_pose_neighbors(pose_index, 0.5, pose_neighbors, 4);
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
  
    Drawer drawer(width, height, "test_correspondences");

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