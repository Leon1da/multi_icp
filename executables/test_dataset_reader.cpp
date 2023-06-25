#include "defs.h"

#include "dataset.h"
#include "drawer.h"

#include "utils.h"


int main (int argc, char** argv) {
    
  
    cout << "Dataset Reader." << endl << endl;
    
    std::string dataset_filename = "/home/leonardo/multi_icp/dataset/dataset_test.txt";

    Dataset dataset(dataset_filename);

    vector<vector<MapPoint>> map;

    Vector2fVector points;
    Vector3fVector poses;
    Vector3fVector sensor_poses;
    
    cout << "Loading data.." << endl;

    dataset.load_data(poses, sensor_poses, points, map, 0, 20);


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
    
            
    int width = 800;
    int height = 800;
  
    Drawer drawer(width, height, "test_dataset_reader");

    DrawerController drawer_controller(height/4, width/4, 1.0, 5.0, 5.0, 5.0);
    
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
          
          }
          
        }
        

        
        drawer.show();

        key=cv::waitKey(0);

        drawer_controller.update(key);

    }


  return 0;
}