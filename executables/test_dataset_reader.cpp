#include "defs.h"

#include "dataset.h"
#include "drawer.h"

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
        
        
        for (size_t pose_index = 0; pose_index < points.size(); pose_index++)
        {
          // draw pose
          drawer.drawPose(poses[pose_index], drawer_controller, red);
          
          // draw point
          for (size_t point_index = 0; point_index < points[pose_index].size(); point_index++)
          { 
            if (valid_points[pose_index][point_index]) {
              Vector2f point_world = points[pose_index][point_index];
              point_world = v2t(poses[pose_index]) * point_world;
              drawer.drawPoint(point_world, drawer_controller, black); 
            
            }
          }
          
        }


        
        drawer.show();

        key=cv::waitKey(0);

        drawer_controller.update(key);

    }


  return 0;
}