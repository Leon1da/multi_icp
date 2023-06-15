#include "defs.h"

#include "dataset.h"
#include "drawer.h"

#include "utils.h"


int main (int argc, char** argv) {
    
  
    cout << "Multi Iterative Closest Points (IPC)." << endl << endl;
    
    std::string dataset_filename = "/home/leonardo/multi_icp/dataset/dataset_test.txt";

    Dataset dataset(dataset_filename);

    IntPairVector pose_point_correspondences;
    Vector2fVector points;
    Vector3fVector poses;
    
    cout << "Loading data.." << endl;

    dataset.load_data(poses, points, pose_point_correspondences);

    
    cout << poses.size() << " poses have been loaded." << endl;
    cout << points.size() << " points have been loaded." << endl;
    
    cout << "Loading data complete." << endl << endl;

            
    int width = 800;
    int height = 800;
  
    Drawer drawer(width, height, "test_dataset_reader");

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
        
        drawer.drawPoints(points, drawer_controller, black);
        
        drawer.show();

        key=cv::waitKey(0);

        drawer_controller.update(key);

    }


  return 0;
}