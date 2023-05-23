#include "defs.h"

#include "dataset.h"
#include "map.h"
#include "drawer.h"

int main (int argc, char** argv) {
    
  
    std::cout << "Multi Iterative Closest Points (IPC)." << std::endl;
    
    std::string dataset_filename = "/home/leonardo/multi_icp/dataset/dataset_test.txt";

    Dataset dataset(dataset_filename);
    Map map;
    
    for(auto record : dataset.records()){
        Pose2D pose = record_to_pose(record);   
        vector<Point2D> points = record_to_points(record);   
        map.add_pose(pose);
        map.add_points(points);
    }


    int width = 1680;
    int height = 1050;
  
    Drawer drawer(width, height, "test_dataset_reader");

    DrawerController drawer_controller(height/4, width/4, 5.0, 5.0, 5.0, 5.0);
  

    char key=0;
    const char ESC_key=27;
    while (key!=ESC_key) {

        drawer.clear();
        
        drawer.drawMap(map, drawer_controller);

        drawer.show();

        key=cv::waitKey(0);

        drawer_controller.update(key);

        
    }


  return 0;
}