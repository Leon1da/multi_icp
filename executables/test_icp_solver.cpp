#include "defs.h"

#include "dataset.h"
#include "drawer.h"

#include "utils.h"
#include "kdtree.h"
#include "icp_solver.h"


int main (int argc, char** argv) {
    
  
    cout << "Multi Iterative Closest Points (IPC)." << endl << endl;
    
    std::string dataset_filename = "/home/leonardo/multi_icp/dataset/dataset_test.txt";

    Dataset dataset(dataset_filename);

    IntPairVector pose_point_correspondences;
    Vector2fVector points;
    Vector3fVector poses;
    Vector2fVector normals;


    
    cout << "Loading data.." << endl;

    dataset.load_data(poses, points, pose_point_correspondences, 0, 2);

    // for (auto pose : poses) cout << "[" << pose.x() << " " << pose.y() << " " << pose.z() << "]" << endl;
    // for (auto pose_point : pose_point_correspondences){
    //     cout << pose_point.first << " " << pose_point.second << endl;
    // }
    
    cout << poses.size() << " poses have been loaded." << endl;
    cout << points.size() << " points have been loaded." << endl;
    
    cout << "Loading data complete." << endl << endl;

    FloatVector points_normal(points.size());
    IntVector points_normal_is_valid(points.size());

    cout << "Loading correspondence finder." << endl;

    kdt::CorrespondenceFinder finder;
    finder.init(points, pose_point_correspondences);

    cout << "Loading correspondence finder complete." << endl << endl;

    finder.compute_other_pose_correspondences(50);

    finder.compute_same_pose_correspondences();

    vector<IntVector> same_pose_correspondences = finder.same_pose_correspondences();
    
    cout << "Estimate points normals.." << endl;
    FloatVector points_normals;
    
    for (size_t i = 0; i < same_pose_correspondences.size(); i++)
    {   
        // cout << same_pose_correspondences[i].size() << " points will be used to estimater the point " << i << " normal" << endl;
        // given a bunch of points compute the line straight line fitting the points and its normal vector
        float point_normal_angle; 
        if(estimate_normal_of_line_fitting_points(points, same_pose_correspondences[i], point_normal_angle)) points_normal[i] = point_normal_angle;
        // cout << point_normal_angle << endl;
        Vector2f normal;
        normal << cos(point_normal_angle), sin(point_normal_angle);
        normals.push_back(normal);

    }
    
    
    cout << "Estimate points normals complete." << endl << endl;

    // cout << "Loading solver .." << endl;
    
    // ICPSolver solver;    
    
    // Eigen::Isometry2f state = Eigen::Isometry2f::Identity();
    // solver.init(state, poses, points, normals);

    // size_t iterations = 10;
    // for (size_t i = 0; i < iterations; i++)
    // {
    //     solver.oneRound(correspondences, false);

    //     std::cout << (*solver.state()).rotation() << std::endl;
    //     std::cout << (*solver.state()).translation() << std::endl;
        

    //     cerr << "iteration: " << i << endl;
    //     cerr << "inliers: " << solver.numInliers() <<  endl;
    //     cerr << "outliers: " << solver.numOutliers() <<  endl;
    //     cerr << "error (inliers): " << solver.chiInliers() << " (outliers): " << solver.chiOutliers() << endl;


        
    // }
    

            


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
        
        drawer.drawPoints(points, drawer_controller, black);
        
        // drawer.drawCorrespondences(points, finder.other_pose_correspondences(), drawer_controller, green);
        // drawer.drawNormals(points, points_normal, points_normal_is_valid, drawer_controller, pink);
        
        drawer.show();

        key=cv::waitKey(0);

        drawer_controller.update(key);

    }


  return 0;
}