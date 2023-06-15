#include "defs.h"

#include "dataset.h"
#include "drawer.h"

#include "utils.h"
#include "kdtree.h"
#include "multi_icp_solver.h"


int main (int argc, char** argv) {
    
  
    cout << "Multi Iterative Closest Points (IPC)." << endl << endl;
    
    std::string dataset_filename = "/home/leonardo/multi_icp/dataset/dataset_test.txt";

    Dataset dataset(dataset_filename);

    IntPairVector pose_point_correspondences;
    Vector2fVector points;
    Vector3fVector poses;
    Vector2fVector normals;


    
    cout << "Loading data.." << endl;

    dataset.load_data(poses, points, pose_point_correspondences, 0, 10);
    size_t num_poses, num_points;
    num_poses = poses.size();
    num_points = points.size();

    cout << num_poses << " poses have been loaded." << endl;
    cout << num_points << " points have been loaded." << endl;
    
    cout << "Loading data complete." << endl << endl;

    FloatVector points_normal(points.size());
    IntVector points_normal_is_valid(points.size());

    cout << "Loading correspondence finder." << endl;

    kdt::CorrespondenceFinder finder;
    finder.init(points, pose_point_correspondences);

    cout << "Loading correspondence finder complete." << endl << endl;

    finder.compute_other_pose_correspondences();
    
    IntPairVector other_pose_correspondences = finder.other_pose_correspondences();

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

    cout << "Loading solver .." << endl;
    
    MultiICPSolver solver;   
    Isometry2fVector state(num_poses);
    
    solver.init(state, poses, points, normals);

    // for (size_t i = 0; i < 30; i++)
    // {
        // solver.oneRound(other_pose_correspondences, pose_point_correspondences, false);
        // cout << "####### Multi-ICP iteration " << i << " #######" << endl; 
        // cout << "inliers: " << solver.numInliers() << " outliers: " << solver.numOutliers() << endl;
        // cout << "error (inliers): " << solver.chiInliers() << " (outliers): " << solver.chiOutliers() << endl;
        // cout << "##############################################" << endl;

        // // update points and correspondences

        // for(auto correspondence : pose_point_correspondences){
        //     int pose_id = correspondence.first;
        //     int land_id = correspondence.second;

        //     // cout << pose_id << " " << land_id << endl;
        //     // cout << points[land_id] << endl << endl;
        //     // cout << (*solver.state())[pose_id].rotation() << endl << endl;
        //     // cout << (*solver.state())[pose_id].translation() << endl << endl;
        //     // cout << (*solver.state())[pose_id] * points[land_id] << endl;
        //     points[land_id] = (*solver.state())[pose_id] * points[land_id];

        //     // cout << points[land_id] << endl;
            
        //     // break;
        // }

        // finder.init(points, pose_point_correspondences);
        // finder.compute_other_pose_correspondences();
        // other_pose_correspondences = finder.other_pose_correspondences();

        // // break;

    // }
    
    // for (auto m : state){
    //     cout << m.rotation().block<1, 2>(0, 0) << " " << m.translation().segment(0, 1) << " | ";
    // }

    // cout << endl;

    // for (auto m : state){
        
    //     cout << m.rotation().block<1, 2>(1, 0) << " " << m.translation().segment(1, 1) << " | ";
    // }

    // cout << endl;


    int width = 800;
    int height = 800;
  
    Drawer map_visualizer(width, height, "test_multi_icp_solver");
    // Drawer optimization_visualizer(width, height, "optimization");

    // DrawerController map_visualizer_controller(5.0, 5.0, 0.5);
    DrawerController map_visualizer_controller(height/4, width/4, 5.0, 5.0, 5.0, 5.0);
    
    Scalar blue(255, 0, 0);
    Scalar red(0, 0, 255);
    Scalar green(0, 255, 0);
    Scalar pink(255, 0, 255);
    Scalar black(0, 0, 0);
    Scalar white(255, 255, 255);
    

    char key=0;
    const char ESC_key=27;
    while (key!=ESC_key) {

        map_visualizer.clear();
        
        map_visualizer.drawPoses(poses, map_visualizer_controller, red);
        
        map_visualizer.drawPoints(points, map_visualizer_controller, black);
        
        map_visualizer.drawCorrespondences(points, finder.other_pose_correspondences(), map_visualizer_controller, green);
        // map_visualizer.drawNormals(points, points_normal, points_normal_is_valid, map_visualizer_controller, pink);
        
        map_visualizer.show();



        key=cv::waitKey(0);

        map_visualizer_controller.update(key);

        if (key == ' '){
            solver.oneRound(other_pose_correspondences, pose_point_correspondences, false);

            cout << "####### Multi-ICP iteration #######" << endl; 
            cout << "total points: " << other_pose_correspondences.size() << endl;
            cout << "inliers: " << solver.numInliers() << " outliers: " << solver.numOutliers() << endl;
            cout << "error (inliers): " << solver.chiInliers() << " (outliers): " << solver.chiOutliers() << endl;
            cout << "##############################################" << endl;


            for(auto correspondence : pose_point_correspondences){
                int pose_id = correspondence.first;
                int land_id = correspondence.second;

                // update point position using state corretion
                // points[land_id] = (*solver.state())[pose_id].inverse() * points[land_id];
                points[land_id] = (*solver.state())[pose_id] * points[land_id];

            }

            // re-build the kdtree
            finder.init(points, pose_point_correspondences);

            // re-compute correspondences
            finder.compute_other_pose_correspondences(5);

            // update correspondences for the nezt optimization step
            other_pose_correspondences = finder.other_pose_correspondences();

        }

    }


  return 0;
}