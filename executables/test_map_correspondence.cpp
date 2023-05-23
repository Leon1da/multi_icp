#include "defs.h"

#include "dataset.h"
#include "map.h"
#include "drawer.h"
#include "correspondence_finder.h"

#include "kdtree.h"


int main (int argc, char** argv) {
    
  
    std::cout << "Multi Iterative Closest Points (IPC)." << std::endl;
    
    std::string dataset_filename = "/home/leonardo/multi_icp/dataset/dataset_test.txt";

    Dataset dataset(dataset_filename);

    Map map;

   
    vector<Point2D> kdtree_points;
    vector<int> kdtree_poses(0);

    int pose_index = 0;
    for(auto record : dataset.records()){
        
        Pose2D pose = record_to_pose(record);   
        vector<Point2D> points = record_to_points(record);   
        map.add_pose(pose);
        map.add_points(points);

        // Fill kdtree
        int npoints = points.size();
        for (int i = 0; i < npoints; i++){

            kdtree_points.push_back(Point2D(points[i]));
            kdtree_poses.push_back(pose_index);
        }

        pose_index = pose_index + 1;
        if (pose_index > 1) break;
    }

    cout << "kdtree_points " << kdtree_points.size() << endl;

    // Build kdtree
    kdt::KDTree<Point2D> kdtree(kdtree_points);
    vector<vector<int>> correspondences;

    for (size_t i = 0; i < kdtree_points.size(); i++){
        Point2D query = kdtree_points[i];
        int query_pose_id = kdtree_poses[i];
        const std::vector<int> idxs = kdtree.knnSearch(query, 50);
        for (auto id : idxs){
            Point2D point_found = kdtree_points[id];
            int point_found_pose_id = kdtree_poses[id];
            if (point_found_pose_id == query_pose_id) continue; // check next point 
            
            int src_pose, dst_pose, src_point, dst_point;
            src_pose = query_pose_id;
            dst_pose = point_found_pose_id;
            src_point = i;
            dst_point = id;

            vector<int> corr;
            corr.push_back(src_pose);
            corr.push_back(dst_pose);
            corr.push_back(src_point);
            corr.push_back(dst_point);
            correspondences.push_back(corr);

            float distance = sqrt(pow(query.x() - point_found.x(), 2) + pow(query.y() - point_found.y(), 2));
            // cout << "distance: " << distance << " [ " << src_point << " " << dst_point << " " << src_pose  << " " << dst_pose << " ] " << endl;
            // cout << "-----------------------" << endl;   
            break;
        }
    }
    cout << "correspondences :" << correspondences.size() << endl;
    // return 0;
    
    // CorrespondenceTable table(3, 4, dst_indices);
    // map.add_correspondences(table);
    

    int width = 800;
    int height = 800;
  
    Drawer drawer(width, height, "test_map_correspondence");

    // DrawerController drawer_controller(5.0, 5.0, 0.5);
    DrawerController drawer_controller(height/4, width/4, 5.0, 5.0, 5.0, 5.0);
  

    char key=0;
    const char ESC_key=27;
    while (key!=ESC_key) {

        drawer.clear();
        
        drawer.drawMap(map, drawer_controller);

        drawer.drawMapCorrespondences(kdtree_points, correspondences, drawer_controller);
        // drawer.drawMapCorrespondences(map, drawer_controller);

        drawer.show();

        key=cv::waitKey(0);

        drawer_controller.update(key);

    }


  return 0;
}