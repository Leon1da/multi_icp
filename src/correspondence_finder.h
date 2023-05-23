#include "defs.h"

class CorrespondenceFinder
{
    private:
        
        cv::flann::Index _flann_index;
        cv::Mat _indices, _dists;
        cv::Mat_<float> _features, _queries;

        vector<Eigen::Vector2f> _points;
        vector<int> _points_pose_index; 
        bool _inizialized;


    public:
        CorrespondenceFinder();
        CorrespondenceFinder(vector<Eigen::Vector2f> points, int points_pose_index);
        ~CorrespondenceFinder();
        
        cv::flann::Index tree();

        void inizialize();
        void add_points(vector<Eigen::Vector2f> points, int points_pose_index);
        void add_point(Eigen::Vector2f point, int point_pose_index);
        void search_correspondence(Eigen::Vector2f query_point, int query_point_pose_index);
        void search_correspondences(vector<Eigen::Vector2f> query_points, int query_points_pose_index);
        


};
