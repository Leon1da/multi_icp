#include "defs.h"

class Map
{
    private:
        vector<Pose2D> _poses;
        vector<vector<Point2D>> _points;

    public:
        Map();
        ~Map();

        void add_pose(Pose2D pose);
        void add_points(vector<Point2D> points);

        vector<Pose2D> poses();
        vector<vector<Point2D>> points();


};

Map::Map(){
};

Map::~Map(){
};

void Map::add_pose(Pose2D pose){
    _poses.push_back(pose);
}

void Map::add_points(vector<Point2D> points){
    _points.push_back(points);
}


vector<Pose2D> Map::poses(){
    return _poses;
}

vector<vector<Point2D>> Map::points(){
    return _points;
}



