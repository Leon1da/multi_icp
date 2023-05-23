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
