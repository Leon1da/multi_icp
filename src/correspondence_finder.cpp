#include "defs.h"
#include "kdtree.h"

class CorrespondenceFinder
{
    private:

        vector<Point2D> _points;
        vector<int> _pose_id;

        kdt::KDTree<Point2D> _kdtree;
        
    public:
        CorrespondenceFinder();
        ~CorrespondenceFinder();

        vector<Pose2D> poses();
        vector<Point2D> points();
        kdt::KDTree<Point2D> kdtree();
        

        

};

CorrespondenceFinder::CorrespondenceFinder(){

}



CorrespondenceFinder::~CorrespondenceFinder(){
}

