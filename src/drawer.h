#include "defs.h"
// #include "map.h"

class DrawerController{
    private:

        float u, v, scale;

        float du;
        float dv;
        float ds;
    
    public:

        DrawerController(float _du, float _dv, float _ds);
        DrawerController(float _u, float _v, float _s,  float _du, float _dv, float _ds);
        ~DrawerController();

        
        float get_u();
        float get_v();
        float get_scale();

        void update(int key);  
        void update_u(int sign);  
        void update_v(int sign);  
        void update_scale(int sign);  
};

class Drawer{
    private:
        int _width;
        int _height;
        string _window_name;
        Mat _image;

        
    public:
        Drawer(int width, int height, string window_name);
        
        int width();
        int height();
        string window_name();
        Mat image();

        void clear();
        void show();
        void drawPoint(Point2D point, DrawerController controller);
        void drawPose(Pose2D pose, DrawerController controller);

        void drawMapPoints(vector<Point2D> map_points,  DrawerController controller);
        void drawMapPoses(vector<Pose2D> map_poses,  DrawerController controller);
        void drawMap(Map map,  DrawerController controller);
        void drawMapCorrespondences(vector<Point2D> points, vector<vector<int>> correspondences, DrawerController controller);

        bool isOutsideImage(Point2D point);
        bool isOutsideImage(float x, float y);

        
};



