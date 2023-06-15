#include "defs.h"

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

        void drawLine(Eigen::Vector2f start_point, Eigen::Vector2f end_point, DrawerController controller, Scalar color);
        void drawLines(Vector2fVector start_points, Vector2fVector end_points , DrawerController controller, Scalar color);
        
        void drawPoint(Eigen::Vector2f point, DrawerController controller, Scalar color);
        void drawPoints(Vector2fVector points, DrawerController controller, Scalar color);
        
        void drawPose(Eigen::Vector3f pose, DrawerController controller, Scalar color);
        void drawPoses(Vector3fVector poses, DrawerController controller, Scalar color);

        void drawCorrespondences(Vector2fVector points, IntPairVector correspondences, DrawerController controller, Scalar color);
        
        void drawNormal(Eigen::Vector2f point, float angle, DrawerController controller, Scalar color);
        void drawNormals(Vector2fVector &points, FloatVector &angles, DrawerController controller, Scalar color);
        
        bool isOutsideImage(Eigen::Vector2f point);
        bool isOutsideImage(float x, float y);
        

        
};



