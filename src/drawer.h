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
        Vector2d _mean_data;

        
    public:
        Drawer(int width, int height, string window_name);
        
        int width();
        int height();
        string window_name();
        Mat image();
        Eigen::Vector2d mean_data();

        
        void init(Vector3dVector& poses, Vector2dVector& points);

        void clear();
        void show();

        void drawLine(Eigen::Vector2d& start_point, Eigen::Vector2d& end_point, DrawerController& controller, Scalar& color);
        
        void drawPoint(Eigen::Vector2d& point, DrawerController& controller, Scalar& color);
        
        void drawPose(Eigen::Vector3d& pose, DrawerController& controller, Scalar& color);

        void plotFillinMatrix(Eigen::MatrixXi fillin, Scalar& color);

        
        bool isOutsideImage(Eigen::Vector2f& point);
        bool isOutsideImage(float x, float y);
        

        
};



