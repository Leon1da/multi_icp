#include "defs.h"

class DrawerController{
    private:

        float _u, _v, _scale;

        float _du;
        float _dv;
        float _ds;
    public:

        DrawerController(float u, float v, float s,  float du, float dv, float ds);
        DrawerController(float du, float dv, float ds);
        ~DrawerController();

        float u();
        float v();
        float scale();

        void update(int key);  
        void update_u(int sign);  
        void update_v(int sign);  
        void update_scale(int sign);  


};

float DrawerController::u(){
    return _u;
}

float DrawerController::v(){
    return _v;
}

float DrawerController::scale(){
    return _scale;
}

DrawerController::DrawerController(float u, float v, float s,  float du, float dv, float ds){
    cout << "Drawer Controller" << endl;
    _u = u;
    _v = v;
    _scale = s;

    _du = du;
    _dv = dv;
    _ds = ds;
}
        
DrawerController::DrawerController(float du, float dv, float ds){
    cout << "Drawer Controller" << endl;
    _u = 0.0;
    _v = 0.0;
    _scale = 1.0;

    _du = du;
    _dv = dv;
    _ds = ds;
}

DrawerController::~DrawerController(){

}

void DrawerController::update(int key){

    switch(key) {
        case 'w': 
            update_v(-1);
            break;
        case 's': 
            update_v(+1);
            break;
        case 'a': 
            update_u(-1);
            break;
        case 'd': 
            update_u(+1);
            break;

        case 'q': 
            update_scale(1);
            break;
        
        case 'e': 
            update_scale(-1);
            break;

        default: ;
    }

    // cout << "\r[u: " << _u << ", v: " << _v << ", scale: " << _scale << "] " << flush;
}

void DrawerController::update_u(int sign){
    _u = _u + sign * _du;
}

void DrawerController::update_v(int sign){
    _v = _v + sign * _dv;   
}

void DrawerController::update_scale(int sign){
    _scale = _scale + sign * _ds;
    if (_scale < 1){
        _scale = 1;
    }
}


class Drawer
{
    private:
        int _width, _height;
        string _window_name;
        Mat _image; 
        Vector2d _data_mean;


    public:
        Drawer(int width, int height, string window_name);
        ~Drawer();

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

Drawer::Drawer(int width, int height, string window_name)
{
    _width = width;
    _height = height;
    _window_name = window_name;

    _image = Mat(width, height, CV_8UC3, Scalar(255, 255, 255));
    // image = _image;
  
    // Check if the image is created
    // successfully
    if (!_image.data) {
        cerr << "Could not open or find the image" << endl;
    }

    cout << "Image: " << _image.rows << " " << _image.cols << endl;
    
}

Drawer::~Drawer()
{
}

int Drawer::height(){
    return _height;
}

int Drawer::width(){
    return _width;
}

string Drawer::window_name(){
    return _window_name;
}

Mat Drawer::image(){
    return _image;
}


Vector2d Drawer::mean_data(){
    return _data_mean;
}

void Drawer::init(Vector3dVector& poses, Vector2dVector& points){
    Vector2d mean;

    size_t num_poses = poses.size();
    for (size_t i = 0; i < num_poses; i++) mean += poses[i].block(0, 0, 2, 1);
    
    size_t num_points = points.size();
    for (size_t i = 0; i < num_points; i++) mean += points[i];
    
    mean = mean / (num_poses + num_points);
    
}

void Drawer::clear(){

    _image = Mat(width(), height(), CV_8UC3, Scalar(255, 255, 255));
    
}

void Drawer::show(){
    
    // cout << "Drawer show." << endl;
    
    imshow(window_name(), image());
    
}

bool Drawer::isOutsideImage(Vector2f& point){
    return point.x() < 0 || point.x() >= height() || point.y() < 0 || point.y() >= width(); 
}

bool Drawer::isOutsideImage(float x, float y){
    return x < 0 || x >= height() || y < 0 || y >= width(); 
}

void Drawer::drawLine(Eigen::Vector2d& start_point, Eigen::Vector2d& end_point, DrawerController& controller, Scalar& color){

    double ri, ci, rf, cf;
    
    ci = controller.scale() * (start_point.x() - _data_mean.x()) + controller.u();
    ri = controller.scale() * (start_point.y() - _data_mean.y()) + controller.v();

    cf = controller.scale() * (end_point.x() - _data_mean.x()) + controller.u();
    rf = controller.scale() * (end_point.y() - _data_mean.y()) + controller.v();

    Point pi = Point(ci, ri);
    Point pf = Point(cf, rf);
    
    cv::line(image(), pi, pf, color);
}

void Drawer::drawPoint(Eigen::Vector2d& point, DrawerController& controller, Scalar& color){
    
    double r, c;
    // int radius = 1 * controller.scale();
    int radius = 1;

    c = controller.scale() * (point.x() - _data_mean.x()) + controller.u();
    r = controller.scale() * (point.y() - _data_mean.y()) + controller.v();
    
    if (isOutsideImage(c, r)) return; // point outside the image

    cv::circle(image(), cv::Point(c,r), radius, color);
}

void Drawer::drawPose(Eigen::Vector3d& pose, DrawerController& controller, Scalar& color){
    double ri, ci, rf, cf;
    
    double theta = pose.z();
    ci = controller.scale() * (pose.x() - _data_mean.x()) + controller.u();
    ri = controller.scale() * (pose.y() - _data_mean.y()) + controller.v();
    
    // int radius = 3 * controller.scale();
    int radius = 3;
    cf = ci + radius * cos(theta);
    rf = ri + radius * sin(theta);

    if (isOutsideImage(ci, ri)) return;

    cv::line(image(), cv::Point(ci,ri), cv::Point(cf,rf), color);
    cv::circle(image(), cv::Point(ci,ri), radius, color);


}

void Drawer::plotFillinMatrix(Eigen::MatrixXi fillin, Scalar& color){
    size_t total_blocks = 0, filled_blocks = 0;
    float scalex = float (width()) / fillin.rows();
    float scaley = float (height()) / fillin.cols();
    
    for (size_t i = 0; i < fillin.rows(); i++)      
    {
        for (size_t j = 0; j < fillin.cols(); j++)
        {
            int value = fillin(i, j);
            if (!value) continue;            
            int c = (int) 255 - (255 * value / 1080);
            cv::rectangle(image(), Point(i*scalex, j*scaley), Point(i*scalex + scalex, j*scaley + scaley), Scalar(c, c, c), FILLED);
            filled_blocks+=1;
        }
        
    }


    cout << "[filled blocks / total blocks] " << filled_blocks << " / " << fillin.rows() * fillin.cols() << endl;
    
}

