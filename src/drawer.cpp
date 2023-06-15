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

    cout << "\r[u: " << _u << ", v: " << _v << ", scale: " << _scale << "] " << flush;
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

    public:
        Drawer(int width, int height, string window_name);
        ~Drawer();

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

void Drawer::clear(){

    _image = Mat(width(), height(), CV_8UC3, Scalar(255, 255, 255));
    
}

void Drawer::show(){
    
    // cout << "Drawer show." << endl;
    
    imshow(window_name(), image());
    
}

bool Drawer::isOutsideImage(Vector2f point){
    return point.x() < 0 || point.x() >= height() || point.y() < 0 || point.y() >= width(); 
}

bool Drawer::isOutsideImage(float x, float y){
    return x < 0 || x >= height() || y < 0 || y >= width(); 
}

void Drawer::drawLine(Eigen::Vector2f start_point, Eigen::Vector2f end_point, DrawerController controller, Scalar color){

    float ri, ci, rf, cf;
    
    ci = controller.scale() * start_point.x() + controller.u();
    ri = controller.scale() * start_point.y() + controller.v();

    cf = controller.scale() * end_point.x() + controller.u();
    rf = controller.scale() * end_point.y() + controller.v();
    
    cv::line(image(), cv::Point(ci,ri), cv::Point(cf,rf), color);
}

void Drawer::drawPoint(Eigen::Vector2f point, DrawerController controller, Scalar color){
    
    float r, c;
    int radius = 1;
    
    c = controller.scale() * point.x() + controller.u();
    r = controller.scale() * point.y() + controller.v();
    
    if (isOutsideImage(c, r)) return; // point outside the image

    cv::circle(image(), cv::Point(c,r), radius, color);
}

void Drawer::drawPose(Eigen::Vector3f pose, DrawerController controller, Scalar color){
    float ri, ci, rf, cf;
    int radius = 5;
    
    
    float theta = pose.z();
    ci = controller.scale() * pose.x() + controller.u();
    ri = controller.scale() * pose.y() + controller.v();
    
    
    cf = ci + radius * cos(theta);
    rf = ri + radius * sin(theta);

    if (isOutsideImage(ci, ri)) return;

    cv::line(image(), cv::Point(ci,ri), cv::Point(cf,rf), color);
    cv::circle(image(), cv::Point(ci,ri), radius, color);

}

void Drawer::drawLines(Vector2fVector start_points, Vector2fVector end_points , DrawerController controller, Scalar color){
    for (size_t i = 0; i < start_points.size(); i++)
        drawLine(start_points[i], end_points[i], controller, color); 
    
}

void Drawer::drawPoints(Vector2fVector points, DrawerController controller, Scalar color){
    for (auto point : points) drawPoint(point, controller, color);
}

void Drawer::drawPoses(Vector3fVector poses, DrawerController controller, Scalar color){
    for (auto pose : poses) drawPose(pose, controller, color);
}

void Drawer::drawCorrespondences(Vector2fVector points, IntPairVector correspondences, DrawerController controller, Scalar color){
    for(auto correspondence : correspondences){
        drawLine(points[correspondence.first], points[correspondence.second], controller, color);   
    }
}

void Drawer::drawNormal(Eigen::Vector2f point, float angle, DrawerController controller, Scalar color){
    float offset = 0.025;
    
    float x2 = point.x() + offset*cos(angle);
    float y2 = point.y() + offset*sin(angle);
    
    Eigen::Vector2f end(x2, y2);
    drawLine(point, end, controller, color);
    
}

void Drawer::drawNormals(Vector2fVector &points, FloatVector &angles, DrawerController controller, Scalar color){
    for (size_t i = 0; i < points.size(); i++) drawNormal(points[i], angles[i], controller, color);
}
