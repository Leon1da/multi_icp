#include "defs.h"
#include "map.h"

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

    cout << "[u, v, scale] " << _u << " " << _v << " " << _scale << endl;
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

        void drawPoint(Point2D point, DrawerController controller);
        void drawPose(Pose2D pose, DrawerController controller);

        void drawMap(Map map,  DrawerController controller);
        void drawMapPoints(vector<Point2D> map_points,  DrawerController controller);
        void drawMapPoses(vector<Pose2D> map_poses,  DrawerController controller);
        void drawMapCorrespondences(vector<Point2D> points, vector<vector<int>> correspondences, DrawerController controller);

        bool isOutsideImage(Point2D point);
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
    
    cout << "Drawer show." << endl;
    
    imshow(window_name(), image());
    
}

bool Drawer::isOutsideImage(Point2D point){
    return point.x() < 0 || point.x() >= height() || point.y() < 0 || point.y() >= width(); 
}

bool Drawer::isOutsideImage(float x, float y){
    return x < 0 || x >= height() || y < 0 || y >= width(); 
}

void Drawer::drawMap(Map map,  DrawerController controller){
    
    drawMapPoses(map.poses(), controller);
    
    for ( auto points : map.points()) drawMapPoints(points, controller);
     
}
        
void Drawer::drawMapPoints(vector<Point2D> points, DrawerController controller){
    for (auto point : points) drawPoint(point, controller);    
}

void Drawer::drawMapPoses(vector<Pose2D> poses,  DrawerController controller){
    for(auto pose : poses) drawPose(pose, controller);
}

void Drawer::drawMapCorrespondences(vector<Point2D> points, vector<vector<int>> correspondences, DrawerController controller){
    Scalar color(0, 255, 0);

    Point2D src_point, dst_point;
    // int src_id, dst_id;

    float ci, ri, cf, rf;
    for (auto corr : correspondences){
        
        src_point = points.at(corr.at(2));
        dst_point = points.at(corr.at(3));

        // cout << src_point << " " << dst_point << endl;
        
        ci = controller.scale() * src_point.x() + controller.u();
        ri = controller.scale() * src_point.y() + controller.v();
    
        cf = controller.scale() * dst_point.x() + controller.u();
        rf = controller.scale() * dst_point.y() + controller.v();
        
        cv::line(image(), cv::Point(ci,ri), cv::Point(cf,rf), color);
        
        
    }
}


void Drawer::drawPoint(Point2D point, DrawerController controller){
    
    float r, c;
    int radius = 1;
    Scalar color(255, 0, 0);

    c = controller.scale() * point.x() + controller.u();
    r = controller.scale() * point.y() + controller.v();
    
    if (isOutsideImage(c, r)) return; // point outside the image

    cv::circle(image(), cv::Point(c,r), radius, color);
    
}

void Drawer::drawPose(Pose2D pose, DrawerController controller){
    float ri, ci, rf, cf;
    int radius = 5;
    Scalar color(0, 0, 255);

    
    Eigen::Vector2f position(pose.x(), pose.y());
    float theta = pose.theta();
    ci = controller.scale() * position.x() + controller.u();
    ri = controller.scale() * position.y() + controller.v();
    
    
    cf = ci + radius * cos(theta);
    rf = ri + radius * sin(theta);

    if (isOutsideImage(ci, ri)) return;

    cv::line(image(), cv::Point(ci,ri), cv::Point(cf,rf), color);
    cv::circle(image(), cv::Point(ci,ri), radius, color);

    
}

