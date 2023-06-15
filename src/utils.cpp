#include "defs.h"

// line fitting point cloud
bool estimate_line_fitting_points(Vector2fVector &points, IntVector &ids, float &angle){

    int n_points = ids.size();
    Eigen::Vector2f mean(0, 0);
    for(auto id : ids) mean = mean + points[id];
    mean = mean / n_points;
    
    float num = 0, den = 0, m = 0;
    for(auto id : ids) 
    {
        num = num + (points[id].x() - mean.x()) * (points[id].y() - mean.y());
        den = den + pow(points[id].x() - mean.x(), 2);
    }
    
    if(!den) angle = 90 * M_PI / 180; // 90 degrees
    else if(!num) angle = 0; // 0 degrees
    else{
        m = num/den;
        angle = atan(m);
    }
    
    return true;

}

// normal of line fitting point cloud
bool estimate_normal_of_line_fitting_points(Vector2fVector &points, IntVector ids, float &angle){

    int n_points = ids.size();
    Eigen::Vector2f mean(0, 0);
    for(auto id : ids) mean = mean + points[id];
    mean = mean / n_points;
    
    float num = 0, den = 0, m = 0;
    for(auto id : ids) 
    {
        num = num + (points[id].x() - mean.x()) * (points[id].y() - mean.y());
        den = den + pow(points[id].x() - mean.x(), 2);
    }
    
    
    if(!den) angle = 90 * M_PI / 180; // 90 degrees
    else if(!num) angle = 0; // 0 degrees
    else{
        
        m = num / den; // slope of line
        m = -1/m; // slope of normal
        angle = atan(m);
    }
    return true;

}

