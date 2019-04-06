#include <vector>
#include <iostream>
#include <opencv2/core.hpp>
using namespace cv;
using namespace std;
#include "intersection.hpp"
Point3f from_camera_to_world(Point3f coords, Camera* camera)
{
    Vec4f temp = camera->matrix.inv()*Vec4f(coords.x, coords.y, coords.z, 1);
    return Point3f(temp[0], temp[1], temp[2]);
}

Point3f from_world_to_camera(Point3f coords, Camera* camera)
{
    Vec4f temp = camera->matrix*Vec4f(coords.x, coords.y, coords.z, 1);
    return Point3f(temp[0], temp[1], temp[2]);
}

Point3f from_pixel_to_camera_coords(Point2f pixel_coords, Camera* camera)
{
    return Point3f(pixel_coords.x-camera->w/2, pixel_coords.y-camera->h/2, camera->f);
}

Point2f from_camera_coords_to_pixel(Point3d camera_coords, Camera* camera)
{
    return Point2f(camera_coords.x*camera->f/camera_coords.z + camera->w/2, camera_coords.y*camera->f/camera_coords.z + camera->h/2);
}


Point2f intersection_line_and_line(Point2f a, Point2f b,  Point2f c, Point2f d)
{ 
    // Line AB represented as a1x + b1y = c1 
    double a1 = b.y - a.y; 
    double b1 = a.x - b.x; 
    double c1 = a1*(a.x) + b1*(a.y); 
  
    // Line CD represented as a2x + b2y = c2 
    double a2 = d.y - c.y; 
    double b2 = c.x - d.x; 
    double c2 = a2*(c.x)+ b2*(c.y); 
  
    double det = a1*b2 - a2*b1; 
  
    if (det == 0) 
        return Point2f(-1, -1);
    else 
        return Point2f((b2*c1 - b1*c2)/det, (a1*c2 - a2*c1)/det);
}

pair<Point2f, Point2f> intersection_line_and_rectangle(Point2f a, Point2f b, int width, int height)
{
    /*
    ------point_up------
    .                   .  
    .               .b  .
    .                   .
point_left         point_right  h
    .                   .
    .   .a              .
    .                   .
    ------point_down-----
              w
    */
    Point2f point_left = intersection_line_and_line(a, b, Point2f(0, 0), Point2f(0, height));
    Point2f point_right = intersection_line_and_line(a, b, Point2f(width, 0), Point2f(width, height));
    Point2f point_up = intersection_line_and_line(a, b, Point2f(0, 0), Point2f(width, 0));
    Point2f point_down = intersection_line_and_line(a, b, Point2f(0, height), Point2f(width, height));

    if (a.x == b.x)
        return make_pair(point_left, point_right);
    if (a.y == b.y)
        return make_pair(point_up, point_down);
    pair<Point2f, Point2f> temp;
    if((a.x-b.x)/(a.y-b.y) < 0)
    {
        if(point_down.x <= point_left.x)
            temp.first = point_left;
        else
            temp.first = point_down;
        if(point_up.x <= point_right.x)
            temp.second = point_up;
        else
            temp.second = point_right;
    }
    else
    {
        if(point_up.x <= point_left.x)
            temp.first = point_left;
        else
            temp.first = point_up;
        if(point_down.x <= point_right.x)
            temp.second = point_down;
        else
            temp.second = point_right;  
    }
    return temp;
}

double my_solve(Point3f a, Point3f b, Point3f c, Point3f d, Point3f e)
{
    return -determinant(Matx44f(1, 1, 1, 1, a.x, b.x, c.x, d.x, a.y, b.y, c.y, d.y, a.z, b.z, c.z, d.z))
    /determinant(Matx44f(1, 1, 1, 0, a.x, b.x, c.x, e.x-d.x, a.y, b.y, c.y, e.y-d.y, a.z, b.z, c.z, e.z-d.z));
}
//Matx44f(1, a, 1, b, 1, c, 1, d).t()

Point3f intersection_line_and_plane(Point3f wap, Point3f wz1, Point3f wbp1, Point3f wbp2, Point3f wz2)
{
    double solve = my_solve(wz2, wbp1, wbp2, wap, wz1);
    return (wz1-wap)*solve + wap;
}

bool true_point(Point3f a, Point3f b, Point3f c, Point3f d)
{
    Point3f ac = c - a;
    Point3f ab = b - a;
    Point3f ad = d - a;
    double coef1 = (ab.x*ad.y - ab.y*ad.x)/(ab.x*ac.y - ab.y*ac.x);
    double coef2 = (ad.x*ac.y - ad.y*ac.x)/(ab.x*ac.y - ab.y*ac.x);
    if (coef1 > 0 && coef2 > 0)
        return true;
    return false;
}

pair<int, int> min_edge_intersection(vector<Point3f> &a_line, Point3f &world_zero1, vector<Point3f> &b_line, Point3f &world_zero2)
{
    for(int b_index = 0; b_index != b_line.size()-1; ++b_index)
    {
        for(int a_index = 0; a_index != a_line.size(); ++a_index)
        {
            Point3f intersec_point = intersection_line_and_plane(a_line[a_index], world_zero1, b_line[b_index], b_line[b_index+1], world_zero2);

            if(true_point(world_zero2, b_line[b_index], b_line[b_index+1], intersec_point))
                return make_pair(a_index, b_index);
        }
    }
    throw 0;
}

bool line_check(Point3f p1, Point3f p2, Point3f p3)
{
    if(p1.x== p2.x)
    {
        if(abs(p3.x - p1.x) < 0.001)
            return 1;
    }
    else
    {
        double a,b;
        a = (p1.y - p2.y)/(p1.x - p2.x);
        b = p1.y - a*p1.x;
        if(abs(a*p3.x + b - p3.y) < 0.001)
            return 1;
    }
    return 0;
}

float evklid(Point2f &p1, Point2f &p2)
{
    return sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y));
}
float distance_point_to_line(Point2f &p, pair<float,float> coef)
{
    float k = coef.first;
    float b = coef.second;
    return float(abs(p.y - k*p.x - b))/sqrt(k*k + 1);
}
float distance_segment_to_segment(pair<Point2f, Point2f> &pp1, pair<Point2f, Point2f> &pp2)
{
    return min(min(evklid(pp1.first, pp2.first), evklid(pp1.first, pp2.second)), min(evklid(pp1.second, pp2.first), evklid(pp1.second, pp2.second)));
}
