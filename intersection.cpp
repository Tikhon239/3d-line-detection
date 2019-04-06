#include <vector>
#include <iostream>
#include <opencv2/core.hpp>
using namespace cv;
using namespace std;
#include "intersection.hpp"
int main()
{
    float f1 = 2958.4;
    int w1 = 2048;
    int h1 = 1536;
    Matx44f transform_matrix1 = Matx44f(-0.9748872041038975, 0.04671274505809842, -0.21774493960508734,2.970590949290061,
        0.005414574973176462, 0.9824363204795558, 0.18651959302028226, -2.660624227023156,
        0.2226333794657618, 0.1806565682496183, -0.9580175273427545, 23.89273819782546,
        0.0, 0.0, 0.0, 1.0);
    Point2f pa(1283, 786);
    Point3f p_check1(-5, -5, -5);
    Point3f p_check2(5, 5, 5);
    Camera camera1 = Camera(f1, w1, h1, transform_matrix1);
    Point3f world_zero1 = from_camera_to_world(Point3f(0, 0, 0), &camera1);

    float f2 = 2958.4;
    int w2 = 2048;
    int h2 = 1536;
    Matx44f transform_matrix2 = Matx44f(-0.9487882441280389, 0.026790311173834142, 0.3147747560266583, -6.991027332256034,
        0.09595272462274344, 0.9737637718173283, 0.20634241283256097, -2.872366219228082,
        -0.3009882762532538, 0.22597875104376564, -0.9264662226086793, 22.09784788051675,
        0.0, 0.0, 0.0, 1.0);
    Point2f pb1(700, 700);
    Point2f pb2(700, 850);
    Camera camera2 = Camera(f2, w2, h2, transform_matrix2);
    Point3f world_zero2 = from_camera_to_world(Point3f(0, 0, 0), &camera2);

    Point2f pa_check1 = from_camera_coords_to_pixel(from_world_to_camera(p_check1, &camera1), &camera1);
    Point2f pa_check2 = from_camera_coords_to_pixel(from_world_to_camera(p_check2, &camera1), &camera1);
    
    pair<Point2f,Point2f> temp = intersection_line_and_rectangle(pa_check1, pa_check2, camera1.w, camera1.h);
    pa_check1 = temp.first;
    pa_check2 = temp.second;

    Point2f pb_check1 = from_camera_coords_to_pixel(from_world_to_camera(p_check1, &camera2), &camera2);
    Point2f pb_check2 = from_camera_coords_to_pixel(from_world_to_camera(p_check2, &camera2), &camera2);

    temp = intersection_line_and_rectangle(pb_check1, pb_check2, camera2.w, camera2.h);
    pb_check1 = temp.first;
    pb_check2 = temp.second;

    vector<Point3f> a_line;
    int a_lenght = 19;
    Point2f a_step = (pa_check2-pa_check1)/a_lenght;
    for(int i = 0; i <= a_lenght; ++i)
    {
        a_line.push_back(from_camera_to_world(from_pixel_to_camera_coords(pa_check1 + i*a_step, &camera1), &camera1));
        cout << a_line[i] << endl;
    }
    cout << endl;
    vector<Point3f> b_line;
    int b_lenght = 19;
    Point2f b_step = (pb_check2-pb_check1)/b_lenght;
    for(int i = 0; i <= b_lenght; ++i)
    {
        b_line.push_back(from_camera_to_world(from_pixel_to_camera_coords(pb_check1 + i*b_step, &camera2), &camera2));
        cout << b_line[i] << endl;
    }

    int a_point_index, b_point_index; 
    try
    {
       pair<int, int> temp = min_edge_intersection(a_line, world_zero1, b_line, world_zero2);
       a_point_index = temp.first;
       b_point_index = temp.second;
    }
    catch(int e)
    {
        cout << "Пересечений не найдено" << endl;
        return 0;
    }

    vector<Point3f> res_line;

    while(a_point_index != a_line.size() && b_point_index != b_line.size()-1)
    {
        Point3f intersec_point = intersection_line_and_plane(a_line[a_point_index], world_zero1, b_line[b_point_index], b_line[b_point_index+1], world_zero2);
        while(a_point_index != a_line.size() && true_point(world_zero2, b_line[b_point_index], b_line[b_point_index+1], intersec_point))
        {
            res_line.push_back(intersec_point); 
            ++a_point_index;
            if (a_point_index != a_line.size())
                intersec_point = intersection_line_and_plane(a_line[a_point_index], world_zero1, b_line[b_point_index], b_line[b_point_index+1], world_zero2);
        }

        ++b_point_index;
        if (b_point_index != b_line.size()-1)
            intersec_point = intersection_line_and_plane(a_line[a_point_index], world_zero1, b_line[b_point_index], b_line[b_point_index+1], world_zero2);

        while(b_point_index != b_line.size()-1 && !true_point(world_zero2, b_line[b_point_index], b_line[b_point_index+1], intersec_point))
        {
            ++b_point_index;
            if (b_point_index != b_line.size()-1)
                intersec_point = intersection_line_and_plane(a_line[a_point_index], world_zero1, b_line[b_point_index], b_line[b_point_index+1], world_zero2);
        } 
    }

    cout << world_zero1 << " " <<  world_zero2 << endl;
    cout << p_check1 << " " << p_check2 << endl;
    for(auto point = res_line.begin(); point != res_line.end(); ++point)
        cout << *point << " " << line_check(p_check1, p_check2, *point) << endl;
    return 0;
}
