class Camera
{
public:
    Camera(float f_data, int w_data, int h_data, Matx44f matrix_data)
    {
        f = f_data;
        w = w_data;
        h = h_data;
        matrix = matrix_data;
    }

    float f;
    int w;
    int h;
    Matx44f matrix;

};

Point3f from_camera_to_world(Point3f coords, Camera* camera);
Point3f from_world_to_camera(Point3f coords, Camera* camera);
Point3f from_pixel_to_camera_coords(Point2f pixel_coords, Camera* camera);
Point2f from_camera_coords_to_pixel(Point3d camera_coords, Camera* camera);
Point2f intersection_line_and_line(Point2f a, Point2f b,  Point2f c, Point2f d);
pair<Point2f, Point2f> intersection_line_and_rectangle(Point2f a, Point2f b, int width, int height)
double my_solve(Point3f a, Point3f b, Point3f c, Point3f d, Point3f e);
Point3f intersection_line_and_plane(Point3f wap, Point3f wz1, Point3f wbp1, Point3f wbp2, Point3f wz2);
bool true_point(Point3f a, Point3f b, Point3f c, Point3f d);
pair<int, int> min_edge_intersection(vector<Point3f> &a_line, Point3f &world_zero1, vector<Point3f> &b_line, Point3f &world_zero2);
bool line_check(Point3f p1, Point3f p2, Point3f p3);
float evklid(Point2f &p1, Point2f &p2);
float distance_point_to_line(Point2f &p, pair<float,float> coef);
float distance_segment_to_segment(pair<Point2f, Point2f> &pp1, pair<Point2f, Point2f> &pp2);
