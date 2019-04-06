#include <iostream>
#include <vector>
#include <queue>
#include "opencv2/core/core.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui/highgui.hpp"
using namespace std;
using namespace cv;

#define INF 1000000
using seg = vector<Point2f>;
using l = vector<seg>;

bool comp(pair< pair<Point2f,Point2f>, pair<float,float> > a, pair< pair<Point2f,Point2f>, pair<float,float> > b)
{
    return a.first.first.x < b.first.first.x;
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

float distance_point_to_line(Point2f &p, pair<float,float> coef)
{
    float k = coef.first;
    float b = coef.second;
    return float(abs(p.y - k*p.x - b))/sqrt(k*k + 1);
}

pair <float, float> centroids(l &line)
{
    float x_centroid = 0, y_centroid = 0;
    int lenght = 0;
    for(int segment = 0; segment < line.size(); ++ segment)
        for(int point = 0; point < line[segment].size(); ++point)
        {
            ++lenght;
            x_centroid += line[segment][point].x;
            y_centroid += line[segment][point].y;
        }
        return make_pair(float(x_centroid)/lenght, float(y_centroid)/lenght);
}

vector<float> statistics(l &line)
{
    vector<float> stat(5);
    pair <float, float> temp = centroids(line);
    stat[0] = temp.first;
    stat[1] = temp.second;
    int lenght = 0;
    for(int segment = 0; segment < line.size(); ++ segment)
        for(int point = 0; point < line[segment].size(); ++point)
        {
            ++lenght;
            stat[2] += (line[segment][point].x - stat[0])*(line[segment][point].x - stat[0]);
            stat[3] += (line[segment][point].y - stat[1])*(line[segment][point].y - stat[1]);
            stat[4] += (line[segment][point].x - stat[0])*(line[segment][point].y - stat[1]);
        }
        stat[2] = float(stat[2])/lenght;
        stat[3] = float(stat[3])/lenght;
        stat[4] = float(stat[4])/lenght;
    return stat;
}

pair <float, float> line_model_fit(l &line)
{
    vector<float> stat;
    stat = statistics(line);
    float k, b;
    k = stat[4]/stat[2];
    b = stat[1] - k*stat[0];
    return(make_pair(k, b));

}

void line_model_fit_and_image(l &line, Mat &image)
{
    vector<float> stat;
    stat = statistics(line);
    float k, b;
    k = stat[4]/stat[2];
    b = stat[1] - k*stat[0];
    Point2f p1 = Point(b, 0);
    Point2f p2 = Point(k*image.rows + b, image.rows);
    pair<Point2f, Point2f> temp = intersection_line_and_rectangle(p1, p2, image.cols, image.rows);
    p1 = temp.first;
    p2 = temp.second;
    cv::line(image, p1, p2, Scalar(0, 0, 255), 1);
}

pair<Point2f,Point2f> first_end(seg &segment)
{
    Point2f first = Point2f(INF, INF);
    Point2f end = Point2f(0, 0);
    for(int point = 0; point < segment.size(); ++point)
    {
        if(segment[point].x < first.x)
            first = segment[point];
        if(segment[point].x > end.x)
            end = segment[point];
    }

    return make_pair(first, end);

}

void segment_image(l &line, Mat &image)
{
    vector<pair< pair<Point2f,Point2f>, pair<float,float> > > supporting_points;
    for(int segment = 0; segment < line.size(); ++ segment)
    {
        l kostl;
        kostl.push_back(line[segment]);
        pair<float, float> coef = line_model_fit(kostl);
        pair<Point2f,Point2f> temp = first_end(line[segment]);
        Point2f p1 = Point2f(coef.first*temp.first.x + coef.second, temp.first.x);
        Point2f p2 = Point(coef.first*temp.second.x + coef.second, temp.second.x);
        supporting_points.push_back(make_pair(make_pair(p1, p2), coef));
    }

    sort(supporting_points.begin(), supporting_points.end(), comp);
    for(int point = 0; point < supporting_points.size()-1; ++point)
    {
        cv::line(image, supporting_points[point].first.first, supporting_points[point].first.second, Scalar(0, 0, 255), 1);
        cv::line(image, supporting_points[point].first.second, supporting_points[point+1].first.second, Scalar(0, 0, 255), 1);
    }
     cv::line(image, supporting_points[supporting_points.size()-1].first.first, supporting_points[supporting_points.size()-1].first.second, Scalar(0, 0, 255), 1);
}

int bfs(int x, int y, int num_type, vector< vector<int> > &pixel_type, l &line, const Mat &image)
{
    int count = 0;
    queue<pair<int, int> > q;
    q.push(make_pair(x, y));
    pixel_type[x][y] = num_type;
    line[0].push_back(Point2f(x, y));
    while(!q.empty())
    {
        pair<int, int> index_pixel = q.front();
        q.pop();
        ++count;
        for(int dx = -1; dx <= 1; ++dx)
            for(int dy = -1; dy <= 1; ++dy)
                if(index_pixel.first + dx >= 0 && index_pixel.first + dx < image.rows && index_pixel.second + dy >= 0 && index_pixel.second + dy < image.cols)
                    if(pixel_type[index_pixel.first + dx][index_pixel.second + dy] == 0 && image.at<uchar>(index_pixel.first + dx, index_pixel.second + dy) > static_cast<unsigned char>(0))
                        {
                            pixel_type[index_pixel.first + dx][index_pixel.second + dy] = num_type;
                            q.push(make_pair(index_pixel.first + dx, index_pixel.second + dy));
                            line[0].push_back(Point2f(index_pixel.first + dx, index_pixel.second + dy));
                        }
    }
    return count;
}

bool merge(l &line1, pair<float,float> &coef1, l &line2, pair<float,float> &coef2)
{
    float distance1 = INF, distance2 = INF;
    for(int segment = 0; segment < line1.size(); ++ segment)
        for(int point = 0; point < line1[segment].size(); ++point)
            distance1 = min(distance1, distance_point_to_line(line1[segment][point], coef2));
    for(int segment = 0; segment < line2.size(); ++ segment)
        for(int point = 0; point < line2[segment].size(); ++point)
            distance2 = min(distance2, distance_point_to_line(line2[segment][point], coef1));
    if(max(distance1, distance2) < 0.05)
        return true;
    return false;
}


template <typename T>
void my_push(vector<T> &vec1, vector<T> &vec2)
{
    for(int element = 0; element < vec2.size(); ++ element)
        vec1.push_back(vec2[element]);
}

int main()
{
    int number_of_lines = 3;
    Mat image1, image_rgb;
    image1 = imread("a_result.jpg", 0);
    vector<unsigned char> pixel_vector;
    for(int x = 0; x < image1.rows; ++x)
        for(int y = 0; y < image1.cols; ++y)
            pixel_vector.push_back(image1.at<uchar>(x, y));
    int k_statistic = image1.rows*image1.cols - 4*number_of_lines*max(image1.rows, image1.cols);
    nth_element(pixel_vector.begin(), pixel_vector.begin()+k_statistic, pixel_vector.end());
    unsigned char light_limit = pixel_vector[k_statistic];

    for(int x = 0; x < image1.rows; ++x)
        for(int y = 0; y < image1.cols; ++y)
            if(image1.at<uchar>(x, y) < light_limit)
                image1.at<uchar>(x, y) = 0;

    cvtColor(image1, image_rgb, COLOR_GRAY2RGB);
    vector< vector<int> > pixel_type(image1.rows, vector<int>(image1.cols, 0));
    int num_type = 0;
    vector <l> lines;
    vector< pair<float, float> > coefficients;
    for(int x = 0; x < image1.rows; ++x)
        for(int y = 0; y < image1.cols; ++y)
            if (pixel_type[x][y] == 0)
            {
                ++num_type;
                l initial_line(1);
                if(bfs(x, y, num_type, pixel_type, initial_line, image1) > 20)
                {
                    lines.push_back(initial_line);
                    coefficients.push_back(line_model_fit(initial_line));
                    //line_model_fit_and_image(initial_line, image_rgb);
                }
            }
    /*
    imshow("segment", image_rgb);
    waitKey();
    imwrite("initial_lines.jpg", image_rgb);
    cvtColor(image1, image_rgb, COLOR_GRAY2RGB);
    */
    int lenght = lines.size();
    for(int line1 = 0; line1 < lenght-1; ++line1)
        for(int line2 = line1+1; line2 < lenght; ++line2)
            if(merge(lines[line1], coefficients[line1], lines[line2], coefficients[line2]))
            {
                my_push(lines[line1], lines[line2]);
                coefficients[line1] = line_model_fit(lines[line1]);
                swap(lines[line2], lines[lenght-1]);
                swap(coefficients[line2], coefficients[lenght-1]);
                lenght -= 1;
                line1 = -1;
                break;
            }
    for(int line = 0; line < lenght; ++line)
        if(lines[line].size() > 1)
            segment_image(lines[line], image_rgb);

    imshow("segment", image_rgb);
    waitKey();
    imwrite("lines_group.jpg", image_rgb);

    return 0;
}
