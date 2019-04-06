#include <iostream>
#include <vector>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
using namespace std;
using namespace cv;

#define INF 1000000
using seg = vector<Point2f>;
using l = vector<seg>;

#include "line_fit.hpp"
#include "intersection.hpp"
#include "statistics.hpp"

bool comp(Point2f &a, Point2f &b)
{
    return a.y < b.y;
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

pair <float, float> line_model_fit(seg &segment)
{
    vector<float> stat;
    stat = statistics(segment);
    float k, b;
    k = stat[4]/stat[2];
    b = stat[1] - k*stat[0];
    return(make_pair(k, b));
}

void line_model_fit_and_image(seg &segment, Mat &image)
{
    vector<float> stat;
    stat = statistics(segment);
    float k, b;
    k = stat[4]/stat[2];
    b = stat[1] - k*stat[0];
    pair<Point2f,Point2f> temp = first_end(segment);
    Point2f p1 = Point2f(k*temp.first.x + b, temp.first.x);
    Point2f p2 = Point2f(k*temp.second.x + b, temp.second.x);
        
    cv::line(image, p1, p2, Scalar(0, 0, 255), 1);
}

void segment_image(l &line, vector< pair<float, float> > &line_coefs, Mat &image)
{
    vector<Point2f> supporting_points;
    pair<Point2f,Point2f> temp;
    for(int segment = 0; segment < line.size(); ++segment)
    {
        temp = first_end(line[segment]);
        float k = line_coefs[segment].first;
        float b = line_coefs[segment].second;
        Point2f p1 = Point2f(k*temp.first.x + b, temp.first.x);
        Point2f p2 = Point2f(k*temp.second.x + b, temp.second.x);
        supporting_points.push_back(p1);
        supporting_points.push_back(p2);
    }

    sort(supporting_points.begin(), supporting_points.end(), comp);
    int point = 0;
    /*
    if(supporting_points[point].x > 0)
    {
        temp = intersection_line_and_rectangle(supporting_points[point], supporting_points[point+2], image.cols, image.rows);
        if(temp.first.x < supporting_points[point].x)
            cv::line(image, temp.first, supporting_points[point], Scalar(0, 0, 255), 1);
        else
            cv::line(image, temp.second, supporting_points[0], Scalar(0, 0, 255), 1);
    }
    */
    for(; point < supporting_points.size()-1; ++point)
        cv::line(image, supporting_points[point], supporting_points[point+1], Scalar(0, 0, 255), 1);
    /*
    if(supporting_points[point+1].x < image.cols)
    {
        temp = intersection_line_and_rectangle(supporting_points[point-2], supporting_points[point], image.cols, image.rows);
         if(supporting_points[point].x < temp.first.x)
            cv::line(image, supporting_points[point], temp.first, Scalar(0, 0, 255), 1);
        else
            cv::line(image, supporting_points[point], temp.second, Scalar(0, 0, 255), 1);
    }
    */

}

bool merge(l &line1, vector< pair<float,float> >&coefs1, l &line2, vector <pair<float,float> > &coefs2)
{
    for(int segment1 = 0; segment1 < line1.size(); ++ segment1)
    {
        float distance1 = INF, distance2 = INF;
        for(int segment2 = 0; segment2 < line2.size(); ++ segment2)
        {
            pair<Point2f, Point2f> temp1 = first_end(line1[segment1]);
            pair<Point2f, Point2f> temp2 = first_end(line2[segment2]);
            if(distance_segment_to_segment(temp1, temp2) < 4*min(evklid(temp1.first, temp1.second), evklid(temp2.first, temp2.second)))
            {
                for(int point = 0; point < line1[segment1].size(); ++point)
                    distance1 = min(distance1, distance_point_to_line(line1[segment1][point], coefs2[segment2]));
                for(int point = 0; point < line2[segment2].size(); ++point)
                    distance2 = min(distance2, distance_point_to_line(line2[segment2][point], coefs1[segment1]));
                if(max(distance1, distance2) < 1)  
                {  
                    //cout << distance1 << " " << distance2 << endl;
                    return true;
                }
            }
        }
    }
    return false;
}
