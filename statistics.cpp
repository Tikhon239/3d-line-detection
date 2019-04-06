#include <iostream>
#include <vector>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
using namespace cv;
using namespace std;
using seg = vector<Point2f>;
using l = vector<seg>;
#include "statistics.hpp"
pair <float, float> centroids(seg &segment)
{
    float x_centroid = 0, y_centroid = 0;
    for(int point = 0; point < segment.size(); ++point)
    {
        x_centroid += segment[point].x;
        y_centroid += segment[point].y;
    }
    return make_pair(float(x_centroid)/segment.size(), float(y_centroid)/segment.size());
}

vector<float> statistics(seg &segment)
{
    vector<float> stat(5);
    pair <float, float> temp = centroids(segment);
    stat[0] = temp.first;
    stat[1] = temp.second;
    for(int point = 0; point < segment.size(); ++point)
    {
        stat[2] += (segment[point].x - stat[0])*(segment[point].x - stat[0]);
        stat[3] += (segment[point].y - stat[1])*(segment[point].y - stat[1]);
        stat[4] += (segment[point].x - stat[0])*(segment[point].y - stat[1]);
    }
    //stat[2] = float(stat[2])/segment.size();
    //stat[3] = float(stat[3])/segment.size();
    //stat[4] = float(stat[4])/segment.size();
    return stat;
}
