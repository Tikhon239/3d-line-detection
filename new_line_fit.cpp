#include <iostream>
#include <vector>
#include <queue>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
using namespace std;
using namespace cv;

using seg = vector<Point2f>;
using l = vector<seg>;
#include "line_fit.hpp"

int bfs(int x, int y, int num_type, vector< vector<int> > &pixel_type, l &line, const Mat &image)
{
    int count = 0;
    queue<pair<int, int> > q;
    q.push(make_pair(x, y));
    pixel_type[x][y] = num_type;
    line[0].push_back(Point2f(x, y));
    while(!q.empty() && count< 200)
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

template <typename T>
void my_push(vector<T> &vec1, vector<T> &vec2)
{
    for(int element = 0; element < vec2.size(); ++element)
        vec1.push_back(vec2[element]);
}

int main()
{
    int number_of_lines = 3;
    Mat image1, image_rgb;
    image1 = imread("result.jpg", 0);
    vector<unsigned char> pixel_vector;
    for(int x = 0; x < image1.rows; ++x)
        for(int y = 0; y < image1.cols; ++y)
            pixel_vector.push_back(image1.at<uchar>(x, y));
    int k_statistic = image1.rows*image1.cols - 4*number_of_lines*max(image1.rows, image1.cols);
    nth_element(pixel_vector.begin(), pixel_vector.begin()+k_statistic, pixel_vector.end());
    unsigned char light_limit = pixel_vector[k_statistic];
    //cout << "limit: " << static_cast<int>(light_limit) << endl;

    for(int x = 0; x < image1.rows; ++x)
        for(int y = 0; y < image1.cols; ++y)
            if(image1.at<uchar>(x, y) < light_limit)
                image1.at<uchar>(x, y) = 0;

    cvtColor(image1, image_rgb, COLOR_GRAY2RGB);
    vector< vector<int> > pixel_type(image1.rows, vector<int>(image1.cols, 0));
    int num_type = 0;
    vector <l> lines;
    vector <vector< pair<float, float> > > coefficients;
    for(int x = 0; x < image1.rows; ++x)
        for(int y = 0; y < image1.cols; ++y)
            if (pixel_type[x][y] == 0)
            {
                ++num_type;
                l initial_line(1);
                if(bfs(x, y, num_type, pixel_type, initial_line, image1) > 20)
                {
                    lines.push_back(initial_line);
                    vector<pair<float, float>> initial_coeff;
                    initial_coeff.push_back(line_model_fit(initial_line[0]));
                    coefficients.push_back(initial_coeff);
                    //line_model_fit_and_image(initial_line[0], image_rgb);
                }
            }
    /*
    imshow("segment", image_rgb);
    waitKey();
    imwrite("initial_lines.jpg", image_rgb);
    cvtColor(image1, image_rgb, COLOR_GRAY2RGB);
    */
    cout << "БФС отработал" << endl;
    int lenght = lines.size();
    for(int line1 = 0; line1 < lenght-1; ++line1)
        for(int line2 = line1+1; line2 < lenght; ++line2)
            if(merge(lines[line1], coefficients[line1], lines[line2], coefficients[line2]))
            {
                //cout << line1 << " " << line2 << endl;
                my_push(lines[line1], lines[line2]);
                my_push(coefficients[line1], coefficients[line2]);
                swap(lines[line2], lines[lenght-1]);
                swap(coefficients[line2], coefficients[lenght-1]);
                lenght -= 1;
                line1 = -1;
                break;
            }

    for(int line = 0; line < lenght; ++line)
    {
        if(lines[line].size() > 10)
            segment_image(lines[line], coefficients[line], image_rgb);
    }
    
    imshow("segment", image_rgb);
    waitKey();
    imwrite("new_lines_group.jpg", image_rgb);
    return 0;
}
