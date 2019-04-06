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
#include "intersection.hpp"
#include "gauss.hpp"

int bfs(int x, int y, int num_type, vector< vector<int> > &pixel_type, const Mat &image)
{
    int count = 0;
    queue<pair<int, int> > q;
    q.push(make_pair(x, y));
    pixel_type[x][y] = num_type;
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
                        }
    }

    return count;
}
int main()
{
    float angle_precision = 0.25;
	Mat image1, image2, image_haf, image4, image5, image_extremum, image_rgb;
    image1 = imread("min.jpg", 0);
    image2 = Mat(2*(image1.rows + image1.cols), 181/angle_precision, CV_32F, Scalar(0));
    image_haf = Mat(2*(image1.rows + image1.cols), 181/angle_precision - 1, CV_8UC1, Scalar(0,0,0));
    image4 = Mat(2*(image1.rows + image1.cols), 181/angle_precision - 1, CV_32F, Scalar(0));
    image_extremum = Mat(2*(image1.rows + image1.cols), 180/angle_precision, CV_8UC1, Scalar(0,0,0));
    image5 = Mat(image1.rows, image1.cols, CV_8UC1, Scalar(0,0,0));
    vector<unsigned char> pixel_vector;
    for(int x = 0; x < image1.rows; ++x)
        for(int y = 0; y < image1.cols; ++y)
            pixel_vector.push_back(image1.at<uchar>(x, y));
    int k_statistic = image1.rows*image1.cols - 12*max(image1.rows, image1.cols);
    nth_element(pixel_vector.begin(), pixel_vector.begin()+k_statistic, pixel_vector.end());
    unsigned char light_limit = pixel_vector[k_statistic];
    cout << "limit: " << static_cast<int>(light_limit) << endl;
    
    for(int x = 0; x < image1.rows; ++x)
        for(int y = 0; y < image1.cols; ++y)
            if(image1.at<uchar>(x, y) < light_limit)
                image1.at<uchar>(x, y) = 0;

    vector< vector<int> > pixel_type(image1.rows, vector<int>(image1.cols, 0));
    vector<int> count_type;
    int num_type = 0;
    count_type.push_back(0);

    for(int x = 0; x < image1.rows; ++x)
        for(int y = 0; y < image1.cols; ++y)
            if (pixel_type[x][y] == 0)
            {
                ++num_type;
                count_type.push_back(bfs(x, y, num_type, pixel_type, image1));
            }

    for(int x = 0; x < image1.rows; ++x)
        for(int y = 0; y < image1.cols; ++y)
            if(count_type[pixel_type[x][y]] < static_cast<unsigned char>(20))
                image1.at<uchar>(x, y) =  static_cast<unsigned char>(0);

    for(int x = 0; x < image1.rows; ++x)
        for(int y = 0; y < image1.cols; ++y)
        {
            //if(image1.at<uchar>(x, y) == 1)
            if(image1.at<uchar>(x, y) >= static_cast<unsigned char>(light_limit))
            {
            	for(float angle = 0; angle < 180; angle += angle_precision)
            	{
            		float a1 = angle;
            		float a2 = angle + angle_precision;
            		int r1 = round(x*cos(a1*M_PI/180) + y*sin(a1*M_PI/180) + (image1.rows + image1.cols));
            		int r2 = round(x*cos(a2*M_PI/180) + y*sin(a2*M_PI/180) + (image1.rows + image1.cols));
            		if(r1 == r2)
            			image2.at<float>(r1, a1/angle_precision) += image1.at<uchar>(x, y);
                    int sign = 1;
                	if(r2 < r1)
                        sign = -1;
            		for(int r = r1; r != r2; r += sign)
        			{
        				float w1 = float(r2 - r)/(r2 - r1);
      					float w2 = 1.0 - w1;
      					image2.at<float>(r, a1/angle_precision) += w1*image1.at<uchar>(x, y);
       					image2.at<float>(r, a2/angle_precision) += w2*image1.at<uchar>(x, y);
        			}
                }
            }
        }

    float max_value = 0;
    for(int x = 0; x < image2.rows; ++x)
        for(int y = 0; y < image2.cols; ++y)
            if(image2.at<float>(x, y) > max_value)
                max_value = image2.at<float>(x, y);
    cout << max_value << endl;

    for(int x = 0; x < image_haf.rows; ++x)
        for(int y = 0; y < image_haf.cols; ++y)
        	image_haf.at<uchar>(x, y) = static_cast<unsigned char>(image2.at<float>(x, y)*255/max_value);

    float sigma = 1;
    int kernel_size = 3;
    int shift = kernel_size/2;
    Mat blur_image2 = Mat(2*(image1.rows + image1.cols), 181/angle_precision, CV_32F, Scalar(0));
    Mat g_filter = gaus_filter(sigma, kernel_size, 0);
    for(int x = 0; x < image2.rows; ++x)
        for(int y = 0; y < image2.cols; ++y)
        {
            float g = 0; 
            for(int ix = 0; ix<kernel_size; ++ix)
                for(int iy = 0; iy<kernel_size; ++iy)
                    if(x - shift + ix >= 0 && x - shift + ix < image2.rows && y - shift + iy >= 0 && y - shift + iy < image2.cols)
                    g += image2.at<float>(x - shift + ix, y - shift + iy)*g_filter.at<float>(ix,iy);
            blur_image2.at<float>(x, y) = g;
        }

    float blur_max_value = 0;
    for(int x = 0; x < blur_image2.rows; ++x)
        for(int y = 0; y < blur_image2.cols; ++y)
            if(blur_image2.at<float>(x, y) > blur_max_value)
                blur_max_value = blur_image2.at<float>(x, y);

	for(int x = 1; x < image4.rows-1; ++x)
        for(int y = 1; y < image4.cols-1; ++y)
        {
        	bool flag = true;
        	for(int dx = -1; dx <= 1; ++dx)
				for(int dy = -1; dy <= 1; ++dy)
                    if(dx != 0 || dy != 0)
                        if(blur_image2.at<float>(x, y) <= blur_image2.at<float>(x+dx, y+dy))
                            flag = false;
			if(flag)
            {
				image4.at<float>(x, y) = blur_image2.at<float>(x, y); 
                image_extremum.at<uchar>(x, y) = image_haf.at<uchar>(x, y);
            }
        }

    for(int x = 1; x < image4.rows-1; ++x)
    {
        bool flag = true;
        if(blur_image2.at<float>(x, 0) <= blur_image2.at<float>(x+1, 0) || blur_image2.at<float>(x, 0) <= blur_image2.at<float>(x-1, 0))
            flag = false;
        for(int dx = -1; dx <= 1; ++dx)
             if(blur_image2.at<float>(x, 0) <= blur_image2.at<float>(x+dx, 1) || blur_image2.at<float>(x, 0) <= blur_image2.at<float>(image4.rows - x + 1 + dx, 181/angle_precision - 1))
                flag = false;
        if(flag)
        {
            image4.at<float>(x, 0) = blur_image2.at<float>(x, 0);
            image_extremum.at<uchar>(x, 0) = image_haf.at<uchar>(x, 0);
        }
    }

    for(int x = 1; x < image4.rows-1; ++x)
    {
        bool flag = true;
        if(blur_image2.at<float>(x, 181/angle_precision - 1) <= blur_image2.at<float>(x+1, 181/angle_precision - 1) || blur_image2.at<float>(x, 181/angle_precision - 1) <= blur_image2.at<float>(x-1, 181/angle_precision - 1))
            flag = false;
        for(int dx = -1; dx <= 1; dx = ++dx)
            if(blur_image2.at<float>(x, 181/angle_precision - 1) <= blur_image2.at<float>(x+dx, 181/angle_precision - 2) || blur_image2.at<float>(x, 181/angle_precision - 1) <= blur_image2.at<float>(image4.rows - x + 1 + dx, 0))
                flag = false;
        if(flag)
        {
            image4.at<float>(x, 181/angle_precision - 1) = blur_image2.at<float>(x, 181/angle_precision - 1);
            image_extremum.at<uchar>(x, 181/angle_precision - 1) = image_haf.at<uchar>(x, 181/angle_precision - 1);
        }
    }
    cvtColor(image1 ,image_rgb, COLOR_GRAY2RGB);
    for(int x = 0; x < image4.rows; ++x)
        for(int y = 0; y < image4.cols; ++y)
            if(image4.at<float>(x, y) > 0.5*blur_max_value)
            {
                int r = x - image4.rows/2;
                int angle = y*angle_precision;
                Point2f a;
                Point2f b;
                if(angle == 0)
                {
                    a = Point2f(0, r);
                    b = Point2f(image_rgb.cols, r);
                } 
                else if (angle == 90)
                {
                    a = Point2f(r, 0);
                    b = Point2f(r, image_rgb.rows);
                }  
                else
                {
                    a = Point2f(r/sin(angle*M_PI/180), 0);
                    b = Point2f(r/sin(angle*M_PI/180) - image_rgb.rows*cos(angle*M_PI/180)/sin(angle*M_PI/180), image_rgb.rows);
                    pair<Point2f, Point2f> temp = intersection_line_and_rectangle(a, b, image_rgb.cols, image_rgb.rows);
                    a = temp.first;
                    b = temp.second;
                }
                cout << x << " " << r<< " " << y << " " << a << " " << b << endl;
                line(image_rgb, a, b, Scalar(0, 0, 255), 1);
            }
    imshow("haf", image_haf);
	waitKey();
	imwrite("min_test_lines.jpg", image_rgb);
    imwrite("min_haf_lines.jpg", image_haf);
    imwrite("min_extremum_lines.jpg", image_extremum);
	return 0;
}
