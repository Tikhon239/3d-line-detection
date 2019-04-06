#include <iostream>
#include <vector>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
using namespace cv;
using namespace std;
#include "gauss.hpp"

int main()
{
	float sigma = 1.5;
	int kernel_size = 5;
	int shift = kernel_size/2;
	Mat image1, image2;
    image1 = imread("a.jpg", 0);
    image2 = Mat(image1.rows, image1.cols, CV_8UC1, Scalar(0,0,0));
	vector<Mat> filters;
    for(float angle = 0; angle < 9*M_PI/8; angle += M_PI/8)
    	filters.push_back(-gaus_filter_xx(sigma, kernel_size, angle));

    int filters_size = filters.size();
	for(int x = shift; x < image1.rows-shift; ++x)
        for(int y = shift; y < image1.cols-shift; ++y)
        	for(int angle =0; angle < filters_size; ++ angle)
        	{
	            float g2 = 0; 
	            for(int ix = 0; ix<kernel_size; ++ix)
	            	for(int iy = 0; iy<kernel_size; ++iy)
	            		g2 += image1.at<uchar>(x - shift + ix, y - shift + iy)*filters[angle].at<float>(ix,iy);
	            if(g2 > 255)
	                g2 = 255;
	            if(g2 < 0)
	                g2 = 0;
	            image2.at<uchar>(x, y) += static_cast<unsigned char>(g2/filters_size);
	            //image2.at<uchar>(x, y) = max(static_cast<unsigned char>(g2), image2.at<uchar>(x, y));
	    	}
    //normalize(image2, image2, 0, 255, NORM_MINMAX);
	imshow("result", image2);
	waitKey();
	imwrite("result.jpg", image2);
	return 0;
}
