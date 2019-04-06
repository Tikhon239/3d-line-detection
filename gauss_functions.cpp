#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
using namespace cv;
#include "gauss.hpp"

Mat normalize(Mat matrix, float positiv_sum, float negativ_sum)
{
	for(int x = 0; x < matrix.rows; ++x)
		for(int y = 0; y < matrix.cols; ++y)
		{
			if(matrix.at<float>(x,y) > 0)
				matrix.at<float>(x,y) /= positiv_sum;
			else if(matrix.at<float>(x,y) < 0)
				matrix.at<float>(x,y) /= negativ_sum;
		}
	return matrix;
}

float gaussian_function(float x, float y, float sigma)
{
	float var = sigma*sigma;
	return exp(-(x*x + y*y)/(2*var))/(sqrt(2*M_PI*var));
}

float gaussian_function_x(float x, float y, float sigma)
{
	return -x*gaussian_function(x, y, sigma)/(sigma*sigma);
}

float gaussian_function_xx(float x, float y, float sigma)
{
	float var = sigma*sigma;
	return (x*x-var)*gaussian_function(x, y, sigma)/(var*var);
}

Mat gaus_filter(float sigma, int kernel_size, float angle)
{
	Mat g2 = Mat(kernel_size, kernel_size, CV_32F);
	int shift = kernel_size/2;
	float positiv_sum = 0;
	float negativ_sum = 0;

	for(int i = 0; i < kernel_size; ++i)
		for(int j = 0; j < kernel_size; ++j)
		{
			float i1 = (i - shift)*cos(angle) - (j - shift)*sin(angle);
			float j1 = (j - shift)*cos(angle) + (i - shift)*sin(angle);
			g2.at<float>(i,j) = gaussian_function(i1, j1, sigma);
			if(g2.at<float>(i,j) > 0)
				positiv_sum += g2.at<float>(i,j);
			else if(g2.at<float>(i,j) < 0)
				negativ_sum -= g2.at<float>(i,j);
		}
	return normalize(g2, positiv_sum, negativ_sum);
}

Mat gaus_filter_xx(float sigma, int kernel_size, float angle)
{
	Mat g2 = Mat(kernel_size, kernel_size, CV_32F);
	int shift = kernel_size/2;
	float positiv_sum = 0;
	float negativ_sum = 0;

	for(int i = 0; i < kernel_size; ++i)
		for(int j = 0; j < kernel_size; ++j)
		{
			float i1 = (i - shift)*cos(angle) - (j - shift)*sin(angle);
			float j1 = (j - shift)*cos(angle) + (i - shift)*sin(angle);
			g2.at<float>(i,j) = gaussian_function_xx(i1, j1, sigma);
			if(g2.at<float>(i,j) > 0)
				positiv_sum += g2.at<float>(i,j);
			else if(g2.at<float>(i,j) < 0)
				negativ_sum -= g2.at<float>(i,j);
		}
	return normalize(g2, positiv_sum, negativ_sum);
}
