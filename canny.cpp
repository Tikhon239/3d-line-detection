#include <iostream>
#include <vector>
#include <queue>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
using namespace std;
using namespace cv;

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
					if(pixel_type[index_pixel.first + dx][index_pixel.second + dy] == 0 && image.at<uchar>(index_pixel.first + dx, index_pixel.second + dy) > static_cast<unsigned char>(50))
						{
							pixel_type[index_pixel.first + dx][index_pixel.second + dy] = num_type;
							q.push(make_pair(index_pixel.first + dx, index_pixel.second + dy));
						}
	}

	return count;
}



int main()
{
	Mat image1, image2, image3;
    image1 = imread("result.jpg", 0);
    image2 = Mat(image1.rows, image1.cols, CV_8UC1, Scalar(0,0,0));
    image3 = Mat(image1.rows, image1.cols, CV_8UC1, Scalar(0,0,0));
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
        	if(count_type[pixel_type[x][y]] > image1.rows/10)
        		image2.at<uchar>(x, y) = 255;
    imshow("canny", image2);
	waitKey();
	imwrite("canny.jpg", image2);
	return 0;
}
