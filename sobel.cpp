#include <iostream>
#include <opencv2/highgui/highgui.hpp>
using namespace std;
using namespace cv;

int main()
{
    Mat image1, image2;
    image1 = imread("a.gpg", 0); //CV_LOAD_IMAGE_GRAYSCALE 
    image2 = image1.clone();
    for(int x = 2; x < image1.rows - 2; ++x)
        for(int y = 2; y < image1.cols - 2; ++y)
        {
            double gy, gx, sum;
            gy = image1.at<uchar>(x-1, y-1) + 2*image1.at<uchar>(x, y-1) + image1.at<uchar>(x+1, y-1)/
            - image1.at<uchar>(x-1, y+1) - 2*image1.at<uchar>(x, y+1) - image1.at<uchar>(x+1, y+1);

            gx = image1.at<uchar>(x-1, y-1) + 2*image1.at<uchar>(x-1, y) + image1.at<uchar>(x-1, y+1)/
            - image1.at<uchar>(x+1, y-1) - 2*image1.at<uchar>(x+1, y) - image1.at<uchar>(x+1, y+1);

            sum = sqrt(pow(gy, 2) + pow(gx, 2));
            if(sum > 255)
                sum = 255;
            image2.at<uchar>(x, y) = sum;
        }
    namedWindow("Sobel operator");
    imshow("Sobel operator", image2);

    waitKey();

    return 0;
}
