//percpectivefile

#include<opencv2/highgui/highgui.hpp>
#include<opencv2/opencv.hpp>
#include<iostream>
#include <opencv2/imgproc.hpp>
using namespace cv;

int main()
{
    Mat input = imread("/home/yrsn/Dev/PFCII/img/drone.jpg");
    imshow("ffff",input);

    Mat output;

    Point2f perspectiveSrc[] = {
        Point2f(245,370),
        Point2f(400,370),
        Point2f(180,465),
        Point2f(455,465)
    };

    Point2f perspectiveDst[] = {
        Point2f(0,0),
        Point2f(370,0),
        Point2f(0,465),
        Point2f(370,465)
    };

    Mat Matrix = getPerspectiveTransform(perspectiveSrc,perspectiveDst);

    warpPerspective(input, output, Matrix,Size(370,465));

    imshow("cnc",output);

    waitKey(0);
    return 0;
}
// /home/yrsn/Dev/PFCII/img/drone.jpg
