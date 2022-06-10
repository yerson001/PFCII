/***********************************
 *  name: sanchez yucra yhon yerson
 *  info: CV 2022
 *  proy: self driving car
 ***********************************/

#include<opencv2/opencv.hpp>
#include<iostream>
#include <opencv2/imgproc.hpp>
#include "laneDetection.h"
#include "functions.h"
#include "calibration.h"

funciones fun;
using namespace std;
using namespace cv;


int sliderValue = 0;
VideoCapture laneVideo;
Mat videoFrame; // Video Frame.
Mat videoFrameUndistorted; // Video Frame (after calibration).
Mat videoFramePerspective; // Video Frame (after perspective transform).
Mat _videoFrameUndistorted;
Mat debugWindow(540, 1280, CV_8UC3, Scalar(0,0,0)); //The Debug window.
Size videoSize; // Input Variable Size.
Mat cameraMatrix, dist; //Calibration Matrix.
Mat perspectiveMatrix; //Homography Matrix.
String coordinatetext = "";

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


std::string PATH = "/home/y3rsn/Dev/cpp/CV_2022/Roadtraking/img/road_img.png";
//std::string PATH2 = "/home/y3rsn/Dev/cpp/CV_2022/Roadtraking/video/test5.mp4";


int main(int argc, char **argv)
{
    perspectiveMatrix = getPerspectiveTransform(perspectiveSrc,perspectiveDst);

    laneVideo.open("/home/yrsn/Dev/PFCII/video/test5.mp4");
    videoSize = Size((int)laneVideo.get(CAP_PROP_FRAME_WIDTH),
                         (int)laneVideo.get(CAP_PROP_FRAME_HEIGHT));

    string PATH = "/home/yrsn/Dev/ALD-main/camera_cal/";

    //--------------------calibrar camara-----------------

    FileStorage fsRead;
    fsRead.open("Intrinsic.xml", FileStorage::READ);
    Mat src = imread(PATH+"calibration1.jpg");
    Mat dst;
    if (fsRead.isOpened() == false)
    {
        CameraCalibrator myCameraCalibrator(PATH,0);
        myCameraCalibrator.doCalibration(cameraMatrix, dist);
        FileStorage fs;
        fs.open("Intrinsic.xml", FileStorage::WRITE);
        fs << "CameraMatrix" << cameraMatrix;
        fs << "Dist" << dist;
        fs.release();
        fsRead.release();
        cout << "There is no existing intrinsic parameters XML file." << endl;
        cout << "Start calibraton......" << endl;
    }
    else
    {
        fsRead["CameraMatrix"] >> cameraMatrix;
        fsRead["Dist"] >> dist;
        fsRead.release();
    }
    undistort(src, dst, cameraMatrix, dist);

    //Display Video Image
    laneVideo.set(CAP_PROP_POS_FRAMES, 0);
    laneVideo >> videoFrame;

    //bool check = imwrite("/home/yrsn/Dev/PFCII/img/drone.jpg", videoFrame);


/*
    circle(videoFrame,perspectiveSrc[0], 3, Scalar(0,255,0), FILLED);
    circle(videoFrame,perspectiveSrc[1], 3, Scalar(0,255,0), FILLED);
    circle(videoFrame,perspectiveSrc[3], 3, Scalar(0,255,0), FILLED);
    circle(videoFrame,perspectiveSrc[2], 3, Scalar(0,255,0), FILLED);
    line(videoFrame, perspectiveSrc[0], perspectiveSrc[1], Scalar(255,0,0), 1);
    line(videoFrame, perspectiveSrc[1], perspectiveSrc[3], Scalar(255,0,0), 1);
    line(videoFrame, perspectiveSrc[3], perspectiveSrc[2], Scalar(255,0,0), 1);
    line(videoFrame, perspectiveSrc[2], perspectiveSrc[0], Scalar(255,0,0), 1);
*/

    //namedWindow("Original Image", WINDOW_NORMAL);
    //imshow("Original Image",videoFrame);
    undistort(videoFrame, videoFrameUndistorted, cameraMatrix, dist);
    _videoFrameUndistorted = videoFrameUndistorted.clone();

    //Start Homography
    warpPerspective(_videoFrameUndistorted, videoFramePerspective, perspectiveMatrix, videoSize);

    //imshow("Original warp",_videoFrameUndistorted);

    //imshow("Original _warp",videoFramePerspective);

    //Applying lane detection algorithm

    laneDetection LaneAlgo(_videoFrameUndistorted, perspectiveMatrix);
    imshow("send",_videoFrameUndistorted);
    LaneAlgo.laneDetctAlgo();

    Mat warpEdge; warpEdge = LaneAlgo.getWarpEdgeDetectResult().clone();

    fun.ReadVideo();


    waitKey(0);
    laneVideo.release();
    destroyAllWindows();
    return 0;
}







