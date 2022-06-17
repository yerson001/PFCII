#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <iostream>
#include <string>
#include "laneDetection.h"
#include "calibration.h"
#include "functions.h"

using namespace cv;
using namespace std;
void videoSliderCallback(int, void*);

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
    Point2f(570,490),
    Point2f(720,490),
    Point2f(450,698),
    Point2f(830,698)};

Point2f perspectiveDst[] = {
    Point2f(200,0),
    Point2f(980,0),
    Point2f(200,720),
    Point2f(980,720)};



funciones fun;


int main(int argc, char **argv)
{
    //Get the Perspective Matrix.
    perspectiveMatrix = getPerspectiveTransform(perspectiveSrc,perspectiveDst);
    //rperspectiveMatrix = getPerspectiveTransform(rperspectiveSrc,rperspectiveDst);


    string d = "jkj";
    // *********************READ VIDEO*************************
    //laneVideo.open("/home/yrsn/Dev/ALD-main/video/video.mp4");
    laneVideo.open("/home/yrsn/Dev/ALD-main/video/video2.avi");
    //laneRoad.open("/home/yrsn/Dev/ALD-main/video/video2.avi");


    videoSize = Size((int)laneVideo.get(CAP_PROP_FRAME_WIDTH),(int)laneVideo.get(CAP_PROP_FRAME_HEIGHT));
    //roadSize = Size((int)laneRoad.get(CAP_PROP_FRAME_WIDTH),(int)laneRoad.get(CAP_PROP_FRAME_HEIGHT));




    std::string PATH = "/home/yrsn/Dev/ALD-main/camera_cal/";

    //--------------Camera Calibration Start-----------------
    FileStorage fsRead;
    fsRead.open("Intrinsic.xml", FileStorage::READ);
    Mat src = imread(PATH+"calibration1.jpg");
    Mat dst;
    if (fsRead.isOpened() == false)
    {
        CameraCalibrator myCameraCalibrator(d,0);
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
    //--------------Camera Calibration Finish-----------------
    //Display Video Image
    //Display Video Image
     laneVideo.set(CAP_PROP_POS_FRAMES, 0);
     laneVideo >> videoFrame;
     namedWindow("Original Image", WINDOW_NORMAL);
     imshow("Original Image",videoFrame);


     undistort(videoFrame, videoFrameUndistorted, cameraMatrix, dist);
     _videoFrameUndistorted = videoFrameUndistorted.clone();
     imshow("Image",fun.ResizeImage(_videoFrameUndistorted,0.5));



     //draw the roi (for perspective transform)
     line(videoFrameUndistorted, perspectiveSrc[0], perspectiveSrc[1], Scalar(0,0,255), 2);
     line(videoFrameUndistorted, perspectiveSrc[1], perspectiveSrc[3], Scalar(0,0,255), 2);
     line(videoFrameUndistorted, perspectiveSrc[3], perspectiveSrc[2], Scalar(0,0,255), 2);
     line(videoFrameUndistorted, perspectiveSrc[2], perspectiveSrc[0], Scalar(0,0,255), 2);
     circle(videoFrameUndistorted, perspectiveSrc[0], 6, Scalar(0,0,255), FILLED);
     circle(videoFrameUndistorted, perspectiveSrc[1], 6, Scalar(0,0,255), FILLED);
     circle(videoFrameUndistorted, perspectiveSrc[2], 6, Scalar(0,0,255), FILLED);
     circle(videoFrameUndistorted, perspectiveSrc[3], 6, Scalar(0,0,255), FILLED);

     imshow("video-frame",videoFrameUndistorted);
     //imshow("road-frame",roadFrameUndistorted);


     int sliderMaxValue = laneVideo.get(CAP_PROP_FRAME_COUNT) - 10;
     createTrackbar("Frame", "Original Image", &sliderValue, sliderMaxValue, videoSliderCallback);
     videoSliderCallback(sliderValue, 0);


     //Start Homography
     warpPerspective(_videoFrameUndistorted, videoFramePerspective, perspectiveMatrix, videoSize);


     /***********************************************************
     *instancia de la clase parametros la imagencorregida por
     * calibracion y la matriz de perspectiva que contiene la
     * zonade interes
     ***********************************************************/
     laneDetection LaneAlgo(_videoFrameUndistorted, perspectiveMatrix);
     LaneAlgo.laneDetctAlgo("roadline");




     /***********************************************************
     *
     *
     ***********************************************************/



     waitKey(0);
     return 0;

}


void videoSliderCallback(int, void*)
{
    laneVideo.set(CAP_PROP_POS_FRAMES, sliderValue);
    laneVideo >> videoFrame;
    imshow("Original Image",fun.ResizeImage(videoFrame,0.5));
    undistort(videoFrame, videoFrameUndistorted, cameraMatrix, dist);
    _videoFrameUndistorted = videoFrameUndistorted.clone();
    line(videoFrameUndistorted, perspectiveSrc[0], perspectiveSrc[1], Scalar(0,0,255), 2);
    line(videoFrameUndistorted, perspectiveSrc[1], perspectiveSrc[3], Scalar(0,0,255), 2);
    line(videoFrameUndistorted, perspectiveSrc[3], perspectiveSrc[2], Scalar(0,0,255), 2);
    line(videoFrameUndistorted, perspectiveSrc[2], perspectiveSrc[0], Scalar(0,0,255), 2);
    circle(videoFrameUndistorted, perspectiveSrc[0], 6, Scalar(0,0,255), FILLED);
    circle(videoFrameUndistorted, perspectiveSrc[1], 6, Scalar(0,0,255), FILLED);
    circle(videoFrameUndistorted, perspectiveSrc[2], 6, Scalar(0,0,255), FILLED);
    circle(videoFrameUndistorted, perspectiveSrc[3], 6, Scalar(0,0,255), FILLED);

    //To warp the image.
    warpPerspective(_videoFrameUndistorted, videoFramePerspective, perspectiveMatrix, videoSize);

    //Applying lane detection algorithm
    laneDetection LaneAlgo(_videoFrameUndistorted, perspectiveMatrix);
    LaneAlgo.laneDetctAlgo("este");

    Mat warpEdge; warpEdge = LaneAlgo.getWarpEdgeDetectResult().clone();
    Mat imageRedChannel; imageRedChannel = LaneAlgo.getRedChannel().clone();
    Mat redBinary; redBinary = LaneAlgo.getRedBinary().clone();
    Mat mergeImage; mergeImage = LaneAlgo.getMergeImage().clone();
    Mat histImage; histImage = LaneAlgo.getHistImage().clone();
    Mat maskImage; maskImage = LaneAlgo.getMaskImage().clone();
    Mat warpMask; warpMask = LaneAlgo.getWarpMask().clone();
    Mat finalResult; finalResult = LaneAlgo.getFinalResult().clone();

    //to create debug window
    Mat debugWindowROI;
    Mat resizePic;

    //Show the undistoted image
    debugWindowROI = debugWindow(Rect(640, 0, 320, 180));
    resize(videoFrameUndistorted, resizePic, Size(320,180));
    debugWindowROI = Scalar(0, 0, 0);
    addWeighted(debugWindowROI, 0, resizePic, 1, 0, debugWindowROI);

    //Show the perspective view
    debugWindowROI = debugWindow(Rect(960, 0, 320, 180));
    resize(videoFramePerspective, resizePic, Size(320,180));
    debugWindowROI = Scalar(0, 0, 0);
    addWeighted(debugWindowROI, 0, resizePic, 1, 0, debugWindowROI);

    //Show the red channel
    debugWindowROI = debugWindow(Rect(0, 180, 320, 180));
    imshow("imageRedChannel",fun.ResizeImage(imageRedChannel,0.5));
    resize(imageRedChannel, resizePic, Size(320,180));
    cvtColor(resizePic, resizePic, COLOR_GRAY2BGR);
    debugWindowROI = Scalar(0, 0, 0);
    addWeighted(debugWindowROI, 0, resizePic, 1, 0, debugWindowROI);

    //Show the canny edge detection result
    debugWindowROI = debugWindow(Rect(320, 180, 320, 180));
      imshow("warpEdge",fun.ResizeImage(warpEdge,0.5));
    resize(warpEdge, resizePic, Size(320,180));
    cvtColor(resizePic, resizePic, COLOR_GRAY2BGR);
    debugWindowROI = Scalar(0, 0, 0);
    addWeighted(debugWindowROI, 0, resizePic, 1, 0, debugWindowROI);

    //Show the thresholding red channel image
    debugWindowROI = debugWindow(Rect(640, 180, 320, 180));
    resize(redBinary, resizePic, Size(320,180));
    cvtColor(resizePic, resizePic, COLOR_GRAY2BGR);
    debugWindowROI = Scalar(0, 0, 0);
    addWeighted(debugWindowROI, 0, resizePic, 1, 0, debugWindowROI);

    //Show the merged image
    debugWindowROI = debugWindow(Rect(960, 180, 320, 180));
     imshow("mrgeImage",fun.ResizeImage(mergeImage,0.5));
    resize(mergeImage, resizePic, Size(320,180));
    debugWindowROI = Scalar(0, 0, 0);
    addWeighted(debugWindowROI, 0, resizePic, 1, 0, debugWindowROI);

    //Show histogram
    debugWindowROI = debugWindow(Rect(0, 360, 320, 180));
    imshow("histImage",fun.ResizeImage(histImage,0.5));
    resize(histImage, resizePic, Size(320,180));
    debugWindowROI = Scalar(0, 0, 0);
    addWeighted(debugWindowROI, 0, resizePic, 1, 0, debugWindowROI);

    //Show mask
    debugWindowROI = debugWindow(Rect(320, 360, 320, 180));
    //imshow("maskImage",fun.ResizeImage(maskImage,0.5));
    resize(maskImage, resizePic, Size(320,180));
    debugWindowROI = Scalar(0, 0, 0);
    addWeighted(debugWindowROI, 0, resizePic, 1, 0, debugWindowROI);

    //Show warp mask
    debugWindowROI = debugWindow(Rect(640, 360, 320, 180));
    resize(warpMask, resizePic, Size(320,180));
    debugWindowROI = Scalar(0, 0, 0);
    addWeighted(debugWindowROI, 0, resizePic, 1, 0, debugWindowROI);

    //show final result
    debugWindowROI = debugWindow(Rect(960, 360, 320, 180));
    imshow("finalResult",finalResult);
    resize(finalResult, resizePic, Size(320,180));
    debugWindowROI = Scalar(0, 0, 0);
    addWeighted(debugWindowROI, 0, resizePic, 1, 0, debugWindowROI);

    //Draw lines to separte the images
    line(debugWindow,Point2f(320,0),Point2f(320,539),Scalar(0,150,255),1.8);
    line(debugWindow,Point2f(640,0),Point2f(640,539),Scalar(0,150,255),1.8);
    line(debugWindow,Point2f(960,0),Point2f(960,539),Scalar(0,150,255),1.8);
    line(debugWindow,Point2f(0,180),Point2f(1279,180),Scalar(0,150,255),1.8);
    line(debugWindow,Point2f(0,360),Point2f(1279,360),Scalar(0,150,255),1.8);

    //namedWindow("DEBUG", WINDOW_AUTOSIZE);
    //imshow("DEBUG", debugWindow);


}


/***********************************************************
*
*
***********************************************************/
