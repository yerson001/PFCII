//class functions
#include<opencv2/calib3d.hpp>
#include<opencv2/core/types.hpp>
#include<opencv2/core/persistence.hpp>
#include<opencv2/imgcodecs.hpp>
#include<opencv2/imgproc.hpp>
#include<opencv2/highgui.hpp>
#include<opencv2/core/types_c.h>
#include<iostream>
using namespace cv;
using namespace std;

class funciones{

 public:

    void ReadVideo()
    {
       Mat temp,temp1;
        VideoCapture cap("/home/y3rsn/Dev/cpp/CV_2022/Roadtraking/video/flydrone.mp4");
        while(1){
           Mat frame;
           cap >> frame;
           if (frame.empty())
             break;
           temp = bordes(frame,62);
           temp1 = dilatacion(temp,1.5);
           imshow( "temp", temp1);
           imshow( "Frame", frame);
           char c=(char)waitKey(25);
           if(c==27)
             break;
         }
         cap.release();
         destroyAllWindows();
    }

    Mat ResizeImage(Mat &image, float scale)
    {
        Mat resized;
        resize(image, resized, Size(image.cols*scale, image.rows*scale), INTER_LINEAR);
        return resized;
    }

    Mat bordes(Mat image,int level)
    {
       Mat gray,canny,dst,out;
       cvtColor(image,gray,COLOR_BGR2GRAY);
       blur(gray,canny,cv::Size(3,3));

       Canny(canny,out,level,level*3,3);
       dst.create(image.size(),image.type());
       return out;
    }

    Mat dilatacion(Mat image ,int dilation_size)
    {
       Mat out,dilation_dst;
       int dilation_type = MORPH_RECT;

       Mat element = getStructuringElement(
             dilation_type,Size(2*dilation_size+1,2*dilation_size+1),
             Point(dilation_size,dilation_size)
             );
       dilate(image,out,element);
      return out;
    }
};


