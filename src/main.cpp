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

funciones fun;
using namespace std;
using namespace cv;

//static void cannyCallBack(int, void *);


cv::Mat image;
cv::Mat gray,dst;
cv::Mat canny_edges;
int canny_low_threshold = 1;

std::string PATH = "/home/y3rsn/Dev/cpp/CV_2022/Roadtraking/img/road_img.png";
std::string PATH2 = "/home/y3rsn/Dev/cpp/CV_2022/Roadtraking/video/flydrone.mp4";


int main(int argc, char **argv)
{

    Mat image = imread(PATH,IMREAD_GRAYSCALE);

//    imshow("this image",image);

    fun.ReadVideo();

    //dst.create(image.size(), image.type());
    //cvtColor(image, gray, cv::COLOR_BGR2GRAY);

    //dst.create(image.size(), image.type());

    // imagen en escala de grises
    //cvtColor(image, gray, cv::COLOR_BGR2GRAY);

    //cv::namedWindow("Canny", 0);

    // Crear ventanas ajustables
    //cv::createTrackbar("Valor del parámetro:", "Canny", &canny_low_threshold, 255, cannyCallBack);

    // Llamar a la función de devolución de llamada
    //cannyCallBack(0, 0);

    //waitKey(0);
    //destroyAllWindows();
    return 0;
}


void cannyCallBack(int, void *)
{
    // Primero usa un kernel 3 × 3 para reducir el ruido
   blur(gray, canny_edges, cv::Size(3, 3));

    cv::Mat out;

    // Llamar al operador de Canny
    Canny(canny_edges, out, canny_low_threshold, canny_low_threshold * 3, 3);
    dst = cv::Scalar::all(0);
    image.copyTo(dst, out);

    // Representación de pantalla
    imshow("Canny", out);
}




