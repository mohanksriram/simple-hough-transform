#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>


void myImageFilter(cv::Mat& img0, cv::Mat& kernel, cv::Mat& result);

void myEdgeFilter(cv::Mat& img0, double sigma, cv::Mat& final, bool perform_nms=true);

