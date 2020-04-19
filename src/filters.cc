#include "filters.h"
#define PI 3.14159265


void myImageFilter(cv::Mat& img0, cv::Mat& kernel, cv::Mat& result) {
    int BORDER_SIZE = 1;
    //cv::copyMakeBorder(img0, img0, BORDER_SIZE, BORDER_SIZE, BORDER_SIZE, BORDER_SIZE, cv::BORDER_REPLICATE);
    std::cout << result.size << std::endl;
    int ddepth=-1;
    /// Apply filter
    cv::filter2D(img0, result, ddepth , kernel);
}


void myEdgeFilter(cv::Mat& img0, double sigma, cv::Mat& final, bool perform_nms) {
    int hsize = 2*int(std::ceil(3*sigma)) + 1;
    cv::Mat kernel = cv::getGaussianKernel(hsize, sigma);
    //cv::Mat kernel_x;

    //kernel = kernel*kernel.t();

    std::cout << "hsize: " << hsize << "Gaussian Kernel: " << kernel << std::endl;
    
    int kernel_size = 3;
    float vertical_lines[3][3] = {{1, 0, -1},
                        {2, 0, -2,},
                        {1, 0, -1}};

    float horizontal_lines[3][3] = {{1, 2, 1},
                        {0, 0, 0,},
                        {-1, -2, -1}};

    cv:: Mat kernel_x = cv::Mat(kernel_size, kernel_size, CV_32F, horizontal_lines)/(float)(kernel_size*kernel_size);
    cv:: Mat kernel_y = cv::Mat(kernel_size, kernel_size, CV_32F, vertical_lines)/(float)(kernel_size*kernel_size);

    cv::Mat blurred, imgx, imgy;

    myImageFilter(img0, kernel, blurred);
    //cv::Sobel(blurred, imgx, 1, 1, hsize);
    //cv::Sobel(blurred, imgy, 1, 1, hsize);
    myImageFilter(blurred, kernel_x, imgx);
    myImageFilter(blurred, kernel_y, imgy);

    std::cout << "img: " << img0.depth() << "imgx: " << imgx.depth() << "imgy: " << imgy.depth() << std::endl;

    cv::Mat direction;
    imgx.convertTo(imgx, CV_32F);
    imgy.convertTo(imgy, CV_32F);
    cv::phase(imgx, imgy, direction, true);
    cv::Mat imgz;
    
    imgx.convertTo(imgz, CV_32F);
    for(int i=0; i<imgx.rows; i++) {
        for(int j=0; j<imgy.cols; j++) {
            float pixel_x = imgx.at<float>(i, j);
            float pixel_y = imgy.at<float>(i, j);
            imgz.at<float>(i, j) = std::sqrt(pow(pixel_x,2) + pow(pixel_y,2));
        }
    }


    int n_rows = direction.rows;
    int n_cols = direction.cols;
    std::cout << "direction size: " << direction.size() << "img0 size: " << img0.size() << std::endl;
    std::cout << "rows: " << n_rows << "cols: " << n_cols << std::endl;
    std::cout << "imgx size: " << imgx.size << "imgy size: " << imgy.size << std::endl;
    
    std::cout << "direction type" << direction.type() << std::endl;
    int max_val = 0;
    for(int i=0; i<n_rows; i++) {
        for(int j=0; j<n_cols; j++) {
           float pixel_x = imgx.at<float>(i, j);
           float pixel_y = imgy.at<float>(i, j);
           float grad_dir = std::atan(pixel_y/pixel_x)*180/PI;
            if(pixel_x == 0) {
                if(pixel_y == 0) {
                    grad_dir = 0;
                } else {
                    grad_dir = 90;
                }
            }
            else if(grad_dir > 0) {
                std::cout<<grad_dir<<std::endl;
            }

           int rounded_grad = round(grad_dir/45);
           direction.at<float>(i, j) = (rounded_grad%4)*45;
           //int rounded_val = round(direction.at<float>(i,j)/45);
           //std::cout << "rounded val: " << rounded_val << std::endl;
           //direction.at<float>(i,j) = (rounded_val%4)*45;
           max_val = std::max((rounded_grad%4)*45, max_val);
            if(max_val > 3) {
            //std::cout << "max_dir: " << direction.at<float>(i,j) << std::endl;
            }
        }
    }
    //std::cout << "direction: " << direction << std:: endl;

    imgz.convertTo(final, CV_32F);

    //NMS
    if(perform_nms) {
        for(int i=0; i<n_rows; i++) {
            for(int j=0; j<n_cols; j++) {
                float cur_dir = direction.at<float>(i, j);
                float cur_pixel = imgz.at<float>(i, j);
                if(cur_pixel > 0) {
                    if(cur_dir == 0) {
                        if(j>0 && j<n_cols-1) {
                            if(imgz.at<float>(i, j-1) > cur_pixel || imgz.at<float>(i, j+1) > cur_pixel) {
                                final.at<float>(i, j) = 0;
                            }
                        }
                    } else if (cur_dir == 45) {
                        if(j>0 && j<n_cols-1 && i>0 && i<n_rows-1) {
                            if(imgz.at<float>(i-1, j-1) > cur_pixel || imgz.at<float>(i+1, j+1) > cur_pixel) {
                                final.at<float>(i, j) = 0;
                            }
                        }
                    } else if (cur_dir == 135) {
                        if(j>0 && j<n_cols-1 && i>0 && i<n_rows-1) {
                            if(imgz.at<float>(i-1, j+1) > cur_pixel || imgz.at<float>(i+1, j-1) > cur_pixel) {
                                final.at<float>(i, j) = 0;
                            }
                        }
                    } else if (cur_dir == 90) {
                        if(i>0 && i<n_rows-1) {
                            if(imgz.at<float>(i, j-1) > cur_pixel || imgz.at<float>(i, j+1) > cur_pixel) {
                                final.at<float>(i, j) = 0;
                            }
                        }
                    }
                }
            }
        }
    }

    //std::cout << direction << std::endl;
    std::cout << "max val: " << max_val << std::endl;
    //direction = ceil((direction/45));
    //std::cout << "direction: " << direction << std::endl;

    cv::imwrite("blurred.jpg", blurred);
    cv::imwrite("horizontal_lines.jpg", imgx);
    cv::imwrite("vertical_lines.jpg", imgy);
    cv::imwrite("edges.jpg", imgx+imgy);
    cv::imwrite("edges2.jpg", imgz);
    cv::imwrite("direction.jpg", direction);
    cv::imwrite("nms.jpg", final);
}
