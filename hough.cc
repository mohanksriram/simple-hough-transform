#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <cmath>
#define PI 3.14159265

struct HoughRes {
    cv::Mat H;
    std::vector<float> rho_scale;
    std::vector<float> theta_scale;
};

void myPointProcessor(cv::Mat& img0, cv::Mat& kernel, cv::Mat& result) {
    int BORDER_SIZE = 1;
    std::cout << img0.size << std::endl;
    cv::copyMakeBorder(img0, img0, BORDER_SIZE, BORDER_SIZE, BORDER_SIZE, BORDER_SIZE, cv::BORDER_REPLICATE);
    result = img0;
    std::cout << result.size << std::endl;
}

void myImageFilter(cv::Mat& img0, cv::Mat& kernel, cv::Mat& result) {
    int BORDER_SIZE = 1;
    //cv::copyMakeBorder(img0, img0, BORDER_SIZE, BORDER_SIZE, BORDER_SIZE, BORDER_SIZE, cv::BORDER_REPLICATE);
    std::cout << result.size << std::endl;
    int ddepth=-1;
    /// Apply filter
    cv::filter2D(img0, result, ddepth , kernel);
}

void myHoughTransform(cv::Mat img, int threshold, 
                    float rho_res, float theta_res, HoughRes& res) {
        
        // Threshold the edge image
        img.setTo(0,img < threshold);
        
        // Initialize the accumulator
        res.H = cv::Mat::zeros(180/theta_res, 800/rho_res, CV_32F);

        // Start accumulation
        int n_rows = img.rows;
        int n_cols = img.cols;
        
        for(int i=0; i<n_rows; i++) {
            for(int j=0; j<n_cols; j++) {
                for(int theta=0; theta<=180; theta+=theta_res) {
                    if(img.at<float>(i, j) > 0) {
                        // count vote
                        float rho = j*std::cos(theta*PI/180) + 
                                    i*std::sin(theta*PI/180);
                        
                        int rounded_val = round(rho/rho_res);
                        int new_rho = (rounded_val%int(rho_res));
                        res.H.at<float>(theta, new_rho) += 1;
                        
                        res.rho_scale.push_back(new_rho);
                        res.theta_scale.push_back(theta);
                    }
                }
            }
        }

        std::cout << "Hough accumulator: " << res.H << std::endl;
    }
//Finding lines
void myHoughLines(cv::Mat& H, int n_lines, std::vector<int> rhos, std::vector<int> thetas) {
    int n_rows = H.rows;
    int n_cols = H.cols;
    int line_count = 0;

    for(int i=0; i<n_rows; i++) {
        for(int j=0; j<n_cols; j++) {
            if(i>0 && j>0 && i<n_rows-1 && j<n_cols-1) {
                if((H.at<float>(i, j) > H.at<float>(i-1, j)) &&
                    (H.at<float>(i, j) > H.at<float>(i+1, j)) &&
                    (H.at<float>(i, j) > H.at<float>(i, j-1)) &&
                    (H.at<float>(i, j) > H.at<float>(i-1, j+1)) &&
                    (H.at<float>(i, j) > H.at<float>(i-1, j-1)) &&
                    (H.at<float>(i, j) > H.at<float>(i-1, j+1)) &&
                    (H.at<float>(i, j) > H.at<float>(i+1, j-1)) &&
                    (H.at<float>(i, j) > H.at<float>(i+1, j+1))
                ) {
                    rhos.push_back(i);
                    thetas.push_back(j);
                    line_count += 1;
                    if(line_count == n_lines) {
                        break;
                    }
                }
            }
        }
    }
}

void myEdgeFilter(cv::Mat& img0, double sigma, cv::Mat& final, bool perform_nms=true) {
    int hsize = 10;//2*int(std::ceil(3*sigma)) + 1;
    cv::Mat kernel = cv::getGaussianKernel(hsize, sigma);

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
    std::cout << "direction size: " << direction.size << "img0 size: " << img0.size() << std::endl;
    std::cout << "imgx size: " << imgx.size << "imgy size: " << imgy.size << std::endl;
    
    std::cout << "direction type" << direction.type() << std::endl;
    
    int max_val = 0;
    for(int i=0; i<n_rows; i++) {
        for(int j=0; j<n_cols; j++) {
           int rounded_val = round(direction.at<float>(i,j)/45);
           //std::cout << "rounded val: " << rounded_val << std::endl;
           direction.at<float>(i,j) = (rounded_val%4)*45;
           max_val = std::max(rounded_val%4, max_val);
           if(max_val == 3) {
               //std::cout << "max_dir: " << direction.at<uchar>(i,j) << std::endl;
           }
        }
    }
    imgz.convertTo(final, CV_32F);

    //NMS
    if(perform_nms) {
        for(int i=0; i<n_rows; i++) {
            for(int j=0; j<n_cols; j++) {
                float cur_dir = direction.at<float>(i, j);
                float cur_pixel = imgz.at<float>(i, j);
                if(cur_dir == 0) {
                    if(j>0 && j<n_cols-1) {
                        if(imgz.at<float>(i, j-1) > cur_pixel || imgz.at<float>(i, j+1) < cur_pixel) {
                            final.at<float>(i, j) = 0;
                        }
                    }
                } else if (cur_dir == 45) {
                    if(j>0 && j<n_cols-1 && i>0 && i<n_rows-1) {
                        if(imgz.at<float>(i-1, j+1) > cur_pixel || imgz.at<float>(i+1, j-1) < cur_pixel) {
                            final.at<float>(i, j) = 0;
                        }
                    }
                } else if (cur_dir == 135) {
                    if(j>0 && j<n_cols-1 && i>0 && i<n_rows-1) {
                        if(imgz.at<float>(i-1, j-1) > cur_pixel || imgz.at<float>(i+1, j+1) < cur_pixel) {
                            final.at<float>(i, j) = 0;
                        }
                    }
                } else if (cur_dir == 90) {
                    if(i>0 && i<n_rows-1) {
                        if(imgz.at<float>(i, j-1) > cur_pixel || imgz.at<float>(i, j+1) < cur_pixel) {
                            final.at<float>(i, j) = 0;
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
}

int main(int argc, char** argv )
{
    // if ( argc != 2 )
    // {
    //     printf("usage: DisplayImage.out <Image_Path>\n");
    //     return -1;
    // }

    cv::Mat image;
    image = cv::imread( "../img01.jpg", 0);

    if ( !image.data )
    {
        printf("No image data \n");
        return -1;
    }

    cv::Mat result1, result2;

    /// Update kernel size for a normalized box filter
    int kernel_size = 3;
    cv:: Mat kernel = cv::Mat::ones( kernel_size, kernel_size, CV_32F )/ (float)(kernel_size*kernel_size);

    float vertical_lines[3][3] = {{1, 0, -1},
                        {2, 0, -2,},
                        {1, 0, -1}};

    float horizontal_lines[3][3] = {{1, 2, 1},
                        {0, 0, 0,},
                        {-1, -2, -1}};

    cv:: Mat custom_kernel1 = cv::Mat(kernel_size, kernel_size, CV_32F, horizontal_lines)/(float)(kernel_size*kernel_size);
    cv:: Mat custom_kernel2 = cv::Mat(kernel_size, kernel_size, CV_32F, vertical_lines)/(float)(kernel_size*kernel_size);

    myImageFilter(image, custom_kernel1, result1);
    myImageFilter(image, custom_kernel2, result2);

    cv::Mat final_res = result1 + result2;
    /*
    cv::namedWindow("Display Image", cv::WINDOW_AUTOSIZE );
    cv::imshow("Image1", image);
    
    cv::namedWindow("Display Image", cv::WINDOW_AUTOSIZE );
    cv::imshow("Image2", result1);

    cv::namedWindow("Display Image", cv::WINDOW_AUTOSIZE );
    cv::imshow("Image3", result2);

    cv::namedWindow("Display Image", cv::WINDOW_AUTOSIZE );
    cv::imshow("Image3", final_res);
    cv::waitKey(0);
    */
   cv::Mat final1, final2;
   myEdgeFilter(image, 10/3, final1, false);
   myEdgeFilter(image, 10/3, final2);
   
   HoughRes res;
   final2.setTo(0, final2 < 3); // TODO:- Why is the threshold value so low?
   //std::cout << final1 << std::endl;
   myHoughTransform(final2, 3, 20, 12, res);

   std::vector<int> rhos, thetas;
   myHoughLines(res.H, 2, rhos, thetas);

   std::cout << "all rhos size: " << rhos.size() << std::endl;


   cv::namedWindow("Display Image", cv::WINDOW_AUTOSIZE );
   cv::imshow("Image1", final1);
   cv::namedWindow("Display Image", cv::WINDOW_AUTOSIZE );
   cv::imshow("Image2", final2);

   cv::waitKey(0);
    return 0;
}
