#include <stdio.h>
#include <filters.h>
#include <collisions.h>
#include <cmath>
#define PI 3.14159265

struct HoughRes {
    cv::Mat H;
    std::vector<float> rho_scale;
    std::vector<float> theta_scale;
};

struct HoughLineRes {
    int num_lines;
    std::vector<float> rhos;
    std::vector<float> thetas;
};

struct HoughElement {
    float value;
    float rho, theta;
};


void myHoughLineSegmentsv3(float line_rho, float line_theta, cv::Mat& img) {
    // create a line
    float xmax = img.cols;
    float ymax = img.rows;

    Line hough_line;
    hough_line.a = std::cos(line_theta*PI/180);
    hough_line.b = std::sin(line_theta*PI/180);
    hough_line.c = line_rho;

    std::vector<Line> lines;

    std::vector<cv::Point> collision_points;
    if(hough_line.a >= -0.001) {
        if(hough_line.a <= 0.001) {

        }
    }

    if(hough_line.a >= -0.001 && hough_line.a <= 0.001) {
        // its of the form y = c;
        if(0 <= hough_line.b <= ymax-1) {
            cv::Point vertical_point;
            vertical_point.y = hough_line.c;
            vertical_point.x = 0;
            collision_points.push_back(vertical_point);
            vertical_point.x = xmax-1;
            collision_points.push_back(vertical_point);
        }

    } else if (hough_line.b >= -0.001 && hough_line.b <= 0.001) {
        if(0 <= hough_line.a <= ymax-1) {
            cv::Point horiz_point;
            horiz_point.x = hough_line.c;
            horiz_point.y = 0;
            collision_points.push_back(horiz_point);
            horiz_point.y = ymax-1;
            collision_points.push_back(horiz_point);
        }
    } else {
        // other image boundary lines
        Line top;
        // 0.x + 1.y = 0
        top.a = 0;
        top.b = 1;
        top.c = 0;

        lines.push_back(top);

        Line bottom;
        // 0.x + 1.y = ymax-1
        bottom.a = 0; 
        bottom.b = 1;
        bottom.c = ymax-1;
        lines.push_back(bottom);

        Line left;
        // 1.x + 0.y = 0
        left.a = 1;
        left.b = 0;
        left.c = 0;
        lines.push_back(left);

        Line right;
        // 1.x + 0.y = xmax-1
        right.a = 1;
        right.b = 0;
        right.c = xmax-1;
        lines.push_back(right);

        for(const Line& line: lines) {
            if(collision_points.size() != 2) {
                // Check for collision
                if(doesCollide(line, hough_line)) {
                    cv::Point collider;
                    getCollisionPoint(line, hough_line, collider);
                    collision_points.push_back(collider);
                }
            } else {
                break;
            }
        }
    }

    // Assuming two colliding points are always found
    if(collision_points.size() == 2) {
        cv::line(img, collision_points[0], collision_points[1], cv::Scalar((0, 0, 255)), 20);
        std::cout << "current collision_points size: " << collision_points.size() << std::endl;
        std::cout << collision_points[0] << std::endl;
        std::cout << collision_points[1] << std::endl;
    } else {
        std::cout << "current collision_points size: " << collision_points.size() << std::endl;
        exit(0);
    }

}


void myHoughLineSegmentsv2(float line_rho, float line_theta, cv::Mat& img) {

    std::cout << "in: line_theta " << line_theta << std::endl;
    cv::Point bottom;
    cv::Point top;

    int xmax = img.cols;
    int ymax = img.rows;

    float a = 0;
    float b = 0;
    if(line_theta == 90) {
        a = xmax-1;
    } else {
        a = line_rho/(std::cos(line_theta*PI/180));
    }
    
    if(line_theta == 0 || line_theta == 180) {
        b = 0;
    } else {
        b = line_rho/(std::sin(line_theta*PI/180));
    }

    std::cout << "a: " << a << "b: " << b << std::endl;


    bottom.x = 0;
    bottom.y = ymax-1-b;
    

    top.x = a;
    top.y = ymax-1;

    std::cout << "top point: " << top << std::endl;
    std::cout << "bottom point: " << bottom << std::endl;

    cv::line(img, bottom, top, cv::Scalar((0, 0, 255)), 20);
}

void myHoughLineSegments(float line_rho, float line_theta, cv::Mat& img) {

    cv::Point bottom;
    cv::Point top;

    float ymax = img.rows;
    float xmax = img.cols;

    float a = std::cos(line_theta*PI/180);
    float b = std::sin(line_theta*PI/180);
    float x0 = line_rho*a;
    float y0 = line_rho*b;

    // Find bottom point
    float y = ymax-1;
    float x = x0 - ((y - y0)/a)*b;

    if(x<0) {
        x = 0;
        y = y0 + ((x0 - x)/b)*a;
    } else if(x >= xmax) {
        x = xmax-1;
        y = y0 + ((x0 - x)/b)*a;
    }

    bottom.x = cvRound(x);
    bottom.y = cvRound(y);

    // Find top point
    float top_x = 0;
    float top_y = y0 + ((x0 - top_x)/b)*a;

    if(top_y >= ymax) {
        top_y = ymax-1;
        top_x = x0 - ((top_y - y0)/a)*b;
    } else if(top_y < 0) {
        top_y = 0;
        top_x = x0 - ((top_y - y0)/a)*b;
    }

    top.x = cvRound(top_x);
    top.y = cvRound(top_y);

    std::cout << "top point: " << top << std::endl;
    std::cout << "bottom point: " << bottom << std::endl;

    cv::line(img, top, bottom, cv::Scalar(0, 0, 255), 20);
}

void myHoughTransform(cv::Mat& img, int threshold, 
                    int rho_res, int theta_res, HoughRes& res) {
        
        // Threshold the edge image
        img.setTo(0, img < threshold);
        cv::imwrite("thresholded_image.jpg", img);
        int count1 = 0;
        int count2 = 0;
        // Initialize the accumulator
        res.H = cv::Mat::zeros(180/theta_res, 800/rho_res, CV_32F);
        std::cout << "H rows: " << res.H.rows << "H cols: " << res.H.cols << std::endl;

        // Start accumulation
        int n_rows = img.rows;
        int n_cols = img.cols;
        
        for(int i=0; i<n_rows; i++) {
            for(int j=0; j<n_cols; j++) {
                for(int theta=0; theta<=180; theta+=theta_res) {
                    if(img.at<float>(i, j) > 0) {
                        count1++;
                        // count vote
                        float rho = j*std::cos(theta*PI/180) + 
                                    i*std::sin(theta*PI/180);
                        
                        if(rho > 0) {
                            count2++;
                            int new_rho = round(rho/rho_res);
                            //res.H.at<float>(new_rho, theta/theta_res) += 1;
                            res.H.at<float>(theta/theta_res, new_rho) += 1;
                            res.rho_scale.push_back(rho);
                            res.theta_scale.push_back(theta);
                        }
                    }
                }
            }
        }

        std::cout << "Overall rhos: " << count1 << "valid rhos: " << count2 << std::endl;

        std::cout << "Hough accumulator: " << res.H << std::endl;
        cv::imwrite("hough_accumulator.jpg", res.H.t());
    }
//Finding lines
bool compareWith(cv::Mat H, int i, int j, unsigned char neighbors) { 
    
    bool result = true;
    
    if(neighbors & 1) {
        //east
        if(H.at<float>(i, j) < H.at<float>(i, j+1)) {
            result = false;
            return result;
        }
    }
    if(neighbors & 2) {
        //south
        if(H.at<float>(i, j) < H.at<float>(i+1, j)) {
            result = false;
            return result;
        }
    }
    if(neighbors & 4) {
        //north
        if(H.at<float>(i, j) < H.at<float>(i-1, j)) {
            result = false;
            return result;
        }
    } 
    if (neighbors & 8) {
        //west
        if(H.at<float>(i, j) < H.at<float>(i, j-1)) {
            result = false;
            return result;
        }
    }
     if(neighbors & 16) {
        //northeast
        if(H.at<float>(i, j) < H.at<float>(i-1, j+1)) {
            result = false;
            return result;
        }
    } 
    if(neighbors & 32) {
         //northwest
        if(H.at<float>(i, j) < H.at<float>(i-1, j-1)) {
            result = false;
            return result;
        }
        
    }
     if(neighbors & 64) {
        //southeast
        if(H.at<float>(i, j) < H.at<float>(i+1, j+1)) {
            result = false;
            return result;
        }
    }
     if(neighbors & 128) {
         //southwest
        if(H.at<float>(i, j) < H.at<float>(i+1, j-1)) {
            result = false;
            return result;
        }
    }
    return result;
}

void myHoughLines(cv::Mat& H, int& n_lines, std::vector<int>& rhos, std::vector<int>& thetas) {
    int n_rows = H.rows;
    int n_cols = H.cols;
    int line_count = 0;
    std::cout << "H rows: " << n_rows << "H cols: " << n_cols << std::endl;

    bool does_line_count = false;

    struct CustomCompare
    {
        bool operator()(const HoughElement& lhs, const HoughElement& rhs)
        {
            return lhs.value < rhs.value;
        }
    };


    std::priority_queue<HoughElement, std::vector<HoughElement>, CustomCompare> H_accumulator;


    for(int i=0; i<n_rows; i++) {
        for(int j=0; j<n_cols; j++) {
            if(i==0) {
                if(j==0) {
                    does_line_count = compareWith(H, i, j, (unsigned char) 67);
                } else if(j==n_cols-1) {
                    does_line_count = compareWith(H, i, j, (unsigned char) 138);
                }
                else {
                    does_line_count = compareWith(H, i, j, (unsigned char) 203);
                }
            } else if(i==n_rows-1) {
                if(j==0) {
                    does_line_count = compareWith(H, i, j, (unsigned char) 21);
                } else if(j==n_cols-1) {
                    does_line_count = compareWith(H, i, j, (unsigned char) 44);
                } else {
                    does_line_count = compareWith(H, i, j, (unsigned char) 63);
                }
            } else if(j==0) {
                does_line_count = compareWith(H, i, j, (unsigned char) 87);
            } else if(j==n_cols-1) {
                does_line_count = compareWith(H, i, j, (unsigned char) 174);
            } else {
                does_line_count = compareWith(H, i, j, (unsigned char) 255);
            }
            if(does_line_count) {
                    HoughElement newh;
                    newh.value = H.at<float>(i, j);
                    newh.theta = i;
                    newh.rho = j;
                    H_accumulator.push(newh);
                    line_count += 1;
                    // if(line_count == n_lines) {
                    //     break;
                    // } else {
                    //     std::cout << line_count << "not equal to " << n_lines << std::endl;
                    // }
            } else {
                H.at<float>(i, j) = 0;
            }
        }
    }

    for(int i=0; i<n_lines; i++) {
        HoughElement highest = H_accumulator.top();
        rhos.push_back(highest.rho);
        thetas.push_back(highest.theta);
        H_accumulator.pop();
        std::cout << "cur highest rho: " << highest.rho << " theta: " << highest.theta << std::endl;  
    }

    cv::imwrite("nms_hough_accumulator.jpg", H.t());
    std::cout << "found: " << n_lines << "hough lines" << std::endl;
}
int main(int argc, char** argv )
{
    // if ( argc != 2 )
    // {
    //     printf("usage: DisplayImage.out <Image_Path>\n");
    //     return -1;
    // }

    cv::Mat image;
    image = cv::imread( argv[1], 0);

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
   cv::Mat final1, final2, final3;
   myEdgeFilter(image, 1/3, final1, false);
   myEdgeFilter(image, 1/3, final2);
   final2.copyTo(final3);
   final3.setTo(0, final2<10);
   
   
   
   //std::cout << final2 << std::endl;
   HoughRes res;
   //final2.setTo(0, final2 < 3); // TODO:- Why is the threshold value so low?
   //std::cout << final1 << std::endl;
   int RHO_THRES = 15;
   int THETA_THRES = 15;
   int INTENSITY_THRES = 10;

   myHoughTransform(final2, INTENSITY_THRES, RHO_THRES, THETA_THRES, res);

   std::vector<int> rhos, thetas;
   int line_count = 20;
   myHoughLines(res.H, line_count, rhos, thetas);
   
   for(int i=0; i<line_count; i++) {
       float actual_rho = (rhos[i])*RHO_THRES;
       float actual_theta = (thetas[i])*THETA_THRES;
       myHoughLineSegmentsv3(actual_rho, actual_theta, final2);
       std::cout << "rhos: " << actual_rho << "thetas: " << actual_theta << std::endl;
   }


   std::cout << "all rhos size: " << rhos.size() << std::endl;

   
   cv::namedWindow("Display Image", cv::WINDOW_AUTOSIZE );
   cv::imshow("Image1", final1);
   cv::namedWindow("Display Image", cv::WINDOW_AUTOSIZE );
   cv::imshow("Image2", final2);
   cv::namedWindow("Display Image", cv::WINDOW_AUTOSIZE );
   cv::imshow("Image3", final3);
   
//    cv::namedWindow("Display Image", cv::WINDOW_NORMAL );
//    cv::imshow("Image3", res.H);

   cv::waitKey(0);
    return 0;
}
