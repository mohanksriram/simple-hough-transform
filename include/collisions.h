#include <opencv2/opencv.hpp>


struct Line
{
    /// Line of the form Ax1 + Bx2 = C
    float a, b, c;
};


bool doesCollide(Line l1, Line l2);

void getCollisionPoint(Line l1, Line l2, cv::Point& collider);