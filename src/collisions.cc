#include <collisions.h>


bool doesCollide(Line l1, Line l2) {
    // Find the determinent
    float determt = l1.a*l2.b - l2.a*l1.b;
    std::cout << "does collide: " << determt << std::endl;
    return (determt <= -0.01 || determt >= 0.01);
};

void getCollisionPoint(Line l1, Line l2, cv::Point& collider) {
    float determt = l1.a*l2.b - l2.a*l1.b;

    std::cout << "collider: " << determt << " l1: " << l1.a << ", " << l1.b<< ", " << l1.c << std::endl;
    std::cout << "collider: " << determt << " l2: " << l2.a << ", " << l2.b<< ", " << l2.c << std::endl;

    collider.x = (l1.c*l2.b - l2.c*l1.b)/determt;
    collider.y = (l2.c*l1.a - l1.c*l2.a)/determt;

    std::cout << "collider x: " << collider.x << std::endl;
    std::cout << "collider y: " << collider.y << std::endl;
};