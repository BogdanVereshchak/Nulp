#include <iostream>
#include <vector>
#include <cmath>
#include <iomanip>
#include <string>
#include <opencv2/opencv.hpp>

using namespace std;

// Canvas settings
const int W = 800;
const int H = 600;
const cv::Point CENTER(W / 2, H / 2);

// Scaling factor: 1 mathematical unit = 80 pixels
// This is needed because sqrt(6) is ~2.45 pixels, which is too small to see.
const double SCALE = 80.0; 

// Ellipse parameters for equation x^2/6 + y^2/3 = 1
// a^2 = 6, b^2 = 3
const double A_MATH = sqrt(6.0); // approx 2.449
const double B_MATH = sqrt(3.0); // approx 1.732

// Convert math coordinates to screen coordinates
cv::Point toScreen(double x, double y) {
    return cv::Point(CENTER.x + x * SCALE, CENTER.y - y * SCALE);
}

void drawGrid(cv::Mat& img) {
    img = cv::Scalar(255, 255, 255);
    // Draw grid lines every 1 unit
    for (int i = -10; i <= 10; i++) {
        cv::Point p1 = toScreen(i, -10);
        cv::Point p2 = toScreen(i, 10);
        cv::line(img, p1, p2, cv::Scalar(240, 240, 240), 1);
        
        cv::Point p3 = toScreen(-10, i);
        cv::Point p4 = toScreen(10, i);
        cv::line(img, p3, p4, cv::Scalar(240, 240, 240), 1);
    }

    // Axes
    cv::line(img, cv::Point(0, H / 2), cv::Point(W, H / 2), cv::Scalar(0, 0, 0), 2); // X
    cv::line(img, cv::Point(W / 2, 0), cv::Point(W / 2, H), cv::Scalar(0, 0, 0), 2); // Y
    
    // Arrows
    cv::arrowedLine(img, cv::Point(W / 2, H / 2), cv::Point(W - 10, H / 2), cv::Scalar(0, 0, 0), 2);
    cv::arrowedLine(img, cv::Point(W / 2, H / 2), cv::Point(W / 2, 10), cv::Scalar(0, 0, 0), 2);
    
    // Axis Labels
    cv::putText(img, "X", cv::Point(W - 20, H / 2 - 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,0,0), 1, cv::LINE_AA);
    cv::putText(img, "Y", cv::Point(W / 2 + 10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,0,0), 1, cv::LINE_AA);
}

void solveAndDraw(cv::Mat& img) {
    // 1. Draw Ellipse: x = sqrt(6)*cos(t), y = sqrt(3)*sin(t)
    vector<cv::Point> ellipsePoints;
    for (int angle = 0; angle <= 360; angle++) {
        double rad = angle * CV_PI / 180.0;
        double x = A_MATH * cos(rad);
        double y = B_MATH * sin(rad);
        ellipsePoints.push_back(toScreen(x, y));
    }
    cv::polylines(img, ellipsePoints, true, cv::Scalar(200, 0, 0), 2, cv::LINE_AA);

    // 2. Calculate Square Sides
    // Equation: x^2/6 + y^2/3 = 1 -> a^2 = 6, b^2 = 3
    // Condition for tangent square (angle 45 deg, k = +/- 1):
    // m = sqrt(a^2 + b^2)
    double m_val = sqrt(A_MATH * A_MATH + B_MATH * B_MATH); // sqrt(6 + 3) = sqrt(9) = 3
    
    cout << "Calculated m: " << m_val << endl;

    // The square vertices for this symmetric ellipse and square will be on the axes
    // Vertices at (0, 3), (3, 0), (0, -3), (-3, 0)
    
    vector<cv::Point> squarePoints;
    squarePoints.push_back(toScreen(0, m_val));  // Top (0, 3)
    squarePoints.push_back(toScreen(m_val, 0));  // Right (3, 0)
    squarePoints.push_back(toScreen(0, -m_val)); // Bottom (0, -3)
    squarePoints.push_back(toScreen(-m_val, 0)); // Left (-3, 0)

    cv::polylines(img, squarePoints, true, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);

    // Draw Points
    auto drawPt = [&](double x, double y, string txt) {
        cv::Point p = toScreen(x, y);
        cv::circle(img, p, 4, cv::Scalar(0,0,0), -1, cv::LINE_AA);
        cv::putText(img, txt, cv::Point(p.x+5, p.y-5), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0,0,0), 1, cv::LINE_AA);
    };
    
    drawPt(A_MATH, 0, "a");
    drawPt(0, B_MATH, "b");
    drawPt(3, 0, "3");
    drawPt(0, 3, "3");

    // Labels and Info
    cv::putText(img, "Ellipse: x^2/6 + y^2/3 = 1", cv::Point(20, 30), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(200, 0, 0), 1, cv::LINE_AA);
    cv::putText(img, "Tangent Square: y = +/- x +/- 3", cv::Point(20, 60), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
    
    string params = "a=" + to_string(A_MATH).substr(0,4) + ", b=" + to_string(B_MATH).substr(0,4);
    cv::putText(img, params, cv::Point(20, 90), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(50, 50, 50), 1, cv::LINE_AA);
}

int main() {
    cv::Mat img(H, W, CV_8UC3);
    
    drawGrid(img);
    solveAndDraw(img);

    cv::imshow("Lab 3: Variant 9 (Specific)", img);
    cv::waitKey(0);
    return 0;
}