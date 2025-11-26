#include <iostream>
#include <vector>
#include <cmath>
#include <iomanip>
#include <opencv2/opencv.hpp>

using namespace std;

const int W = 800;
const int H = 800;
const int SCALE = 40; 
const int OFFSET_X = 50; 
const int OFFSET_Y = 50;

struct PointMath {
    double x, y;
};

// P0[8, 4], P1[3, 16], P2[15, 9], P3[1, 9], P4[9, 1], P5[14, 2]
const vector<PointMath> CONTROL_POINTS = {
    {8, 4}, {3, 16}, {15, 9}, {1, 9}, {9, 1}, {14, 2}
};

cv::Point toScreen(PointMath p) {
    return cv::Point(
        (int)(p.x * SCALE) + OFFSET_X,
        H - ((int)(p.y * SCALE) + OFFSET_Y)
    );
}

unsigned long long factorial(int n) {
    if (n <= 1) return 1;
    unsigned long long res = 1;
    for (int i = 2; i <= n; i++) res *= i;
    return res;
}

unsigned long long binomialCoeff(int n, int k) {
    return factorial(n) / (factorial(k) * factorial(n - k));
}

double bernstein(int i, int n, double t) {
    return binomialCoeff(n, i) * pow(t, i) * pow(1.0 - t, n - i);
}

PointMath calculateBezierPoint(double t, const vector<PointMath>& points) {
    PointMath p = {0, 0};
    int n = points.size() - 1; 

    for (int i = 0; i <= n; i++) {
        double b = bernstein(i, n, t);
        p.x += points[i].x * b;
        p.y += points[i].y * b;
    }
    return p;
}

void drawGrid(cv::Mat& img) {
    img = cv::Scalar(245, 245, 245); 
    
    for (int i = 0; i <= 20; i++) {
        cv::line(img, toScreen({(double)i, 0}), toScreen({(double)i, 20}), cv::Scalar(220, 220, 220), 1);
        cv::line(img, toScreen({0, (double)i}), toScreen({20, (double)i}), cv::Scalar(220, 220, 220), 1);
        
        cv::putText(img, to_string(i), toScreen({(double)i, -0.5}), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0,0,0), 1);
        if(i > 0) cv::putText(img, to_string(i), toScreen({-0.8, (double)i}), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0,0,0), 1);
    }

    cv::line(img, toScreen({0, 0}), toScreen({20, 0}), cv::Scalar(0, 0, 0), 2); // X
    cv::line(img, toScreen({0, 0}), toScreen({0, 20}), cv::Scalar(0, 0, 0), 2); // Y
}

int main() {
    cv::Mat img(H, W, CV_8UC3);
    drawGrid(img);

    for (size_t i = 0; i < CONTROL_POINTS.size() - 1; i++) {
        cv::line(img, toScreen(CONTROL_POINTS[i]), toScreen(CONTROL_POINTS[i+1]), cv::Scalar(200, 100, 100), 1, cv::LINE_AA);
    }

    for (size_t i = 0; i < CONTROL_POINTS.size(); i++) {
        cv::Point center = toScreen(CONTROL_POINTS[i]);
        cv::circle(img, center, 4, cv::Scalar(0, 0, 255), -1, cv::LINE_AA);
        cv::putText(img, "P" + to_string(i), cv::Point(center.x + 5, center.y - 5), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 150), 1);
    }

    vector<cv::Point> curvePoints;
    double step = 0.01;
    
    cout << "Calculated Points (t step = 0.05 for console brevity):" << endl;
    cout << "t\tX\tY" << endl;

    for (double t = 0; t <= 1.0 + step/2; t += step) {
        PointMath p = calculateBezierPoint(t, CONTROL_POINTS);
        curvePoints.push_back(toScreen(p));

        if (abs(fmod(t, 0.05)) < step/2 || t == 0.0 || t >= 1.0) {
            cout << fixed << setprecision(2) << t << "\t" << p.x << "\t" << p.y << endl;
        }
    }

    cv::polylines(img, curvePoints, false, cv::Scalar(0, 180, 0), 2, cv::LINE_AA);

    cv::putText(img, "Lab 5: Bezier Curve (Order 5)", cv::Point(20, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 0), 2);

    cv::imshow("Bezier Curve", img);
    cv::waitKey(0);

    return 0;
}