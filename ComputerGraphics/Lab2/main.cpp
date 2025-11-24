#include <iostream>
#include <vector>
#include <cmath>
#include <string>
#include <iomanip>
#include <opencv2/opencv.hpp>

using namespace std;

const double PI = M_PI;
const int W = 800;
const int H = 800;
const int SCALE = 40; 

struct Point2D {
    double x, y;
};

double toDegrees(double rad) { return rad * 180.0 / PI; }
double toRadians(double deg) { return deg * PI / 180.0; }

cv::Point toScreen(Point2D p, int offsetX = 0, int offsetY = 0) {
    return cv::Point(W/2 + p.x * SCALE + offsetX, H/2 - p.y * SCALE + offsetY);
}

void drawAxes(cv::Mat &img, std::string title) {
    img = cv::Scalar(255, 255, 255);
    cv::line(img, cv::Point(0, H/2), cv::Point(W, H/2), cv::Scalar(200, 200, 200), 1); // X axis
    cv::line(img, cv::Point(W/2, 0), cv::Point(W/2, H), cv::Scalar(200, 200, 200), 1); // Y axis
    
    for(int i=0; i<W; i+=SCALE) cv::line(img, cv::Point(i, 0), cv::Point(i, H), cv::Scalar(240,240,240), 1);
    for(int i=0; i<H; i+=SCALE) cv::line(img, cv::Point(0, i), cv::Point(W, i), cv::Scalar(240,240,240), 1);
    cv::line(img, cv::Point(0, H/2), cv::Point(W, H/2), cv::Scalar(0, 0, 0), 2);
    cv::line(img, cv::Point(W/2, 0), cv::Point(W/2, H), cv::Scalar(0, 0, 0), 2);
    cv::arrowedLine(img, cv::Point(W/2, H/2), cv::Point(W-10, H/2), cv::Scalar(0,0,0), 2);
    cv::arrowedLine(img, cv::Point(W/2, H/2), cv::Point(W/2, 10), cv::Scalar(0,0,0), 2);
    cv::putText(img, title, cv::Point(20, 40), cv::FONT_HERSHEY_DUPLEX, 0.8, cv::Scalar(0,0,0), 1, cv::LINE_AA);
}

void drawPoint(cv::Mat &img, Point2D p, string label, cv::Scalar color) {
    cv::Point sp = toScreen(p);
    cv::circle(img, sp, 6, color, -1, cv::LINE_AA);
    cv::putText(img, label, cv::Point(sp.x + 10, sp.y - 10), cv::FONT_HERSHEY_DUPLEX, 0.6, cv::Scalar(0,0,0), 1, cv::LINE_AA);
}

void Task_A() {
    Point2D E = {3, -2};
    Point2D SymE = {E.x, -E.y}; 
    
    double r = sqrt(SymE.x * SymE.x + SymE.y * SymE.y);
    double phi = atan2(SymE.y, SymE.x);

    cout << "\n--- Task A (Symmetry) ---" << endl;
    cout << "E(" << E.x << ", " << E.y << ") -> SymE(" << SymE.x << ", " << SymE.y << ")" << endl;
    cout << "Polar SymE: r=" << r << ", phi=" << toDegrees(phi) << " deg" << endl;

    cv::Mat img(H, W, CV_8UC3);
    drawAxes(img, "Task A: Symmetry relative to Polar Axis (X)");
    drawPoint(img, E, "E(3, -2)", cv::Scalar(200, 0, 0));
    drawPoint(img, SymE, "SymE(3, 2)", cv::Scalar(0, 0, 200));
    
    cv::line(img, toScreen(E), toScreen(SymE), cv::Scalar(100,100,100), 1, cv::LINE_AA);
    cv::imshow("Task A", img);
}

void Task_B() {
    Point2D A = {0, 0};
    Point2D B = {5, -3};
    double k = 4.0; 

    cout << "\n--- Task B (Scaling x4) ---" << endl;
    cout << "Old A(0,0) -> New A(" << A.x/k << ", " << A.y/k << ")" << endl;
    cout << "Old B(5,-3) -> New B(" << B.x/k << ", " << B.y/k << ")" << endl;

    cv::Mat img(H, W, CV_8UC3);
    drawAxes(img, "Task B: Scaling (Unit length x4)");

    drawPoint(img, A, "A(0,0)", cv::Scalar(0,0,0));
    drawPoint(img, B, "B(5,-3)", cv::Scalar(255,0,0));

    string txt = "New Coord of B: (" + to_string(B.x/k).substr(0,4) + ", " + to_string(B.y/k).substr(0,4) + ")";
    cv::putText(img, txt, cv::Point(20, H-40), cv::FONT_HERSHEY_DUPLEX, 0.7, cv::Scalar(0,0,150), 1, cv::LINE_AA);
    cv::imshow("Task B", img);
}

void Task_C() {
    Point2D D = {-sqrt(3.0), 1.0}; // x = -1.732, y = 1
    double axisRotDeg = 45.0;
    double axisRotRad = toRadians(axisRotDeg);

    double r = sqrt(D.x*D.x + D.y*D.y);
    double phi_std = atan2(D.y, D.x); // Standard angle
    double phi_new = phi_std - axisRotRad; // Angle relative to new axis

    cout << "\n--- Task C (Rotated Polar Axis) ---" << endl;
    cout << "D(" << D.x << ", " << D.y << ")" << endl;
    cout << "Std Polar: r=" << r << ", phi=" << toDegrees(phi_std) << " deg" << endl;
    cout << "New Polar (axis +45deg): r=" << r << ", phi=" << toDegrees(phi_new) << " deg" << endl;

    cv::Mat img(H, W, CV_8UC3);
    drawAxes(img, "Task C: Rotated Polar Axis (+45 deg)");

    cv::Point center = toScreen({0,0});
    cv::Point axisEnd = toScreen({cos(axisRotRad)*8, sin(axisRotRad)*8});
    cv::arrowedLine(img, center, axisEnd, cv::Scalar(0,150,0), 2, cv::LINE_AA);
    cv::putText(img, "Polar Axis'", cv::Point(axisEnd.x+10, axisEnd.y), cv::FONT_HERSHEY_DUPLEX, 0.6, cv::Scalar(0,150,0), 1, cv::LINE_AA);
    drawPoint(img, D, "D(-sqrt(3), 1)", cv::Scalar(0,0,255));
    cv::line(img, center, toScreen(D), cv::Scalar(0,0,255), 1, cv::LINE_AA);
    cv::imshow("Task C", img);
}

void Task_D() {
    double h[] = {6, 4, 6, 4}; // x, y, z, w
    Point2D affine = {h[0]/h[3], h[1]/h[3]};
    double z_affine = h[2]/h[3];

    cout << "\n--- Task D (Homogeneous -> Affine) ---" << endl;
    cout << "H(6,4,6,4) -> Affine(" << affine.x << ", " << affine.y << ", " << z_affine << ")" << endl;

    cv::Mat img(H, W, CV_8UC3);
    drawAxes(img, "Task D: Homogeneous(6,4,6,4) -> Affine");
    
    drawPoint(img, affine, "P(1.5, 1.0)", cv::Scalar(255,0,255));
    cv::putText(img, "Z-coord: 1.5", cv::Point(toScreen(affine).x, toScreen(affine).y+30), cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar(100,100,100), 1, cv::LINE_AA);
    cv::imshow("Task D", img);
}

void Task_E() {
    Point2D A = {3, -5};
    Point2D B = {2, -9};
    
    // Vector BA
    double dx = A.x - B.x; // 1
    double dy = A.y - B.y; // 4
    double lenBA = sqrt(dx*dx + dy*dy);
    double angle = atan2(dy, dx);
    
    cout << "\n--- Task E (Affine Transform) ---" << endl;
    cout << "Origin -> B(2,-9). X-axis aligns with BA." << endl;
    cout << "BA Vector: (" << dx << ", " << dy << "), Length: " << lenBA << endl;
    cout << "Rotation Angle: " << toDegrees(angle) << " deg" << endl;
    
    // Step 1: Initial State
    cv::Mat img1(H, W, CV_8UC3);
    drawAxes(img1, "Task E (Step 1): Initial Points A & B");
    int viewOffset = -200;

    drawPoint(img1, A, "A(3, -5)", cv::Scalar(255,0,0));
    drawPoint(img1, B, "B(2, -9)", cv::Scalar(0,0,255));
    cv::line(img1, toScreen(B), toScreen(A), cv::Scalar(200,200,200), 1, cv::LINE_AA);
    
    // Step 2: New Origin at B
    cv::Mat img2 = img1.clone();
    cv::rectangle(img2, cv::Point(0,0), cv::Point(W,60), cv::Scalar(255,255,255), -1); // Clear title
    cv::putText(img2, "Task E (Step 2): Origin shifted to B", cv::Point(20, 40), cv::FONT_HERSHEY_DUPLEX, 0.8, cv::Scalar(0,0,0), 1, cv::LINE_AA);
    cv::Point screenB = toScreen(B);
    cv::line(img2, cv::Point(screenB.x-100, screenB.y), cv::Point(screenB.x+100, screenB.y), cv::Scalar(0,150,150), 1, cv::LINE_AA); // New X'
    cv::line(img2, cv::Point(screenB.x, screenB.y-100), cv::Point(screenB.x, screenB.y+100), cv::Scalar(0,150,150), 1, cv::LINE_AA); // New Y'
    
    // Step 3: Rotation
    cv::Mat img3 = img2.clone();
    cv::rectangle(img3, cv::Point(0,0), cv::Point(W,60), cv::Scalar(255,255,255), -1);
    cv::putText(img3, "Task E (Step 3): Axes Rotated along AB", cv::Point(20, 40), cv::FONT_HERSHEY_DUPLEX, 0.8, cv::Scalar(0,0,0), 1, cv::LINE_AA);
    // Draw Final Axes aligned with BA
    cv::Point axisEnd = toScreen({B.x + cos(angle)*4, B.y + sin(angle)*4});
    cv::arrowedLine(img3, screenB, axisEnd, cv::Scalar(0,180,0), 2, cv::LINE_AA); // Final X axis
    cv::putText(img3, "New X Axis", cv::Point(axisEnd.x, axisEnd.y-10), cv::FONT_HERSHEY_DUPLEX, 0.6, cv::Scalar(0,180,0), 1, cv::LINE_AA);

    cv::imshow("Task E - 1. Initial", img1);
    cv::imshow("Task E - 2. Shift", img2);
    cv::imshow("Task E - 3. Rotate", img3);
}

int main() {
    Task_A();
    Task_B();
    Task_C();
    Task_D();
    Task_E();

    cout << "\nPress any key in window to exit..." << endl;
    cv::waitKey(0);
    return 0;
}