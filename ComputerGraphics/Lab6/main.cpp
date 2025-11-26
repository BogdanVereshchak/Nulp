#include <iostream>
#include <vector>
#include <cmath>
#include <opencv2/opencv.hpp>

using namespace std;

const int W = 1200;
const int H = 1000;
const double PI = 3.14159265358979323846;

double windOffset = 0.0;
double windSpeed = 0.0;
double windTarget = 0.0;
double windMax = 1.0;
double windChangeSpeed = 0.02;

struct Point2D {
    double x, y;
};

class AffineTransformer {
public:
    static Point2D scale(Point2D p, Point2D center, double sx, double sy) {
        return {
            center.x + (p.x - center.x) * sx,
            center.y + (p.y - center.y) * sy
        };
    }

    static Point2D translate(Point2D p, double dx, double dy) {
        return { p.x + dx, p.y + dy };
    }
    
    static Point2D rotate(Point2D p, Point2D center, double angleRad) {
        double tx = p.x - center.x;
        double ty = p.y - center.y;
        return {
            center.x + tx * cos(angleRad) - ty * sin(angleRad),
            center.y + tx * sin(angleRad) + ty * cos(angleRad)
        };
    }
};

int main() {
    cv::Mat img(H, W, CV_8UC3);

    vector<Point2D> domeBase;
    int segments = 20;
    double radius = 100.0;
    
    Point2D localOrigin = {0, 0}; 

    for (int i = 0; i <= segments; i++) {
        double angle = PI + (double)i / segments * PI; 
        domeBase.push_back({
            radius * cos(angle),
            radius * sin(angle) 
        });
    }

    double posX = W / 2.0;
    double posY = 100.0;
    double openFactor = 0.1;
    double fallSpeed = 2.0;
    bool autoMode = false;
    bool opening = true; 

    cout << "Parachute Simulation" << endl;
    cout << "Controls:" << endl;
    cout << " [A] - Toggle Auto/Manual Mode" << endl;
    cout << " [SPACE] - Reset Position" << endl;
    cout << " Manual Mode:" << endl;
    cout << "   [O]   - Open Parachute" << endl;
    cout << "   [C] - Close Parachute" << endl;
    cout << "   [L/R] - Wind (Move)" << endl;
    cout << " [ESC] - Exit" << endl;

    while (true) {
        img = cv::Scalar(255, 200, 135); 

        if (autoMode) {
            posY += fallSpeed;
            if (posY > H + 150) posY = -150;

            if (posY > H / 3 && openFactor < 1.0) openFactor += 0.01;
            if (posY < 0) openFactor = 0.1; 
            
            fallSpeed = 5.0 - (openFactor * 4.0);   
        }

        int key = cv::waitKey(1);
        if (key == 27) break; 
        if (key == 'a' || key == 'A') autoMode = !autoMode;
        if (key == 32) { posY = 100; openFactor = 0.1; } 
        
        if (!autoMode) {
            if (key == 'o' || key == 'O') { 
                if (openFactor < 1.0) openFactor += 0.05;
            }
            if (key == 'c' || key == 'C') { 
                if (openFactor > 0.1) openFactor -= 0.05;
            }
            if (key == 'l' || key == 'L') posX -= 5; 
            if (key == 'r' || key == 'R') posX += 5; 
  
            fallSpeed = 5.0 - (openFactor * 4.0);
            posY += fallSpeed;
            if (posY > H + 150) posY = -150;
        }

        vector<cv::Point> drawPoints;
        Point2D currentCenter = {posX, posY};

        double cordLength = 120.0;
        Point2D loadPos = {posX, posY + cordLength}; 

        if (!autoMode || true) { 
            if (rand() % 100 == 0) {
                windTarget = ((rand() % 200) - 100) / 100.0 * windMax;
            }
            windSpeed += (windTarget - windSpeed) * 0.01;

            windOffset += windSpeed;
            for (int i = 0; i < drawPoints.size(); i++) {
                double factor = (double)i / drawPoints.size();
                drawPoints[i].x += (windSpeed * (0.5 + factor)); 
                drawPoints[i].y += sin(posY / 30.0 + factor * 2.0) * 2.0; 
            }

            posX += windSpeed;
        }

        for (const auto& p : domeBase) {
            Point2D transformed = AffineTransformer::scale(p, localOrigin, openFactor, 1.0 - (openFactor * 0.3));
            transformed = AffineTransformer::translate(transformed, posX, posY);
            drawPoints.push_back(cv::Point((int)transformed.x, (int)transformed.y));
        }

        cv::rectangle(img, cv::Point(loadPos.x - 10, loadPos.y), cv::Point(loadPos.x + 10, loadPos.y + 30), cv::Scalar(50, 50, 50), -1);
        
        if (!drawPoints.empty()) {
            cv::line(img, drawPoints[0], cv::Point(loadPos.x, loadPos.y), cv::Scalar(100, 100, 100), 1);
            cv::line(img, drawPoints[drawPoints.size()/2], cv::Point(loadPos.x, loadPos.y), cv::Scalar(100, 100, 100), 1);
            cv::line(img, drawPoints.back(), cv::Point(loadPos.x, loadPos.y), cv::Scalar(100, 100, 100), 1);
        }

        vector<vector<cv::Point>> contours;
        contours.push_back(drawPoints);
        cv::fillPoly(img, contours, cv::Scalar(0, 0, 200)); 
        cv::polylines(img, drawPoints, false, cv::Scalar(0, 0, 100), 2, cv::LINE_AA);

        string modeStr = autoMode ? "AUTO" : "MANUAL";
        cv::putText(img, "Mode: " + modeStr, cv::Point(20, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0,0,0), 2);
        
        cv::putText(img, "Affine Parameters:", cv::Point(20, 60), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,0,0), 1);
        string s_scale = "Scale X (Openness): " + to_string(openFactor).substr(0, 4);
        string s_trans = "Translation Y (Alt): " + to_string((int)posY);
        string s_speed = "Speed Y: " + to_string((double)fallSpeed).substr(0, 4);
        cv::putText(img, s_scale, cv::Point(30, 80), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,0,0), 1);
        cv::putText(img, s_trans, cv::Point(30, 100), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,0,0), 1);
        cv::putText(img, s_speed, cv::Point(30, 120), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,0,0), 1);

        cv::imshow("Lab 6: Parachute Animation", img);
    }

    return 0;
}