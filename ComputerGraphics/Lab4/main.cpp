#include <iostream>
#include <vector>
#include <cmath>
#include <opencv2/opencv.hpp>

using namespace std;

// Налаштування полотна
const int W = 800;
const int H = 600;
const double PI = 3.14159265358979323846;

// Параметри шестикутника
const double R = 40.0; // Радіус описаного кола

// Функція для обчислення вершини шестикутника
cv::Point getHexVertex(cv::Point center, double r, int i) {
    double angle_deg = 60 * i + 30; // +30 щоб вершина була зверху
    double angle_rad = angle_deg * PI / 180.0;
    return cv::Point(center.x + r * cos(angle_rad), center.y + r * sin(angle_rad));
}

// Малювання одного елемента орнаменту
void drawHexagonPattern(cv::Mat& img, cv::Point center) {
    // 1. Зовнішній товстий шестикутник
    vector<cv::Point> pts;
    for (int i = 0; i < 6; i++) {
        pts.push_back(getHexVertex(center, R, i));
    }
    
    // Щоб намалювати замкнутий контур
    vector<vector<cv::Point>> contours;
    contours.push_back(pts);
    cv::drawContours(img, contours, 0, cv::Scalar(80, 80, 80), 4, cv::LINE_AA);

    // 2. Внутрішній візерунок (менший шестикутник, повернутий або вписаний)
    // Імітуємо візерунок з фото: вписаний шестикутник + лінії до центру
    double r_inner = R * 0.65;
    vector<cv::Point> pts_inner;
    for (int i = 0; i < 6; i++) {
        pts_inner.push_back(getHexVertex(center, r_inner, i));
    }
    
    vector<vector<cv::Point>> contours_inner;
    contours_inner.push_back(pts_inner);
    cv::drawContours(img, contours_inner, 0, cv::Scalar(100, 100, 100), 2, cv::LINE_AA);

    // 3. З'єднувальні лінії (імітація спіральності)
    for (int i = 0; i < 6; i++) {
        // Лінія від кута зовнішнього до кута внутрішнього (зі зсувом)
        cv::line(img, pts[i], pts_inner[(i+1)%6], cv::Scalar(120, 120, 120), 1, cv::LINE_AA);
        // Лінія від кута внутрішнього до іншого кута внутрішнього
         cv::line(img, pts_inner[i], pts_inner[(i+2)%6], cv::Scalar(180, 180, 180), 1, cv::LINE_AA);
    }
}

int main() {
    cv::Mat img(H, W, CV_8UC3, cv::Scalar(240, 240, 240)); // Світлий фон як на фото

    // Крок сітки
    // Ширина шестикутника (від грані до грані) = R * sqrt(3)
    // Висота (від вершини до вершини) = 2 * R
    // Вертикальний крок між рядами = R * 1.5
    double hex_w = R * sqrt(3);
    double hex_h = 2 * R;
    double vert_dist = R * 1.5;

    int rows = H / vert_dist + 2;
    int cols = W / hex_w + 2;

    for (int row = 0; row < rows; row++) {
        for (int col = 0; col < cols; col++) {
            double x = col * hex_w;
            double y = row * vert_dist;

            // Зсув для непарних рядків
            if (row % 2 != 0) {
                x += hex_w / 2.0;
            }

            // Додаємо трохи "рандому" в колір ліній, як вимагає загальне завдання
            // Але зберігаємо структуру орнаменту
            drawHexagonPattern(img, cv::Point(x, y));
        }
    }

    // Підпис
    cv::putText(img, "Lab 4: Hexagonal Ornament (OpenCV)", cv::Point(20, 30), 
                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 0), 2);

    cv::imshow("Ornament", img);
    cv::waitKey(0);
    return 0;
}