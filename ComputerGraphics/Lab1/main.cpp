#include <iostream>
#include <sstream>
#include <iomanip>
#include <string>
#include <cstdint>
#include <cmath>
#include <opencv2/opencv.hpp>

using namespace std;

const string colorName = "blanchedalmond";
const char hexCode[] = "#FFEBCD";

struct RGB {
    uint8_t r;
    uint8_t g;
    uint8_t b;
};

// RGB to Delphi TColor (0x00BBGGRR)
uint32_t RGBToTColor(const RGB &c) {
    return (uint32_t(c.b) << 16) | (uint32_t(c.g) << 8) | uint32_t(c.r);
}

string RGBToHex(const RGB &c) {
    std::ostringstream ss;
    ss << '#' << std::uppercase << std::hex
       << std::setw(2) << std::setfill('0') << int(c.r)
       << std::setw(2) << std::setfill('0') << int(c.g)
       << std::setw(2) << std::setfill('0') << int(c.b);
    return ss.str();
}

RGB ApplyMatrixTransform(const RGB &in) {
    const double m[3][3] = {
        {0.412, 0.35,   0.1804},
        {0.21,  0.715,  0.072},
        {0.0193,0.1191, 0.8202}
    };

    double R = in.r;
    double G = in.g;
    double B = in.b;

    double Rp = m[0][0]*R + m[0][1]*G + m[0][2]*B;
    double Gp = m[1][0]*R + m[1][1]*G + m[1][2]*B;
    double Bp = m[2][0]*R + m[2][1]*G + m[2][2]*B;

    auto clamp_and_round = [](double v)->uint8_t {
        v = std::round(v);
        if (v < 0.0) return 0;
        if (v > 255.0) return 255;
        return static_cast<uint8_t>(v);
    };

    return RGB{ clamp_and_round(Rp), clamp_and_round(Gp), clamp_and_round(Bp) };
}

int main() {
    RGB original{ 255, 235, 205 }; // blanchedalmond

    uint32_t tcolor_orig = RGBToTColor(original);

    RGB transformed = ApplyMatrixTransform(original);
    uint32_t tcolor_trans = RGBToTColor(transformed);

    // Output to console
    cout << "Color: " << colorName << " (" << hexCode << ")\n";
    cout << "Original RGB: (" << int(original.r) << ", " << int(original.g) << ", " << int(original.b) << ")\n";
    cout << "Original Hex: " << RGBToHex(original) << '\n';
    cout << "Original TColor (Hex): 0x" << uppercase << hex << setw(8) << setfill('0') << tcolor_orig << dec << "\n";
    cout << "Original TColor (Dec): " << tcolor_orig << "\n\n";

    cout << "Matrix Transform Result:\n";
    cout << "Transformed RGB: (" << int(transformed.r) << ", " << int(transformed.g) << ", " << int(transformed.b) << ")\n";
    cout << "Transformed Hex: " << RGBToHex(transformed) << '\n';
    cout << "Transformed TColor (Hex): 0x" << uppercase << hex << setw(8) << setfill('0') << tcolor_trans << dec << "\n";
    cout << "Transformed TColor (Dec): " << tcolor_trans << "\n";

    // Visualization
    const int H = 400;
    const int W = 600;
    cv::Mat img(H, W, CV_8UC3, cv::Scalar(50, 50, 50));

    cv::Scalar origColorBGR(original.b, original.g, original.r);
    cv::Scalar transColorBGR(transformed.b, transformed.g, transformed.r);

    cv::rectangle(img, cv::Rect(50, 50, 200, 200), origColorBGR, cv::FILLED);
    cv::rectangle(img, cv::Rect(350, 50, 200, 200), transColorBGR, cv::FILLED);

    int font = cv::FONT_HERSHEY_SIMPLEX;
    double scale = 0.5;
    cv::Scalar white(255, 255, 255);

    cv::putText(img, "ORIGINAL", cv::Point(50, 40), font, 0.7, white, 1);
    cv::putText(img, "R:255 G:235 B:205", cv::Point(50, 280), font, scale, white, 1);
    cv::putText(img, "TColor: " + to_string(tcolor_orig), cv::Point(50, 300), font, scale, white, 1);

    cv::putText(img, "TRANSFORMED", cv::Point(350, 40), font, 0.7, white, 1);
    cv::putText(img, "R:" + to_string(transformed.r) + " G:" + to_string(transformed.g) + " B:" + to_string(transformed.b), cv::Point(350, 280), font, scale, white, 1);
    cv::putText(img, "TColor: " + to_string(tcolor_trans), cv::Point(350, 300), font, scale, white, 1);

    cv::imshow("Lab 1: Geometric Modeling", img);
    cv::waitKey(0);

    return 0;
}