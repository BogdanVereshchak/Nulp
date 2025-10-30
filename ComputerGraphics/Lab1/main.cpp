#include <iostream>
#include <sstream>
#include <iomanip>
#include <string>
#include <cstdint>
#include <cmath>
#include <opencv2/opencv.hpp>

using namespace std;

// Original color: blanchedalmond = #FFEBCD (R=255, G=235, B=205)
const string colorName = "blanchedalmond";
const char hexCode[] = "#FFEBCD";

struct RGB {
    uint8_t r;
    uint8_t g;
    uint8_t b;
};

// Convert RGB -> Delphi TColor (same byte order as Windows COLORREF):
// TColor = 0x00BBGGRR (low byte = R)
uint32_t RGBToTColor(const RGB &c) {
    return (uint32_t(c.b) << 16) | (uint32_t(c.g) << 8) | uint32_t(c.r);
}

string RGBToHex(const RGB &c) {
    std::ostringstream ss;
    ss << '#' << std::uppercase << std::hex
       << int(c.r)
       << int(c.g)
       << int(c.b);
    return ss.str();
}

// Apply 3x3 matrix transform to RGB vector (matrix values given in the task).
RGB ApplyMatrixTransform(const RGB &in) {
    // matrix
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

    RGB out{ clamp_and_round(Rp), clamp_and_round(Gp), clamp_and_round(Bp) };
    return out;
}

int main() {
    // original color
    RGB original{ 255, 235, 205 };

    // compute TColor for original
    uint32_t tcolor_orig = RGBToTColor(original);

    // apply matrix transform
    RGB transformed = ApplyMatrixTransform(original);
    uint32_t tcolor_trans = RGBToTColor(transformed);

    // print results
    cout << "Color name: " << colorName << '\n';
    cout << "Original RGB: (" << int(original.r) << ", " << int(original.g) << ", " << int(original.b) << ")\n";
    cout << "Original hex: " << RGBToHex(original) << '\n';
    cout << "Original TColor (hex): 0x" << uppercase << hex << setw(8) << setfill('0') << tcolor_orig << dec << setfill(' ') << '\n';
    cout << "Original TColor (dec): " << tcolor_orig << "\n\n";

    cout << "Matrix transform result (R' G' B'):\n";
    cout << "Transformed RGB: (" << int(transformed.r) << ", " << int(transformed.g) << ", " << int(transformed.b) << ")\n";
    cout << "Transformed hex: " << RGBToHex(transformed) << '\n';
    cout << "Transformed TColor (hex): 0x" << uppercase << hex << setw(8) << setfill('0') << tcolor_trans << dec << setfill(' ') << '\n';
    cout << "Transformed TColor (dec): " << tcolor_trans << "\n";

    // --- Show colors in a window using OpenCV ---
    const int H = 200;
    const int W = 400;
    cv::Mat img(H, W, CV_8UC3, cv::Scalar(0,0,0));

    // OpenCV uses BGR order
    cv::Scalar origColorBGR(original.b, original.g, original.r);
    cv::Scalar transColorBGR(transformed.b, transformed.g, transformed.r);

    // left half: original; right half: transformed
    cv::rectangle(img, cv::Rect(0,0,W/2,H), origColorBGR, cv::FILLED);
    cv::rectangle(img, cv::Rect(W/2,0,W/2,H), transColorBGR, cv::FILLED);

    // labels
    int font = cv::FONT_HERSHEY_SIMPLEX;
    double fontScale = 0.6;
    int thickness = 1;
    cv::putText(img, "Original", cv::Point(10, H-20), font, fontScale, cv::Scalar(255,255,255), thickness);
    cv::putText(img, RGBToHex(original), cv::Point(10, H-40), font, fontScale, cv::Scalar(255,255,255), thickness);

    cv::putText(img, "Transformed", cv::Point(W/2 + 10, H-20), font, fontScale, cv::Scalar(255,255,255), thickness);
    cv::putText(img, RGBToHex(transformed), cv::Point(W/2 + 10, H-40), font, fontScale, cv::Scalar(255,255,255), thickness);

    cv::namedWindow("Colors", cv::WINDOW_NORMAL);
    cv::imshow("Colors", img);
    cout << "Press any key in the image window to exit..." << endl;
    cv::waitKey(0);
    cv::destroyAllWindows();


    return 0;
}