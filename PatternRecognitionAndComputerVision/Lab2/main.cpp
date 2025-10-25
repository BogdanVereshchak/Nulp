#include <opencv2/opencv.hpp>
#include <iostream>
#include <filesystem>

using namespace cv;
using namespace std;
namespace fs = std::filesystem;

struct ImageInfo {
    int width;
    int height;
    int channels;
};

ImageInfo getImageInfo(const Mat& img) {
    ImageInfo info;
    info.width = img.cols;
    info.height = img.rows;
    info.channels = img.channels();
    return info;
}

vector<string> saveFrames(VideoCapture& cap, int count = 5, const string& prefix = "frame") {
    vector<string> filenames;
    Mat frame;
    for (int i = 0; i < count; ++i) {
        string filename = prefix + to_string(i) + ".jpg";

        if (!fs::exists(filename)) {
            cap >> frame;
            if (frame.empty()) break;

            imwrite(filename, frame);
            cout << "Saved " << filename << endl;
        } else {
            cout << "Frame already exists: " << filename << endl;
        }

        filenames.push_back(filename);
    }
    return filenames;
}

Mat openImage(const string& filename, bool open = true, bool grayscale = false) { 
    Mat img = grayscale ? imread(filename, IMREAD_GRAYSCALE) : imread(filename);
    
    if (img.empty()) {
        cerr << "Cant open image: " << filename << endl;
    } else if (open){
        // Відобразити кадр
        namedWindow("Image", WINDOW_NORMAL);
        imshow("Image", img);
        waitKey(0);
    }
    return img;
}

Mat calcFrameDiff(const Mat& frame1, const Mat& frame2, bool useThreshold = true, int threshVal = 10){
    Mat diff;
    absdiff(frame1, frame2, diff);    
    if (useThreshold){
        threshold(diff,diff, threshVal, 255, THRESH_BINARY);
    }
    
    return diff;
}

Mat combineFrameDiffs(vector<Mat> frames){
    if (frames.empty()) return Mat();

    Mat combined = frames[0].clone();
    for (size_t i = 1; i < frames.size(); ++i) {
        GaussianBlur(combined, combined, Size(3,3),0);
        threshold(combined,combined, 100, 255, THRESH_BINARY);
        bitwise_or(combined, frames[i], combined);
    }

    return combined;
}

vector<vector<Point>> findMovingObjContours(Mat frame, double minArea = 500.0){
    vector<vector<Point>> contours;
    findContours(frame, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    vector<vector<Point>> filtered;
    for (const auto& c : contours) {
        if (contourArea(c) > minArea) {
            filtered.push_back(c);
        }
    }
    return filtered;
} 

vector<Point2f> getContourCenters(const vector<vector<Point>>& contours) {
    vector<Point2f> centers;
    for (const auto& c : contours) {
        Moments m = moments(c);
        if (m.m00 != 0) {
            centers.push_back(Point2f(float(m.m10/m.m00), float(m.m01/m.m00)));
        }
    }
    return centers;
}

Mat drawContoursAndCenters(const Mat& frame, const vector<vector<Point>>& contours, const vector<Point2f>& centers, bool isGray = true) {
    Mat output;
    if (isGray)
        cvtColor(frame, output, COLOR_GRAY2BGR);
    else
        output = frame.clone();

    drawContours(output, contours, -1, Scalar(0,0,255), 2);
    for (const auto& c : contours) {
        Rect bbox = boundingRect(c);
        rectangle(output, bbox, Scalar(0,255,0), 2);
    }
    for (const auto& center : centers) {
        circle(output, center, 5, Scalar(255,0,0), -1);
    }
    return output;
}

Mat cleanMask(Mat mask){
    Mat cleanedMask;
    morphologyEx(mask, cleanedMask, MORPH_OPEN, getStructuringElement(MORPH_ELLIPSE, Size(5,5)));
    morphologyEx(cleanedMask, cleanedMask, MORPH_CLOSE, getStructuringElement(MORPH_ELLIPSE, Size(5,5)));
    return cleanedMask;
}

int main() {
    VideoCapture cap("Traffic.mp4");
    if (!cap.isOpened()) {
        cout << "Cant open video!" << endl;
        return -1;
    }

    namedWindow("Video", WINDOW_NORMAL); 
    resizeWindow("Video", 800, 600); 

    Mat frame;
    while (true) {
        cap >> frame;
        if (frame.empty()) break;

        imshow("Video", frame);
        if (waitKey(30) == 27)
            break;
    }

    cap.set(CAP_PROP_POS_FRAMES, 0);

    vector<string> filenames = saveFrames(cap, 10);
    vector<Mat> frames;
    vector<Mat> frameDiffs;

    for (int i = 0; i < 5; i++){
        frames.push_back(openImage(filenames[i], false, true));
    }

    for (int i = 1; i < frames.size(); ++i) {
        frameDiffs.push_back(calcFrameDiff(frames[i-1], frames[i], true));
    }

    // for (size_t i = 0; i < frameDiffs.size(); ++i) {
    //     namedWindow("Frame Diff", WINDOW_NORMAL);
    //     imshow("Frame Diff", frameDiffs[i]);
    //     waitKey(0); // Adjust delay as needed
    // }

    Mat motionMask = combineFrameDiffs(frameDiffs);
    motionMask = cleanMask(motionMask);
    namedWindow("Combined Diff", WINDOW_NORMAL);
    imshow("Combined Diff", motionMask);
    waitKey(0);

    vector<vector<Point>> contours = findMovingObjContours(motionMask);
    vector<Point2f> centers = getContourCenters(contours);
    Mat background = openImage(filenames[0], false, false);
    Mat result = drawContoursAndCenters(background, contours, centers, false);
    namedWindow("Motion Detection", WINDOW_NORMAL);
    imshow("Motion Detection", result);
    waitKey(0);

    destroyAllWindows();
    return 0;
}
