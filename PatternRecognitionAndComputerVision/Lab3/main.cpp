#include <opencv2/opencv.hpp>
#include <iostream>
#include <filesystem>

using namespace cv;
using namespace std;
namespace fs = std::filesystem;

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
            cap >> frame;
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
    GaussianBlur(diff, diff, Size(9,9), 0);
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
        bitwise_or(combined, frames[i], combined);
    }

    return combined;
}

Mat cleanMask(Mat mask){
    Mat cleanedMask;
    morphologyEx(mask, cleanedMask, MORPH_OPEN, getStructuringElement(MORPH_ELLIPSE, Size(3,3)));
    morphologyEx(cleanedMask, cleanedMask, MORPH_CLOSE, getStructuringElement(MORPH_ELLIPSE, Size(7,7)));
    dilate(cleanedMask, cleanedMask, getStructuringElement(MORPH_ELLIPSE, Size(7,7)), Point(-1,-1), 1);
    return cleanedMask;
}

vector<vector<Point>> findMovingObjContours(Mat frame, double minArea = 2000.0){
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

int findClosest(const Point2f& pt, const vector<Point2f>& centers, double maxDist = 50.0) {
    int idx = -1;
    double minDist = maxDist;
    for (int i = 0; i < centers.size(); ++i) {
        double d = norm(pt - centers[i]);
        if (d < minDist) {
            minDist = d;
            idx = i;
        }
    }
    return idx;
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

struct movingObj
{
    int id;
    Point2f center;
    Point2f prevCenter;
    Rect boundingBox;
    double area;
    vector<Point2f> trajectory;
    double speed;
    double averageSpeed;
    Scalar color;
};

vector<movingObj> initObjects(const vector<Point2f>& centers) {
    vector<movingObj> objs;
    int idCounter = 0;
    for (auto& c : centers) {
        movingObj obj;
        obj.id = idCounter++;
        obj.center = c;
        obj.prevCenter = c;
        obj.speed = 0;
        obj.averageSpeed = 0;
        obj.trajectory.push_back(c);
        obj.color = Scalar(rand()%256, rand()%256, rand()%256);
        objs.push_back(obj);
    }
    return objs;
}

void updateObjects(vector<movingObj>& objs, const vector<Point2f>& centers) {
    vector<bool> matched(centers.size(), false);

    for (auto& obj : objs) {
        int idx = findClosest(obj.center, centers);
        if (idx != -1) {
            obj.prevCenter = obj.center;
            obj.center = centers[idx];
            obj.trajectory.push_back(centers[idx]);
            obj.speed = norm(obj.center - obj.prevCenter);
            obj.averageSpeed = (obj.averageSpeed * (obj.trajectory.size() - 2) + obj.speed) / (obj.trajectory.size() - 1);
            matched[idx] = true;
        }
    }

    int nextID = objs.size();
    for (int j = 0; j < centers.size(); ++j) {
        if (!matched[j]) {
            movingObj newObj;
            newObj.id = nextID++;
            newObj.center = centers[j];
            newObj.prevCenter = centers[j];
            newObj.speed = 0;
            newObj.averageSpeed = 0;
            newObj.trajectory.push_back(centers[j]);
            newObj.color = Scalar(rand()%256, rand()%256, rand()%256);
            objs.push_back(newObj);
        }
    }
}

void removeOverlappingObjects(vector<movingObj>& objs, double minDist = 10.0) {
    for (size_t i = 0; i < objs.size(); ++i) {
        for (size_t j = i + 1; j < objs.size(); ) {
            if (norm(objs[i].center - objs[j].center) < minDist) {
                objs.erase(objs.begin() + j);
            } else {
                ++j;
            }
        }
    }
}

void drawTracking(Mat& frame, const vector<movingObj>& objs) {
    for (auto& obj : objs) {
        if (obj.trajectory.size() > 1) {
            for (int t = 1; t < obj.trajectory.size(); ++t) {
                line(frame, obj.trajectory[t-1], obj.trajectory[t], obj.color, 2);
            }
        }
        string label = "ID:" + to_string(obj.id) + 
                       " Speed:" + to_string(cvRound(obj.speed)) + 
                       " avg:" + to_string(cvRound(obj.averageSpeed)) + "px/frame";

        putText(frame, label, obj.center + Point2f(10, -10), FONT_HERSHEY_SIMPLEX, 0.5, obj.color, 1);
        circle(frame, obj.center, 5, obj.color, -1);
    }
}

int main() {
    VideoCapture cap("Traffic.mp4");
    if (!cap.isOpened()) {
        cout << "Cant open video!" << endl;
        return -1;
    } 

    vector<string> filenames = saveFrames(cap, 100);
    vector<Mat> frames;

    for (auto& f : filenames) {
        frames.push_back(openImage(f, false, true));
    }
    
    vector<movingObj> objects;
    bool initialized = false;
    Ptr<BackgroundSubtractor> bg = createBackgroundSubtractorMOG2(200,20, false);

    for (int i = 1; i < frames.size(); ++i) {
        Mat diff = calcFrameDiff(frames[i-1], frames[i]);
        namedWindow("Diff", WINDOW_NORMAL);
        imshow("Diff",diff);

        
        Mat fgMask;
        bg->apply(frames[i], fgMask);
        inRange(fgMask, Scalar(200), Scalar(255), fgMask);

        Mat mask = cleanMask(diff);

        namedWindow("Mask", WINDOW_NORMAL);
        imshow("Mask",mask);
        vector<vector<Point>> contours = findMovingObjContours(mask);
        vector<Point2f> centers = getContourCenters(contours);
        if (!initialized) {
            objects = initObjects(centers);
            initialized = true;
        } else {
            updateObjects(objects, centers);
            removeOverlappingObjects(objects, 15.0);
        }

        Mat frameColor = openImage(filenames[i], false, false);
        drawTracking(frameColor, objects);
        namedWindow("Tracking", WINDOW_NORMAL);
        imshow("Tracking", frameColor);
        if (waitKey(0) == 27) break;
    }

    destroyAllWindows();
    return 0;
}
