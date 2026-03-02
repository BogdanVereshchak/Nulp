#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <string>

using namespace cv;
using namespace std;

// --- КОНФІГУРАЦІЯ ---
const double PIXELS_TO_METERS = 0.07;
const double CONTOUR_MIN_AREA = 1000.0; 
const double TRUCK_AREA_THRESHOLD = 4000.0;
const double ZOOM_FACTOR = 1.05; 

struct movingObj {
    int id;
    Point2f center;
    Point2f prevCenter;
    vector<Point2f> trajectory;
    double speed;
    double area;
    Rect boundRect; 
    string type;
    Scalar color;
    int framesNotSeen;
};

/**
 * @brief Очищення маски руху.
 */
Mat cleanMaskImproved(Mat mask) {
    if (mask.empty()) return mask;
    Mat cleaned;
    medianBlur(mask, cleaned, 5); 
    Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(15, 15));
    morphologyEx(cleaned, cleaned, MORPH_CLOSE, kernel);
    Mat kernelOpen = getStructuringElement(MORPH_ELLIPSE, Size(5, 5));
    morphologyEx(cleaned, cleaned, MORPH_OPEN, kernelOpen);
    return cleaned;
}

/**
 * @brief Кроп та зум для приховування країв.
 */
Mat cropAndZoom(const Mat& img, double zoom) {
    if (img.empty()) return img;
    int h = img.rows;
    int w = img.cols;
    int cropH = cvRound(h / zoom);
    int cropW = cvRound(w / zoom);
    int x1 = (w - cropW) / 2;
    int y1 = (h - cropH) / 2;
    Rect roi(x1, y1, cropW, cropH);
    Mat cropped = img(roi).clone();
    resize(cropped, cropped, Size(w, h), 0, 0, INTER_LINEAR);
    return cropped;
}

/**
 * @brief Оновлення логіки трекінгу.
 */
void updateTracking(vector<movingObj>& objs, const vector<vector<Point>>& contours, int& nextID) {
    vector<Point2f> centers;
    for(const auto& c : contours) {
        Moments m = moments(c);
        if(m.m00 != 0) centers.push_back(Point2f(float(m.m10/m.m00), float(m.m01/m.m00)));
    }

    for (auto& obj : objs) obj.framesNotSeen++;

    for (size_t i = 0; i < centers.size(); ++i) {
        int bestIdx = -1;
        double minDist = 60.0; 
        for(int j=0; j<(int)objs.size(); ++j) {
            double d = norm(objs[j].center - centers[i]);
            if(d < minDist) { minDist = d; bestIdx = j; }
        }

        if(bestIdx != -1) {
            objs[bestIdx].prevCenter = objs[bestIdx].center;
            objs[bestIdx].center = centers[i];
            objs[bestIdx].trajectory.push_back(centers[i]);
            if(objs[bestIdx].trajectory.size() > 20) objs[bestIdx].trajectory.erase(objs[bestIdx].trajectory.begin());
            objs[bestIdx].speed = objs[bestIdx].speed * 0.7 + norm(objs[bestIdx].center - objs[bestIdx].prevCenter) * 0.3;
            objs[bestIdx].area = contourArea(contours[i]);
            objs[bestIdx].boundRect = boundingRect(contours[i]); 
            objs[bestIdx].framesNotSeen = 0;
        } else {
            movingObj n;
            n.id = nextID++;
            n.center = n.prevCenter = centers[i];
            n.trajectory.push_back(centers[i]);
            n.speed = 0; n.framesNotSeen = 0;
            n.color = Scalar(rand()%255, rand()%255, rand()%255);
            n.area = contourArea(contours[i]);
            n.boundRect = boundingRect(contours[i]);
            n.type = (n.area > TRUCK_AREA_THRESHOLD) ? "Truck" : "Car";
            objs.push_back(n);
        }
    }
    objs.erase(remove_if(objs.begin(), objs.end(), [](const movingObj& o){
        return o.framesNotSeen > 15;
    }), objs.end());
}

int main() {
    VideoCapture cap("Traffic3.mp4");
    if (!cap.isOpened()) return -1;

    Mat ref_frame, ref_grey;
    cap >> ref_frame;
    if (ref_frame.empty()) return -1;
    
    const int w = ref_frame.cols;
    const int h = ref_frame.rows;
    double fps = cap.get(CAP_PROP_FPS);
    if (fps <= 0) fps = 30.0;

    cvtColor(ref_frame, ref_grey, COLOR_BGR2GRAY);

    // Опорні точки для абсолютної фіксації
    vector<Point2f> ref_pts;
    goodFeaturesToTrack(ref_grey, ref_pts, 500, 0.01, 10);

    Ptr<BackgroundSubtractorMOG2> bg = createBackgroundSubtractorMOG2(500, 30, true);
    bg->setShadowValue(127);

    vector<movingObj> objects;
    int nextID = 0;
    Mat canvas = Mat::zeros(Size(w * 2, h * 2), CV_8UC3);

    while (true) {
        Mat frame, cur_grey;
        if (!cap.read(frame)) break;
        cvtColor(frame, cur_grey, COLOR_BGR2GRAY);

        // --- 1. АБСОЛЮТНА СТАБІЛІЗАЦІЯ ---
        vector<Point2f> cur_pts;
        vector<uchar> status;
        vector<float> err;
        calcOpticalFlowPyrLK(ref_grey, cur_grey, ref_pts, cur_pts, status, err);

        vector<Point2f> matched_ref, matched_cur;
        for (size_t i = 0; i < status.size(); i++) {
            if (status[i]) {
                matched_ref.push_back(ref_pts[i]);
                matched_cur.push_back(cur_pts[i]);
            }
        }

        Mat T_final = Mat::eye(2, 3, CV_64F);
        if (matched_ref.size() > 20) {
            // RANSAC ігнорує машини, що рухаються
            Mat M = estimateAffinePartial2D(matched_cur, matched_ref, noArray(), RANSAC, 3.0);
            if (!M.empty()) {
                // Тільки зсув (залізна фіксація)
                T_final.at<double>(0, 2) = M.at<double>(0, 2);
                T_final.at<double>(1, 2) = M.at<double>(1, 2);
            }
        }

        Mat stabilized;
        warpAffine(frame, stabilized, T_final, Size(w, h), INTER_LINEAR, BORDER_REPLICATE);
        stabilized = cropAndZoom(stabilized, ZOOM_FACTOR);

        // --- 2. МАСКИ ТА ФОН ---
        Mat fgMask;
        bg->apply(stabilized, fgMask);
        
        Mat bgModel;
        bg->getBackgroundImage(bgModel);

        Mat objMask;
        inRange(fgMask, Scalar(250), Scalar(255), objMask);
        Mat cleanObjMask = cleanMaskImproved(objMask);

        // --- 3. ТРЕКІНГ ---
        vector<vector<Point>> contours;
        findContours(cleanObjMask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
        vector<vector<Point>> validContours;
        for(const auto& c : contours) {
            if(contourArea(c) > CONTOUR_MIN_AREA) validContours.push_back(c);
        }

        updateTracking(objects, validContours, nextID);

        // --- 4. ВІЗУАЛІЗАЦІЯ ДАШБОРДУ ---
        Mat visFrame = stabilized.clone();
        for(auto& obj : objects) {
            if (obj.framesNotSeen < 5) {
                rectangle(visFrame, obj.boundRect, obj.color, 2);
                double speedKmh = obj.speed * PIXELS_TO_METERS * fps * 3.6;
                string txt = obj.type + " " + to_string((int)speedKmh) + " km/h";
                putText(visFrame, txt, Point(obj.boundRect.x, obj.boundRect.y - 5), FONT_HERSHEY_SIMPLEX, 0.5, obj.color, 2);
                
                if(obj.trajectory.size() > 1) {
                    vector<Point> p;
                    for(auto pt : obj.trajectory) p.push_back(Point(cvRound(pt.x), cvRound(pt.y)));
                    polylines(visFrame, p, false, obj.color, 2);
                }
            }
        }

        // Формування 4-панельного вигляду
        Mat bgVis, rawMaskVis, cleanMaskVis;
        if (!bgModel.empty()) resize(bgModel, bgVis, Size(w, h)); else bgVis = Mat::zeros(Size(w,h), CV_8UC3);
        cvtColor(fgMask, rawMaskVis, COLOR_GRAY2BGR);
        cvtColor(cleanObjMask, cleanMaskVis, COLOR_GRAY2BGR);

        visFrame.copyTo(canvas(Rect(0, 0, w, h)));
        bgVis.copyTo(canvas(Rect(w, 0, w, h)));
        rawMaskVis.copyTo(canvas(Rect(0, h, w, h)));
        cleanMaskVis.copyTo(canvas(Rect(w, h, w, h)));

        // Підписи
        putText(canvas, "TRACKING (STABILIZED)", Point(20, 40), 1, 1.5, Scalar(0, 255, 0), 2);
        putText(canvas, "BACKGROUND MODEL", Point(w + 20, 40), 1, 1.5, Scalar(255, 255, 0), 2);
        putText(canvas, "RAW MOVEMENT MASK", Point(20, h + 40), 1, 1.5, Scalar(0, 0, 255), 2);
        putText(canvas, "CLEANED MASK (FOR TRACKER)", Point(w + 20, h + 40), 1, 1.5, Scalar(255, 0, 255), 2);

        Mat display;
        resize(canvas, display, Size(1280, 720));
        imshow("Absolute Stabilization Dashboard", display);

        // Авто-оновлення опорних точок, якщо їх стало замало (через зміну освітлення тощо)
        if (matched_ref.size() < 100) {
            cvtColor(stabilized, ref_grey, COLOR_BGR2GRAY);
            goodFeaturesToTrack(ref_grey, ref_pts, 500, 0.01, 10);
        }

        if(waitKey(1) == 27) break;
    }

    cap.release();
    destroyAllWindows();
    return 0;
}