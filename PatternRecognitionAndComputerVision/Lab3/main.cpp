#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <string>


using namespace cv;
using namespace std;

const double PIXELS_TO_METERS = 0.07; // (Метрів / 1 піксель) 
// Мінімальна площа контуру (в пікселях), щоб вважатися об'єктом
const double CONTOUR_MIN_AREA = 1000.0; 
// Площа (в пікселях), вище якої об'єкт вважається вантажівкою
const double TRUCK_AREA_THRESHOLD = 8000.0; 


struct movingObj
{
    int id;
    Point2f center;
    Point2f prevCenter;
    vector<Point2f> trajectory;
    double speed;
    double averageSpeed;
    double area;
    string type;
    Scalar color;
    int framesNotSeen;
};


/**
 * @brief Налаштовує 4 фільтри Калмана для (x, y, кут, масштаб).
 */
void setupKalmanFilters(KalmanFilter& kf_x, KalmanFilter& kf_y, KalmanFilter& kf_a, double Q, double R) {
    auto setup = [&](KalmanFilter& kf, double q, double r) {
        kf.init(2, 1, 0, CV_64F); // Стан: [val, d_val], Вимірювання: [val]
        kf.transitionMatrix = (Mat_<double>(2, 2) << 1, 1, 0, 1);
        kf.measurementMatrix = (Mat_<double>(1, 2) << 1, 0);
        kf.processNoiseCov = (Mat_<double>(2, 2) << q, q / 2, q / 2, q);
        kf.measurementNoiseCov = (Mat_<double>(1, 1) << r);
        kf.errorCovPost = (Mat_<double>(2, 2) << 1, 1, 1, 1);
        kf.statePost = (Mat_<double>(2, 1) << 0, 0);
    };
    setup(kf_x, Q, R);
    setup(kf_y, Q, R);
    setup(kf_a, Q / 100.0, R / 100.0); // Кут менш "шумний"
}

/**
 * @brief Виправляє та прогнозує стан фільтра Калмана.
 */
double filterAndPredict(KalmanFilter& kf, double measurement) {
    kf.correct((Mat_<double>(1, 1) << measurement));
    Mat prediction = kf.predict();
    return prediction.at<double>(0, 0);
}


Mat cleanMask(Mat mask) {
    Mat cleanedMask;
    // morphologyEx(mask, cleanedMask, MORPH_OPEN, getStructuringElement(MORPH_ELLIPSE, Size(3, 3)));
    // morphologyEx(cleanedMask, cleanedMask, MORPH_CLOSE, getStructuringElement(MORPH_ELLIPSE, Size(7, 7)));
    // dilate(cleanedMask, cleanedMask, getStructuringElement(MORPH_ELLIPSE, Size(7, 7)), Point(-1, -1), 1);

    medianBlur(mask, cleanedMask, 3);
    dilate(cleanedMask, cleanedMask, getStructuringElement(MORPH_RECT, Size(3,3)), Point(-1,-1), 1);
    return cleanedMask;
}

vector<vector<Point>> findMovingObjContours(Mat frame, double minArea) {
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
            centers.push_back(Point2f(float(m.m10 / m.m00), float(m.m01 / m.m00)));
        }
    }
    return centers;
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

vector<movingObj> initObjects(const vector<vector<Point>>& contours) {
    vector<movingObj> objs;
    vector<Point2f> centers = getContourCenters(contours);
    int idCounter = 0;

    for (size_t i = 0; i < contours.size(); ++i) {
        movingObj obj;
        obj.id = idCounter++;
        obj.center = centers[i];
        obj.prevCenter = centers[i];
        obj.speed = 0;
        obj.averageSpeed = 0;
        obj.trajectory.push_back(centers[i]);
        obj.color = Scalar(theRNG().uniform(0,256), theRNG().uniform(0,256), theRNG().uniform(0,256));
        obj.area = contourArea(contours[i]);
        obj.type = (obj.area > TRUCK_AREA_THRESHOLD) ? "Truck" : "Car";
        obj.framesNotSeen = 0;
        objs.push_back(obj);
    }
    return objs;
}

void updateObjects(vector<movingObj>& objs, const vector<vector<Point>>& contours) {
    vector<Point2f> centers = getContourCenters(contours);
    vector<bool> matched(centers.size(), false);
    int nextID = 0; // Буде оновлено

    for (auto& obj : objs) {
        obj.framesNotSeen++; // Припускаємо, що об'єкт не видно
        if (obj.id >= nextID) nextID = obj.id + 1;

        int idx = findClosest(obj.center, centers);
        if (idx != -1) {
            obj.prevCenter = obj.center;
            obj.center = centers[idx];
            obj.trajectory.push_back(centers[idx]);
            obj.speed = norm(obj.center - obj.prevCenter);
            obj.averageSpeed = (obj.averageSpeed * (obj.trajectory.size() - 2) + obj.speed) / (obj.trajectory.size() - 1);
            
            obj.area = contourArea(contours[idx]);
            obj.type = (obj.area > TRUCK_AREA_THRESHOLD) ? "Truck" : "Car";
            
            matched[idx] = true;
            obj.framesNotSeen = 0;
        }
    }

    objs.erase(remove_if(objs.begin(), objs.end(), [](const movingObj& obj) {
        return obj.framesNotSeen > 10;
    }), objs.end());


    for (int j = 0; j < centers.size(); ++j) {
        if (!matched[j]) {
            movingObj newObj;
            newObj.id = nextID++;
            newObj.center = centers[j];
            newObj.prevCenter = centers[j];
            newObj.speed = 0;
            newObj.averageSpeed = 0;
            newObj.trajectory.push_back(centers[j]);
            newObj.color = Scalar(theRNG().uniform(0,256), theRNG().uniform(0,256), theRNG().uniform(0,256));
            newObj.area = contourArea(contours[j]);
            newObj.type = (newObj.area > TRUCK_AREA_THRESHOLD) ? "Truck" : "Car";
            newObj.framesNotSeen = 0;
            objs.push_back(newObj);
        }
    }
}

void drawTracking(Mat& frame, const vector<movingObj>& objs, double fps, double px_to_m_ratio) {
    for (auto& obj : objs) {
        // Малюємо траєкторію
        if (obj.trajectory.size() > 1) {
            for (int t = 1; t < obj.trajectory.size(); ++t) {
                line(frame, obj.trajectory[t - 1], obj.trajectory[t], obj.color, 2);
            }
        }
        
        double speed_mps = obj.speed * px_to_m_ratio * fps; 
        double speed_kph = speed_mps * 3.6; 

        string label = "ID:" + to_string(obj.id) + " " + obj.type;
        string speed_label = to_string(cvRound(speed_kph)) + " km/h";

        Point labelPos = obj.center + Point2f(10, -10);
        Point speedPos = obj.center + Point2f(10, 10);
        putText(frame, label, labelPos, FONT_HERSHEY_SIMPLEX, 0.6, obj.color, 2);
        putText(frame, speed_label, speedPos, FONT_HERSHEY_SIMPLEX, 0.5, obj.color, 1);
        circle(frame, obj.center, 5, obj.color, -1);
        
        // Малюємо рамку (з контурів не так просто, візьмемо з центру)
        // Rect bbox = boundingRect(c); // Ми не маємо тут контуру, можна додати в struct
        // rectangle(frame, bbox, obj.color, 2);
    }
}

int main() {
    VideoCapture cap("Traffic.mp4");
    if (!cap.isOpened()) {
        cout << "Cant open video!" << endl;
        return -1;
    }

    double fps = cap.get(CAP_PROP_FPS);
    int frame_w = (int)cap.get(CAP_PROP_FRAME_WIDTH);
    int frame_h = (int)cap.get(CAP_PROP_FRAME_HEIGHT);
    Size frameSize(frame_w, frame_h);
    
    Size gridSize(frame_w * 2, frame_h * 2);
    Mat canvas = Mat::zeros(gridSize, CV_8UC3);

    Rect roi_TL(0, 0, frame_w, frame_h); // Top-Left
    Rect roi_TR(frame_w, 0, frame_w, frame_h); // Top-Right
    Rect roi_BL(0, frame_h, frame_w, frame_h); // Bottom-Left
    Rect roi_BR(frame_w, frame_h, frame_w, frame_h); // Bottom-Right

    namedWindow("Traffic Dashboard", WINDOW_NORMAL);


    // --- Ініціалізація стабілізатора ---
    // KalmanFilter kf_x, kf_y, kf_a;
    // setupKalmanFilters(kf_x, kf_y, kf_a, 0.01, 0.3);
    double smooth_dx = 0.0, smooth_dy = 0.0;
    const double ALPHA = 0.85;
    Mat prev_frame, prev_grey;
    vector<Point2f> prev_pts;
    cap.read(prev_frame);
    if (prev_frame.empty()) return -1;
    cvtColor(prev_frame, prev_grey, COLOR_BGR2GRAY);

    Ptr<BackgroundSubtractor> bg = createBackgroundSubtractorMOG2(500, 16, true); // true = detectShadows

    // cv::cuda::GpuMat gpu_frame, gpu_fgmask;
    // cv::Ptr<cv::cuda::BackgroundSubtractorMOG2> bg = cv::cuda::createBackgroundSubtractorMOG2(500, 16, true);

    vector<movingObj> objects;
    bool initialized = false;
    TickMeter tm;
    Mat frame;
    cv::setUseOptimized(true);
    cv::setNumThreads(8);
    while (cap.read(frame)) {
        if (frame.empty()) break;
        tm.start();
        // --- 1. Стабілізація ---
        Mat cur_frame = frame.clone();
        Mat cur_grey;
        cvtColor(cur_frame, cur_grey, COLOR_BGR2GRAY);

        Mat fgMaskPrev;
        bg->apply(prev_frame, fgMaskPrev); 
        Mat staticMaskPrev;
        threshold(fgMaskPrev, staticMaskPrev, 200, 255, THRESH_BINARY_INV);

        if (prev_pts.size() < 80) {
            vector<Point2f> candidates;
            goodFeaturesToTrack(prev_grey, candidates, 100, 0.01, 8, staticMaskPrev);
            prev_pts = candidates;
        }

        vector<Point2f> cur_pts;
        vector<uchar> status;
        vector<float> err;

        if (!prev_pts.empty()) {
            calcOpticalFlowPyrLK(prev_grey, cur_grey, prev_pts, cur_pts, status, err, Size(25,25), 3);
        }

        vector<double> dxs, dys;
        for (size_t i = 0; i < status.size(); ++i) {
            if (status[i]) {
                double dx = cur_pts[i].x - prev_pts[i].x;
                double dy = cur_pts[i].y - prev_pts[i].y;
                dxs.push_back(dx);
                dys.push_back(dy);
            }
        }

        double dx = 0.0, dy = 0.0;
        bool haveGoodEstimate = false;

        if (dxs.size() >= 6) {
            auto median = [](vector<double>& v) -> double {
                size_t n = v.size();
                nth_element(v.begin(), v.begin() + n/2, v.end());
                double med = v[n/2];
                if (n % 2 == 0) {
                    double a = *max_element(v.begin(), v.begin() + n/2);
                    med = (med + a) / 2.0;
                }
                return med;
            };
            dx = median(dxs);
            dy = median(dys);
            haveGoodEstimate = true;
        } else {
            Mat f1, f2;
            prev_grey.convertTo(f1, CV_32F);
            cur_grey.convertTo(f2, CV_32F);
            Point2d p = phaseCorrelate(f1, f2);
            dx = p.x;
            dy = p.y;
            if (std::hypot(dx, dy) < 1000.0) haveGoodEstimate = true;
        }

        // Згладжування (EMA)
        smooth_dx = ALPHA * smooth_dx + (1.0 - ALPHA) * dx;
        smooth_dy = ALPHA * smooth_dy + (1.0 - ALPHA) * dy;

        // Побудова матриці трансформації для чистого зміщення
        Mat transformation_matrix = Mat::eye(2, 3, CV_64F);
        transformation_matrix.at<double>(0, 2) = -smooth_dx; 
        transformation_matrix.at<double>(1, 2) = -smooth_dy;

        Mat stabilized_frame;
        warpAffine(cur_frame, stabilized_frame, transformation_matrix, cur_frame.size(), INTER_LINEAR, BORDER_REPLICATE);

        // --- 2.2 Відображення "Original vs Stabilized" ---
        // Mat comparison_canvas = Mat::zeros(cur_frame.rows, cur_frame.cols * 2 + 10, cur_frame.type());
        // cur_frame.copyTo(comparison_canvas(Rect(0, 0, cur_frame.cols, cur_frame.rows)));
        // stabilized_frame.copyTo(comparison_canvas(Rect(cur_frame.cols + 10, 0, cur_frame.cols, cur_frame.rows)));
        
        // putText(comparison_canvas, "Original (Shaky)", Point(10, 30), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 0, 255), 2);
        // putText(comparison_canvas, "Stabilized", Point(cur_frame.cols + 10 + 10, 30), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 0), 2);
        
        // if (comparison_canvas.cols > 1920) {
        //     resize(comparison_canvas, comparison_canvas, Size(comparison_canvas.cols / 2, comparison_canvas.rows / 2));
        // }
        // imshow("Original vs Stabilized", comparison_canvas);

        // --- 2. Віднімання фону та тіней ---
        Mat fgMask,backgroundModel, shadowMask, foregroundOnlyMask;
        bg->apply(stabilized_frame, fgMask);
        bg->getBackgroundImage(backgroundModel);
        
        compare(fgMask, 127, shadowMask, CMP_EQ); // 127 = тінь
        compare(fgMask, 255, foregroundOnlyMask, CMP_EQ); // 255 = передній план
        
        Mat mask = cleanMask(foregroundOnlyMask);

        // --- 3. Виявлення та відстеження об'єктів ---
        vector<vector<Point>> contours = findMovingObjContours(mask, CONTOUR_MIN_AREA);
       
        if (!initialized && !contours.empty()) {
            objects = initObjects(contours);
            initialized = true;
        } else if (initialized) {
            updateObjects(objects, contours);
        }

        // --- 4. Малювання результатів ---
        drawTracking(stabilized_frame, objects, fps, PIXELS_TO_METERS);

        // --- 5. Збірка сітки (Grid) ---
                Mat bgModelVis, shadowVis, maskVis;
        
        if (!backgroundModel.empty())
            resize(backgroundModel, bgModelVis, frameSize);
        else
            bgModelVis = Mat::zeros(frameSize, CV_8UC3); // Заглушка, якщо модель ще не готова

        cvtColor(shadowMask, shadowVis, COLOR_GRAY2BGR);
        resize(shadowVis, shadowVis, frameSize);
        
        cvtColor(mask, maskVis, COLOR_GRAY2BGR);
        resize(maskVis, maskVis, frameSize);

        stabilized_frame.copyTo(canvas(roi_TL));
        bgModelVis.copyTo(canvas(roi_TR));
        maskVis.copyTo(canvas(roi_BL));
        shadowVis.copyTo(canvas(roi_BR));

        putText(canvas, "Tracking (Stabilized)", roi_TL.tl() + Point(10, 30), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(255, 255, 0), 2);
        putText(canvas, "Background Model", roi_TR.tl() + Point(10, 30), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(255, 255, 0), 2);
        putText(canvas, "Cleaned Mask (FG Only)", roi_BL.tl() + Point(10, 30), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(255, 255, 0), 2);
        putText(canvas, "Shadows Only", roi_BR.tl() + Point(10, 30), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(255, 255, 0), 2);

        // --- 6. Відображення ---
        
        tm.stop();
        double fps = 1.0 / tm.getTimeSec(); 
        tm.reset();

        putText(canvas, format("FPS: %.1f", fps), Point(20, 60), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(255, 255, 0), 2);
        putText(canvas, format("Frame time: %.1f ms", 1000.0 / fps), Point(20, 90), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 255, 255), 1);

        Mat display; 
        resize(canvas, display, Size(), 0.5, 0.5);
        imshow("Traffic Dashboard", display);
        //imshow("Traffic Dashboard", canvas);

        prev_frame = cur_frame.clone();
        prev_grey = cur_grey.clone();
        vector<Point2f> newPrevPts;
        for (size_t i = 0; i < status.size(); ++i) {
            if (status[i]) newPrevPts.push_back(cur_pts[i]);
        }
        prev_pts = newPrevPts;
        
        if (waitKey(1) == 27) break; // Вихід по ESC
    }

    cap.release();
    destroyAllWindows();
    return 0;
}
