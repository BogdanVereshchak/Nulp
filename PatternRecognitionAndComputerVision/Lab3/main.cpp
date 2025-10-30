#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <string>

using namespace cv;
using namespace std;

const double PIXELS_TO_METERS = 0.03; // (Метрів / 1 піксель) 

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


// --- ФУНКЦІЇ ОБРОБКИ ЗОБРАЖЕННЯ ---

Mat cleanMask(Mat mask) {
    Mat cleanedMask;
    morphologyEx(mask, cleanedMask, MORPH_OPEN, getStructuringElement(MORPH_ELLIPSE, Size(3, 3)));
    morphologyEx(cleanedMask, cleanedMask, MORPH_CLOSE, getStructuringElement(MORPH_ELLIPSE, Size(7, 7)));
    dilate(cleanedMask, cleanedMask, getStructuringElement(MORPH_ELLIPSE, Size(7, 7)), Point(-1, -1), 1);
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


// --- ФУНКЦІЇ ВІДСТЕЖЕННЯ---

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

/**
 * @brief Ініціалізує об'єкти на основі перших контурів 
 */
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
        obj.color = Scalar(rand() % 256, rand() % 256, rand() % 256);
        obj.area = contourArea(contours[i]);
        obj.type = (obj.area > TRUCK_AREA_THRESHOLD) ? "Truck" : "Car";
        obj.framesNotSeen = 0;
        objs.push_back(obj);
    }
    return objs;
}

/**
 * @brief Оновлює стан об'єктів новими контурами
 */
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
            
            // Оновлюємо тип та площу
            obj.area = contourArea(contours[idx]);
            obj.type = (obj.area > TRUCK_AREA_THRESHOLD) ? "Truck" : "Car";
            
            matched[idx] = true;
            obj.framesNotSeen = 0; // Об'єкт видно
        }
    }

    // Видаляємо об'єкти, яких не бачили > 10 кадрів
    objs.erase(remove_if(objs.begin(), objs.end(), [](const movingObj& obj) {
        return obj.framesNotSeen > 10;
    }), objs.end());


    // Додаємо нові об'єкти
    for (int j = 0; j < centers.size(); ++j) {
        if (!matched[j]) {
            movingObj newObj;
            newObj.id = nextID++;
            newObj.center = centers[j];
            newObj.prevCenter = centers[j];
            newObj.speed = 0;
            newObj.averageSpeed = 0;
            newObj.trajectory.push_back(centers[j]);
            newObj.color = Scalar(rand() % 256, rand() % 256, rand() % 256);
            newObj.area = contourArea(contours[j]);
            newObj.type = (newObj.area > TRUCK_AREA_THRESHOLD) ? "Truck" : "Car";
            newObj.framesNotSeen = 0;
            objs.push_back(newObj);
        }
    }
}

/**
 * @brief Малює траєкторії та інформацію про об'єкти
 */
void drawTracking(Mat& frame, const vector<movingObj>& objs, double fps, double px_to_m_ratio) {
    for (auto& obj : objs) {
        // Малюємо траєкторію
        if (obj.trajectory.size() > 1) {
            for (int t = 1; t < obj.trajectory.size(); ++t) {
                line(frame, obj.trajectory[t - 1], obj.trajectory[t], obj.color, 2);
            }
        }
        
        // --- Обрахунок швидкості ---
        double speed_mps = obj.speed * px_to_m_ratio * fps; // (пікс/кадр) * (м/пікс) * (кадр/сек) = м/сек
        double speed_kph = speed_mps * 3.6;                 // (м/сек) * (3600 сек/год) / (1000 м/км) = км/год

        // --- Формування тексту ---
        string label = "ID:" + to_string(obj.id) + " " + obj.type;
        string speed_label = to_string(cvRound(speed_kph)) + " km/h";

        // Малюємо мітку
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

// --- ГОЛОВНА ФУНКЦІЯ ---

int main() {
    VideoCapture cap("Traffic.mp4");
    if (!cap.isOpened()) {
        cout << "Cant open video!" << endl;
        return -1;
    }

    // Отримуємо FPS та розмір кадру
    double fps = cap.get(CAP_PROP_FPS);
    int frame_w = (int)cap.get(CAP_PROP_FRAME_WIDTH);
    int frame_h = (int)cap.get(CAP_PROP_FRAME_HEIGHT);
    Size frameSize(frame_w, frame_h);
    
    // Розмір сітки 2x2
    Size gridSize(frame_w * 2, frame_h * 2);
    Mat canvas = Mat::zeros(gridSize, CV_8UC3);

    // Визначаємо регіони (ROIs) для копіювання
    Rect roi_TL(0, 0, frame_w, frame_h); // Top-Left
    Rect roi_TR(frame_w, 0, frame_w, frame_h); // Top-Right
    Rect roi_BL(0, frame_h, frame_w, frame_h); // Bottom-Left
    Rect roi_BR(frame_w, frame_h, frame_w, frame_h); // Bottom-Right

    // Вікно для сітки
    namedWindow("Traffic Dashboard", WINDOW_NORMAL);


    // --- Ініціалізація стабілізатора ---
    KalmanFilter kf_x, kf_y, kf_a;
    setupKalmanFilters(kf_x, kf_y, kf_a, 0.01, 0.1);
    Mat prev_frame, prev_grey;
    vector<Point2f> prev_pts;
    cap.read(prev_frame);
    if (prev_frame.empty()) return -1;
    cvtColor(prev_frame, prev_grey, COLOR_BGR2GRAY);

    // --- Ініціалізація віднімання фону ---
    Ptr<BackgroundSubtractor> bg = createBackgroundSubtractorMOG2(500, 16, true); // true = detectShadows

    // --- Ініціалізація відстеження ---
    vector<movingObj> objects;
    bool initialized = false;

    Mat frame;
    while (cap.read(frame)) {
        if (frame.empty()) break;

        // --- 1. Стабілізація ---
        Mat cur_frame = frame.clone();
        Mat cur_grey;
        cvtColor(cur_frame, cur_grey, COLOR_BGR2GRAY);
        vector<Point2f> cur_pts;

        if (prev_pts.size() < 50) {
            goodFeaturesToTrack(prev_grey, prev_pts, 200, 0.01, 10);
        }

        vector<uchar> status;
        vector<float> err;
        calcOpticalFlowPyrLK(prev_grey, cur_grey, prev_pts, cur_pts, status, err);

        vector<Point2f> prev_pts_tracked, cur_pts_tracked;
        for (size_t i = 0; i < status.size(); ++i) {
            if (status[i]) {
                prev_pts_tracked.push_back(prev_pts[i]);
                cur_pts_tracked.push_back(cur_pts[i]);
            }
        }

        Mat T;
        if (prev_pts_tracked.size() > 5) {
             T = estimateAffinePartial2D(prev_pts_tracked, cur_pts_tracked);
        }
       
        double dx = 0, dy = 0, da = 0;
        if (!T.empty()) {
            dx = T.at<double>(0, 2);
            dy = T.at<double>(1, 2);
            da = atan2(T.at<double>(1, 0), T.at<double>(0, 0));
        }

        double smooth_dx = filterAndPredict(kf_x, dx);
        double smooth_dy = filterAndPredict(kf_y, dy);
        double smooth_da = filterAndPredict(kf_a, da);
       
        Mat transformation_matrix = Mat::zeros(2, 3, CV_64F);
        transformation_matrix.at<double>(0, 0) = cos(smooth_da);
        transformation_matrix.at<double>(0, 1) = -sin(smooth_da);
        transformation_matrix.at<double>(1, 0) = sin(smooth_da);
        transformation_matrix.at<double>(1, 1) = cos(smooth_da);
        transformation_matrix.at<double>(0, 2) = smooth_dx;
        transformation_matrix.at<double>(1, 2) = smooth_dy;

        Mat stabilized_frame;
        warpAffine(cur_frame, stabilized_frame, transformation_matrix, cur_frame.size());

        // --- 2. (ПОВЕРНУЛИ) Відображення "Original vs Stabilized" ---
        Mat comparison_canvas = Mat::zeros(cur_frame.rows, cur_frame.cols * 2 + 10, cur_frame.type());
        cur_frame.copyTo(comparison_canvas(Rect(0, 0, cur_frame.cols, cur_frame.rows)));
        stabilized_frame.copyTo(comparison_canvas(Rect(cur_frame.cols + 10, 0, cur_frame.cols, cur_frame.rows)));
        
        // Додаємо підписи
        putText(comparison_canvas, "Original (Shaky)", Point(10, 30), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 0, 255), 2);
        putText(comparison_canvas, "Stabilized", Point(cur_frame.cols + 10 + 10, 30), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 0), 2);
        
        // Зменшуємо розмір, якщо завеликий для екрану
        if (comparison_canvas.cols > 1920) {
            resize(comparison_canvas, comparison_canvas, Size(comparison_canvas.cols / 2, comparison_canvas.rows / 2));
        }
        imshow("Original vs Stabilized", comparison_canvas);

        // --- 2. Віднімання фону та тіней ---
        Mat fgMask, backgroundModel, shadowMask, foregroundOnlyMask;
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
        // Малюємо трекінг ПРЯМО на стабілізований кадр
        drawTracking(stabilized_frame, objects, fps, PIXELS_TO_METERS);

        // --- 5. Збірка сітки (Grid) ---
        
        // Створюємо версії для відображення
        Mat bgModelVis, shadowVis, maskVis;
        
        // Модель фону (вже CV_8UC3)
        if (!backgroundModel.empty())
            resize(backgroundModel, bgModelVis, frameSize);
        else
            bgModelVis = Mat::zeros(frameSize, CV_8UC3); // Заглушка, якщо модель ще не готова

        // Маска тіней (CV_8UC1 -> CV_8UC3)
        cvtColor(shadowMask, shadowVis, COLOR_GRAY2BGR);
        resize(shadowVis, shadowVis, frameSize);
        
        // Очищена маска (CV_8UC1 -> CV_8UC3)
        cvtColor(mask, maskVis, COLOR_GRAY2BGR);
        resize(maskVis, maskVis, frameSize);

        // Копіюємо всі кадри в сітку
        stabilized_frame.copyTo(canvas(roi_TL)); // Головне вікно
        bgModelVis.copyTo(canvas(roi_TR));       // Модель фону
        maskVis.copyTo(canvas(roi_BL));          // Маска (без тіней)
        shadowVis.copyTo(canvas(roi_BR));        // Тільки тіні

        // Додаємо підписи до сітки
        putText(canvas, "Tracking (Stabilized)", roi_TL.tl() + Point(10, 30), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(255, 255, 0), 2);
        putText(canvas, "Background Model", roi_TR.tl() + Point(10, 30), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(255, 255, 0), 2);
        putText(canvas, "Cleaned Mask (FG Only)", roi_BL.tl() + Point(10, 30), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(255, 255, 0), 2);
        putText(canvas, "Shadows Only", roi_BR.tl() + Point(10, 30), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(255, 255, 0), 2);

        // --- 6. Відображення ---
        imshow("Traffic Dashboard", canvas);
        
        // --- 7. Оновлення стану ---
        prev_grey = cur_grey.clone();
        prev_pts = cur_pts_tracked;
        
        if (waitKey(1) == 27) break; // Вихід по ESC
    }

    cap.release();
    destroyAllWindows();
    return 0;
}
