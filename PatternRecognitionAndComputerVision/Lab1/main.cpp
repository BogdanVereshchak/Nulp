#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <limits>

using namespace cv;
using namespace std;

// ==================== Структури ====================
struct ImageInfo {
    int width;
    int height;
    int channels;
};

// ==================== Функції ====================

// 1. Відкрити відеофайл
VideoCapture openVideo(const string& filename) {
    VideoCapture cap(filename);
    if (!cap.isOpened()) {
        cerr << "Cant open video: " << filename << endl;
    }
    return cap;
}

// 2. Зберегти 5 кадрів відео у файли
vector<string> saveFrames(VideoCapture& cap, int count = 5, const string& prefix = "frame") {
    vector<string> filenames;
    Mat frame;
    for (int i = 0; i < count; ++i) {
        cap >> frame;
        if (frame.empty()) break;
        string filename = prefix + to_string(i) + ".jpg";
        imwrite(filename, frame);
        filenames.push_back(filename);

        namedWindow("Video", WINDOW_NORMAL);
        imshow("Video", frame);
        waitKey(0);

        cout << "Saved " << filename << endl;
    }
    return filenames;
}

// 3. Відкрити зображення
Mat openImage(const string& filename) {
    Mat img = imread(filename);
    if (img.empty()) {
        cerr << "Cant open image: " << filename << endl;
    } else {
        // Відобразити кадр
        namedWindow("Image", WINDOW_NORMAL);
        imshow("Image", img);
        waitKey(0);
    }
    return img;
}

// 4. Визначити розмір зображення
ImageInfo getImageInfo(const Mat& img) {
    ImageInfo info;
    info.width = img.cols;
    info.height = img.rows;
    info.channels = img.channels();
    return info;
}

// 6. Перетворити зображення в чорнобіле
Mat convertToGray(const Mat& img) {
    Mat gray;
    if (img.channels() == 3)
        cvtColor(img, gray, COLOR_BGR2GRAY);
    else
        gray = img.clone();

    // Показати
    namedWindow("Gray", WINDOW_NORMAL);
    imshow("Gray", gray);
    waitKey(0);

    return gray;
}

// 7. Застосувати медіанний фільтр
Mat applyMedianFilter(const Mat& img, int ksize = 3) {
    Mat filtered;
    medianBlur(img, filtered, ksize);

    // Показати
    namedWindow("Median Filter", WINDOW_NORMAL);
    imshow("Median Filter", filtered);
    waitKey(0);

    return filtered;
}

// 8. Зсунути два кадри для мінімальної різниці
Point findMinimalDifferenceShift(const Mat& frame1, const Mat& frame2, int maxShift = 5)
{
    // Перетворюємо кадри в grayscale
    Mat gray1, gray2;
    if(frame1.channels() == 3) cvtColor(frame1, gray1, COLOR_BGR2GRAY);
    else gray1 = frame1.clone();
    if(frame2.channels() == 3) cvtColor(frame2, gray2, COLOR_BGR2GRAY);
    else gray2 = frame2.clone();

    int minSum = numeric_limits<int>::max();
    Point bestShift(0,0);

    Mat diffOriginal;
    absdiff(gray1, gray2, diffOriginal); // різниця до зміщення

    // Перебираємо всі зсуви
    for(int dx=-maxShift; dx<=maxShift; dx++)
    {
        for(int dy=-maxShift; dy<=maxShift; dy++)
        {
            Mat shifted;
            Mat translationMat = (Mat_<double>(2,3) << 1,0,dx, 0,1,dy);
            warpAffine(gray2, shifted, translationMat, gray2.size());

            Mat diff;
            absdiff(gray1, shifted, diff);
            int sumDiff = sum(diff)[0];

            if(sumDiff < minSum){
                minSum = sumDiff;
                bestShift = Point(dx,dy);
            }
        }
    }

    // Показуємо кадри та різниці
    Mat shiftedBest;
    Mat translationMatBest = (Mat_<double>(2,3) << 1,0,bestShift.x, 0,1,bestShift.y);
    warpAffine(gray2, shiftedBest, translationMatBest, gray2.size());

    Mat diffAfter;
    absdiff(gray1, shiftedBest, diffAfter);

    namedWindow("Frame1", WINDOW_NORMAL); imshow("Frame1", gray1);
    namedWindow("Frame2 Original", WINDOW_NORMAL); imshow("Frame2 Original", gray2);
    namedWindow("Frame2 Shifted", WINDOW_NORMAL); imshow("Frame2 Shifted", shiftedBest);
    namedWindow("Difference Before", WINDOW_NORMAL); imshow("Difference Before", diffOriginal);
    namedWindow("Difference After", WINDOW_NORMAL); imshow("Difference After", diffAfter);
    waitKey(0); // чекаємо натискання клавіші

    cout << "Best shift: dx=" << bestShift.x << ", dy=" << bestShift.y << endl;
    return bestShift;
}

// ==================== Головна функція ====================
int main() {
    VideoCapture cap = openVideo("Traffic.mp4");
    if (!cap.isOpened()) return -1;

    vector<string> filenames = saveFrames(cap, 5);

    vector<Mat> processedImages;
    for (const auto& fname : filenames) {
        Mat img = openImage(fname);
        if (img.empty()) continue;

        ImageInfo info = getImageInfo(img);
        cout << fname << " : " << info.width << "x" << info.height << ", channels=" << info.channels << endl;

        Mat gray = convertToGray(img);
        Mat filtered = applyMedianFilter(gray, 5);

        processedImages.push_back(filtered);
    }

    if (processedImages.size() >= 2) {
        Point shift = findMinimalDifferenceShift(processedImages[0], processedImages[1], 5);
        cout << "Minimal difference between two frames dx=" 
             << shift.x << ", dy=" << shift.y << endl;
    }

    destroyAllWindows();
    cout << "All Done!" << endl;
    return 0;
}
