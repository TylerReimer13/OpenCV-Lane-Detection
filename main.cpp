#include "opencv2/opencv.hpp"
#include <iostream>
#include <chrono>
#include <thread>
#include <vector>

using namespace cv;

Mat gen_mask(Mat& frame, bool im_show);
std::vector<std::vector<Point>> contours(Mat& edges, bool im_show);
void fill_lane(std::vector<std::vector<Point>> contours, bool im_show);

Mat frame, grey_frame, cropped, thresholded, contoured, dilated, lanes;

int threshold_value = 170;
int const max_binary_value = 255;
int threshold_type = 3;


int main() {
    VideoCapture cap("C:/Users/tyler/Pictures/OpenCv/road_pov.gif");

    if (!cap.isOpened()) {
        std::cout << "Error opening video stream or file" << std::endl;
        return -1;
    }

    while (true) {
        bool bSuccess = cap.read(frame);
        if (bSuccess == false)
        {
            std::cout << "End of video" << std::endl;
            break;
        }

        imshow("Original", frame);
        cv::cvtColor(frame, grey_frame, cv::COLOR_BGR2GRAY);
        blur(grey_frame, grey_frame, Size(3, 3));
        threshold(grey_frame, thresholded, threshold_value, max_binary_value, threshold_type);

        //imshow("Grey", grey_frame);
        //imshow("Thresholded", thresholded);

        cropped = gen_mask(thresholded, false);
        dilate(cropped, dilated, Mat(), Point(-1, -1), 3, 1, 1);
        //imshow("Dilated", dilated);
        std::vector<std::vector<Point>> conts = contours(dilated, false);
        
        fill_lane(conts, false);
        
        if (waitKey(10) == 27)
        {
            std::cout << "Video ended by user" << std::endl;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(150));
    }

    return 0;
}

Mat gen_mask(Mat& frame, bool im_show = false) {
    // (450, 250)
    Point points[1][6];
    points[0][0] = Point(125, 110); // 1
    points[0][1] = Point(325, 110); // 2
    points[0][2] = Point(445, 150); // 3
    points[0][3] = Point(450, 250); // 4
    points[0][4] = Point(0, 250); // 5
    points[0][5] = Point(5, 150); // 6

    const Point* ppt[1] = { points[0] };
    int npt[] = { 6 };

    Mat mask(frame.size(), CV_8UC1, Scalar(0, 0, 0));
    int lineType = LINE_8;

    fillPoly(mask, ppt, npt, 1, Scalar(255, 0, 0), lineType);

    Mat result(frame.size(), CV_8UC1);

    bitwise_and(frame, mask, result);

    if (im_show) {
        imshow("Cropped", result);
    };

    return result;
};

std::vector<std::vector<Point>> contours(Mat& edges, bool im_show = false) {
    std::vector<std::vector<Point>> contours;
    std::vector<Vec4i> hierarchy;

    findContours(edges, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
    //std::cout << "Countour: " << contours << std::endl;
    contoured = Mat::zeros(edges.size(), CV_8UC3);
    for (size_t i = 0; i < contours.size(); i++)
    {
        Scalar color = Scalar(255, 255, 255);
        drawContours(contoured, contours, (int)i, color, 2, LINE_8, hierarchy, 0);
    }

    if (im_show) {
        imshow("Contours", contoured);
    };

    return contours;
};

void fill_lane(std::vector<std::vector<Point>> contours, bool im_show = false) {
    std::vector<std::vector<Point>> hull(contours.size());
    for (int i = 0; i < contours.size(); i++) {
        convexHull(Mat(contours[i]), hull[i], false);
    }

    // Draw contours + hull results
    RNG rng;
    Mat drawing = Mat::zeros(dilated.size(), CV_8UC3);
    for (int i = 0; i < contours.size(); i++)
    {
        drawContours(drawing, contours, i, Scalar(0, 255, 0), 1, 8, std::vector<Vec4i>(), 0, Point());
        drawContours(drawing, hull, i, Scalar(0, 255, 0), 1, 8, std::vector<Vec4i>(), 0, Point());
    }
    
    addWeighted(frame, 1., drawing, .5, 0.0, lanes);
    imshow("Output", lanes);

    if (im_show) {
        // Show in a window
        //imshow("Hull demo", drawing);
    };
}

