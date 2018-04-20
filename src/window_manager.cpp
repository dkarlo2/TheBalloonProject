#include "window_manager.h"

#include "opencv2/highgui/highgui.hpp"

void MyWindow::init(int w, int h) {
    cv::namedWindow(name, CV_WINDOW_NORMAL);
    if (w > maxW) {
        double factor = (double)maxW / w;
        cv::resizeWindow(name, maxW, h * factor);
    } else {
        cv::resizeWindow(name, w, h);
    }
    cv::moveWindow(name, x, y);
}

void MyWindow::destroy() {
    cv::destroyWindow(name);
}

void MyWindow::showImage(cv::Mat img) {
    int imgW = img.cols;
    int imgH = img.rows;
    if (imgW > maxW) {
        double factor = (double)maxW / imgW;
        cv::resizeWindow(name, maxW, imgH * factor);
    } else {
        cv::resizeWindow(name, imgW, imgH);
    }
    cv::moveWindow(name, x, y);
    imshow(name, img);
}

void MyWindow::setMouseCallback(CvMouseCallback callback, void* data) {
    cv::setMouseCallback(name, callback, data);
}
