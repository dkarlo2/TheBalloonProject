#include "circles.h"
#include "util.h"

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <algorithm>
#include <iostream>

#define BORDER_THICKNESS 10

static double minHit;
static double outRegion;
static int nCirclesObserved;
static int maxAge;

static double houghInverseRatio;
static double houghMinDistance;
static double houghThresholdCanny;
static double houghThresholdAccumulator;
static int houghMinRadius;
static int houghMaxRadius;

static int erodeSize;
static int dilateSize;

static int blurSize;
static double blurSigma;

void init_circles(ConfigParser config) {
    minHit = config.getDouble("minHitGoodness");
    maxAge = config.getInt("particleMaximumAge");
    outRegion = config.getDouble("outsideRegionFactor");
    nCirclesObserved = config.getInt("nCirclesObserved");
    houghInverseRatio = config.getDouble("houghInverseRatio");
    houghMinDistance = config.getDouble("houghMinDistance");
    houghThresholdCanny = config.getDouble("houghThresholdCanny");
    houghThresholdAccumulator = config.getDouble("houghThresholdAccumulator");
    houghMinRadius = config.getInt("houghMinRadius");
    houghMaxRadius = config.getInt("houghMaxRadius");
    erodeSize = config.getInt("circleErodeSize");
    dilateSize = config.getInt("circleDilateSize");
    blurSize = config.getInt("circleBlurSize");
    blurSigma = config.getDouble("circleBlurSigma");
}

static void age_and_remove_dead(std::vector<Condensation*> filters) {
    for (int j = 0; j < filters.size(); j++) {
        std::vector<CMeasurement>* measurements = filters[j]->getMeasurements();
        std::vector<int> toRemove;
        for (int i = 0; i < measurements->size(); i++) {
            CMeasurement *m = &measurements->operator[](i);
            m->age();
            if (m->isDead()) {
                toRemove.push_back(i);
            }
        }

        for (int i = 0; i < toRemove.size(); i++) {
            measurements->erase(measurements->begin() + toRemove[i] - i);
        }
    }
}

/*
 * Class that stores a data representing a circle measurement, its goodness, and the balloon it belongs to.
*/
class MeasurementHit {
public:
    int index;
    int balloon;
    double hit;
    CMeasurement measurement;
    MeasurementHit(int i, int b, double h, CMeasurement m)
        : index(i), balloon(b), hit(h), measurement(m) {}
};

static bool sortfunction(MeasurementHit a, MeasurementHit b) {
    return a.hit > b.hit;
}

static void match_circles_filters(std::vector<cv::Vec3f> circles, std::vector<Condensation*> filters, std::vector<cv::Rect> regions,
                    cv::Mat originalImg) {
    // cv::Mat drawing = originalImg.clone();

    std::vector<MeasurementHit> mhs;

    // observe only the best nCirclesObserved circles
    int nc = (circles.size() < nCirclesObserved) ? circles.size() : nCirclesObserved;
    for (int i = 0; i < nc; i++) {
        cv::Point center(std::max(0, cvRound(circles[i][0]) - BORDER_THICKNESS), std::max(0, cvRound(circles[i][1]) - BORDER_THICKNESS));
        // int radius = cvRound(circles[i][2]);
        // circle(drawing, center, radius, cv::Scalar(0, 0, 255), 2);
        for (int j = 0; j < filters.size(); j++) {
            cv::Rect region = regions[j];

            double w = 1;
            // reduce the weight if it's out of the inspect region
            if (center.x < region.x || center.x > region.x + region.width ||
                    center.y < region.y || center.y > region.y + region.height) {
                w = outRegion;
            }

            // estimate the hit goodness
            cv::Mat mask = cv::Mat::zeros(originalImg.size(), CV_8UC1);
            circle(mask, center, houghMinRadius, cv::Scalar(255), CV_FILLED);
            double hit = filters[j]->estimateHit(originalImg, mask);
            // if the hit goodness is better than the predefined threshold, add it to the measurements list
            if (hit > minHit) {
                mhs.push_back(MeasurementHit(i, j, hit, CMeasurement(center.x, center.y, w*hit*hit, maxAge)));
            }
        }
    }

    /*static int id = 0;
    id++;
    if (id == 5) {
        id = 1;
    }
    imshow(concat("Drawing", id), drawing);*/

    // circle measurements are matched to the balloons in a fashion that
    // the best measurement for each balloon is added to the measurements list of the corresponding filter,
    // but keeping in mind that the same masurement can't be matched to more than one balloon
    
    sort(mhs.begin(), mhs.end(), sortfunction);

    bool *bm = new bool[nc]();
    bool *bb = new bool[filters.size()]();

    for (int i = 0; i < mhs.size(); i++) {
        if (!bm[mhs[i].index] && !bb[mhs[i].balloon]) {
            filters[mhs[i].balloon]->getMeasurements()->push_back(mhs[i].measurement);
            bm[mhs[i].index] = true;
            bb[mhs[i].balloon] = true;
        }
    }

    delete bm;
    delete bb;
}

void update_circles(cv::Mat motionImg, cv::Mat originalImg, std::vector<Condensation*> filters, std::vector<cv::Rect> regions) {
    if (filters.size() != regions.size()) {
        std::cerr << "Vector dimensions don't match!" << std::endl;
        return;
    }
    
    age_and_remove_dead(filters);
    
    // adding a border to cover the cases when the balloon is on the edge of the frame
    int b = BORDER_THICKNESS;
    cv::Mat motionImg_border;
    copyMakeBorder(motionImg, motionImg_border, b, b, b, b, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
    
    // converting the motion image to gray
    cv::Mat motionImg_gray;
    cvtColor(motionImg_border, motionImg_gray, CV_BGR2GRAY);
    
    // this part of code was removed due to ineffectiveness
    /*// apply Gaussian blur
    cv::Mat motionImg_blurred;
    GaussianBlur(motionImg_gray, motionImg_blurred, cv::Size(blurSize, blurSize), blurSigma, blurSigma);
    
    // find circles
    std::vector<cv::Vec3f> circles;
    HoughCircles(motionImg_blurred, circles, CV_HOUGH_GRADIENT, houghInverseRatio, houghMinDistance,
            80, 10, houghMinRadius, houghMaxRadius); // TODO 80,10
    
    match_circles_filters(circles, filters, regions, originalImg);
    circles.clear();
    */
    
    // thresholding in a manner that every non-zero pixel from the image gets maximum value
    cv::Mat motionImg_bin;
    threshold(motionImg_gray, motionImg_bin, 0, 255, cv::THRESH_BINARY);
    
    // morphological opening
    int es = erodeSize;
    int ds = dilateSize;
    int ep = (erodeSize - 1) / 2 - 1;
    int dp = (dilateSize - 1) / 2 - 1;
    erode(motionImg_bin, motionImg_bin, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(es, es), cv::Point(ep, ep)));
    dilate(motionImg_bin, motionImg_bin, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(ds, ds), cv::Point(dp, dp)));
    
    // adding border to the original image
    cv::Mat originalImg_border;
    copyMakeBorder(originalImg, originalImg_border, b, b, b, b,  cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
    
    // extract only the part of the original image that's moving
    cv::Mat originalImgM_border;
    originalImg_border.copyTo(originalImgM_border, motionImg_bin);
    
    // convert to grayscale
    cv::Mat originalImgM_gray;
    cvtColor(originalImgM_border, originalImgM_gray, CV_BGR2GRAY);
    
    // apply Gaussian blur
    cv::Mat originalImgM_blurred;
    GaussianBlur(originalImgM_gray, originalImgM_blurred, cv::Size(blurSize, blurSize), blurSigma, blurSigma);
    
    // find circles
    std::vector<cv::Vec3f> circles;
    HoughCircles(originalImgM_blurred, circles, CV_HOUGH_GRADIENT, houghInverseRatio, houghMinDistance,
            houghThresholdCanny, houghThresholdAccumulator, houghMinRadius, houghMaxRadius);
            
    match_circles_filters(circles, filters, regions, originalImg);
}
