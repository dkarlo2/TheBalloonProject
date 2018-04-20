#include "video_tracker.h"
#include "circles.h"
#include "motion_detection.h"
#include "util.h"

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <chrono>
#include <cmath>
#include <ctime>
#include <iostream>
#include <string>

static int inspectW;
static int inspectH;

/**
 * Makes a rectangle describing region around the given position and takes
 * care of image boundaries. The region size is defined by inspectWidth and
 * inspectHeight in the configuration file.
 * @param pos Position of the expected center of the region.
 *              This point will not be in the center in the case when it's
 *              close enough to the edge of the image.
 * @param rows Image height.
 * @param cols Image width.
 * @return Rectangle describing the region.
 */
static cv::Rect getInspectRegion(cv::Mat pos, int rows, int cols) {
    float sx = pos.at<float>(0);
    float sy = pos.at<float>(1);
    float x1 = max<float>(0, sx - inspectW/2);
    float x2 = min<float>(cols, sx + inspectW/2);
    float y1 = max<float>(0, sy - inspectH/2);
    float y2 = min<float>(rows, sy + inspectH/2);
    return cv::Rect(x1, y1, x2-x1, y2-y1);
}

static void onMouse( int event, int x, int y, int, void* data) {
    if (event == cv::EVENT_LBUTTONDOWN) {
        Condensation *filter = (Condensation*) data;
        filter->reinitialize();
    }
}

void VideoTracker::init(ConfigParser config) {
    int imgArea = config.getInt("imageArea");
    inspectW = config.getInt("inspectWidth");
    inspectH = config.getInt("inspectHeight");
    
    // open the video file or the camera stream
    const char* v = config.getString(concat("video", id+1));
    if (is_number(v)) {
        int d;
        sscanf(v, "%d", &d);
        vid = new cv::VideoCapture(d);
    } else {
        vid = new cv::VideoCapture(v);
    }
    
    if (!vid->isOpened()) {
        std::cerr << "Cannot open video file " << (id+1) << "." << std::endl;
        return;
    }
    
    fw = vid->get(CV_CAP_PROP_FRAME_WIDTH);
    fh = vid->get(CV_CAP_PROP_FRAME_HEIGHT);
    
    // calculate scale for resizing
    scaleFactor = sqrt(imgArea / (double)(fw*fh));

    // init filters
    int n = config.getInt("nBalloons");
    for (int i = 0; i < n; i++) {
        Condensation *filter = new Condensation(i);
        filter->init(config, fw * scaleFactor, fh * scaleFactor);
        filters.push_back(filter);
    }
    
    frameCount = vid->get(CV_CAP_PROP_FRAME_COUNT);
    fps = vid->get(CV_CAP_PROP_FPS);
    
    // load calibration data
    const char* calibFile = config.getString("calibrationFile");
    cv::FileStorage fs(calibFile, cv::FileStorage::READ);
    cv::Mat CM, D, R, P;
    fs[concat("CM", id+1)] >> CM;
    fs[concat("D", id+1)] >> D;
    fs[concat("R", id+1)] >> R;
    fs[concat("P", id+1)] >> P;
    
    initUndistortRectifyMap(CM, D, R, P, cv::Size(fw, fh), CV_32FC1, ur_mapx, ur_mapy);
    
    estimatedStates = new cv::Mat[n];
    
    md->init(config);
    
    int swx = config.getInt("startWindowX");
    int swy = config.getInt("startWindowY");
    int dwx = config.getInt("deltaWindowX");
    int dwy = config.getInt("deltaWindowY");
    int mww = config.getInt("maxWindowWidth");
    
    window = new MyWindow(concat("Balloon Tracker Camera", id+1), swx+dwx*id, swy+dwy*id, mww);
    window->init(fw, fh);
    /*for (int i = 0; i < n; i++) {
        window->setMouseCallback(onMouse, (void*)filters[i]);
    }*/
}

int VideoTracker::next_frame() {
    if (frameCount == -1 || vid->get(CV_CAP_PROP_POS_FRAMES) != frameCount) {
        if (!vid->read(frame)) {
            std::cout << "Cannot read the frame." << std::endl;
            return -1;
        }
    } else {
        return 1;
    }
    
    // undistort and rectify
    remap(frame, frame, ur_mapx, ur_mapy, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar());
    
    // resize to reduce computation time
    resize(frame, frame, cv::Size(), scaleFactor, scaleFactor);
    
    // create the image that will be displayed
    cv::Mat frame_visual = frame.clone();
    
    // inspect regions for each filter
    std::vector<cv::Rect> regions;

    for (int i = 0; i < filters.size(); i++) {
        Condensation *filter = filters[i];

        // filter prediction step
        cv::Mat prediction = filter->predict();

        // this part of code will set the inspect region for searching,
        // depending on the current state of the tracking
        cv::Rect region;
        if (prediction.at<float>(0) < 0 || prediction.at<float>(0) > frame.cols
                || prediction.at<float>(1) < 0 || prediction.at<float>(1) > frame.rows
                || estimatedStates[i].empty()) {
            region = cv::Rect(0, 0, frame.cols, frame.rows);
        } else {
            region = getInspectRegion(prediction, frame.rows, frame.cols); // testing
            // region = getInspectRegion(estimatedStates[i], frame.rows, frame.cols);
        }

        // draw the inspect region
        rectangle(frame_visual, region, filter->getBalloonMean());

        regions.push_back(region);
    }
    
    // get the foreground of the frame
    cv::Mat frame_motion = md->detect(frame);
    
    // update measurements
    update_circles(frame_motion, frame, filters, regions);
    
    for (int i = 0; i < filters.size(); i++) {
        Condensation *filter = filters[i];

        // correct the state prediction
        estimatedStates[i] = filter->correct();

        // draw the posterior state on the screen in green
        double xe = estimatedStates[i].at<float>(0);
        double ye = estimatedStates[i].at<float>(1);

        drawCross(frame_visual, cv::Point(xe, ye), cv::Scalar(0, 255, 0), 3);

        // filter->drawParticles(&frame_visual);
    }
    
    window->showImage(frame_visual);
    
    return 0;
}
