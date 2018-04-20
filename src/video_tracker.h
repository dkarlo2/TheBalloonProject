#ifndef VIDEO_TRACKER
#define VIDEO_TRACKER

#include "condensation.h"
#include "config_parser.h"
#include "motion_detection.h"
#include "window_manager.h"

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <vector>

class VideoTracker {
    int id;
    
    cv::VideoCapture *vid;
    std::vector<Condensation*> filters;
    
    cv::Mat frame;
    int fw;
    int fh;
    int frameCount;
    int fps;
    
    double scaleFactor;
    
    cv::Mat ur_mapx, ur_mapy; // undistort rectify matrices
    
    cv::Mat *estimatedStates;
    
    MotionDetector *md;
    
    MyWindow *window;
public:
    VideoTracker(int ID) : id(ID) {
        md = new MotionDetector(ID);
    };
    void init(ConfigParser config);
    int next_frame();
    ~VideoTracker() {
        delete vid;
        delete estimatedStates;
        delete md;
        for (int i = 0; i < filters.size(); i++) {
            delete filters[i];
        }
    }
    double getScaledWidth() {
        return scaleFactor * fw;
    }
    double getScaledHeight() {
        return scaleFactor * fh;
    }
    cv::Mat getFrame() {
        return frame.clone();
    }
    double getStateX(int balloon) {
        return estimatedStates[balloon].at<float>(0);
    }
    double getStateY(int balloon) {
        return estimatedStates[balloon].at<float>(1);
    }
};

#endif