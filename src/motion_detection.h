#ifndef MOTION_DETECTION
#define MOTION_DETECTION

#include "config_parser.h"

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/ocl/ocl.hpp"

class MotionDetector {
    int id;
    
    cv::ocl::MOG2 *mog2;

    cv::ocl::oclMat d_frame;
    cv::ocl::oclMat d_fgmask;
    cv::ocl::oclMat d_fgimg;
    cv::ocl::oclMat d_bgimg;

    cv::Mat fgmask;
    cv::Mat fgimg;
    cv::Mat bgimg;

public:
    MotionDetector(int ID) : id(ID) {};
    void init(ConfigParser config);
    cv::Mat detect(cv::Mat img);
    ~MotionDetector() {
        delete mog2;
    };
};

#endif