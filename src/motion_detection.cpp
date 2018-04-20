#include "motion_detection.h"

static bool nulSize(cv::Mat m) {
    return m.rows == 0 || m.cols == 0;
}

void MotionDetector::init(ConfigParser config) {
    int gaussians = config.getInt("nGaussianMixtures");
    mog2 = new cv::ocl::MOG2(gaussians);
}

cv::Mat MotionDetector::detect(cv::Mat img) {
    if (nulSize(img)) {
        return img;
    }
    
    d_frame.upload(img);
    mog2->operator()(d_frame, d_fgmask, -0.5);
    mog2->getBackgroundImage(d_bgimg);
    d_fgimg.create(d_frame.size(), d_frame.type());
    d_fgimg.setTo(cv::Scalar::all(0));
    d_frame.copyTo(d_fgimg, d_fgmask);

    d_fgmask.download(fgmask);
    d_fgimg.download(fgimg);
    d_bgimg.download(bgimg);
    
    return fgimg;
}
