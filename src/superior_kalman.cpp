#include "superior_kalman.h"

#include <iostream>

SuperiorKalman::SuperiorKalman(ConfigParser config) {
    stateSize = config.getInt("stateSize");
    measurementSize = config.getInt("measurementSize");

    transitionMatrix = config.getMatrix("transitionMatrix");
    measurementMatrix = config.getMatrix("measurementMatrix");
    
    statePost = cv::Mat_<float>::zeros(stateSize, 1);
    statePost.at<float>(2) = 1;
    
    processNoiseCov = cv::Mat_<float>(stateSize, stateSize);
    measurementNoiseCov = cv::Mat_<float>(measurementSize, measurementSize);
    errorCovPost = cv::Mat_<float>(stateSize, stateSize);
    
    double processNoise = config.getDouble("processNoise");
    double measurementNoise = config.getDouble("measurementNoise");

    setIdentity(processNoiseCov, cv::Scalar::all(processNoise));
    setIdentity(measurementNoiseCov, cv::Scalar::all(measurementNoise));
    setIdentity(errorCovPost);
}

cv::Mat SuperiorKalman::predict() {
    statePre = transitionMatrix * statePost;
    errorCovPre = transitionMatrix * errorCovPost * transitionMatrix.t() + processNoiseCov;
    cv::Mat sp;
    statePre.copyTo(sp);
    return sp;
}

cv::Mat SuperiorKalman::correct(cv::Mat measurement) {
    if (measurement.rows != measurementSize || measurement.cols != 1) {
        std::cerr << "Wrong measurement dimensions!" << std::endl;
        return cv::Mat();
    }

    // Mat measurementJacob = getMJacobian();
    
    gain = errorCovPre * measurementMatrix.t() * (measurementMatrix * errorCovPre * measurementMatrix.t() + measurementNoiseCov).inv();
    // gain = errorCovPre * measurementJacob.t() * (measurementJacob * errorCovPre * measurementJacob.t() + measurementNoiseCov).inv();
    
    statePost = statePre + gain * (measurement - measurementMatrix * statePre);
    // statePost = statePre + gain * (measurement - getMeasurement());

    cv::Mat I = cv::Mat_<float>(gain.rows, measurementMatrix.cols);
    setIdentity(I);

    errorCovPost = (I - gain * measurementMatrix) * errorCovPre;
    // errorCovPost = (I - gain * measurementJacob) * errorCovPre;

    cv::Mat sp;
    statePost.copyTo(sp);
    return sp;
}

/*Mat SuperiorKalman::getMeasurement() {

}

Mat SuperiorKalman::getMJacobian() {

}*/
