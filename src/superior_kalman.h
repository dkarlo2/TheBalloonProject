#ifndef SUPERIOR_KALMAN_H
#define	SUPERIOR_KALMAN_H

#include "config_parser.h"

#include "opencv2/core/core.hpp"
#include "config_parser.h"

class SuperiorKalman {
    int stateSize;
    int measurementSize;

    cv::Mat statePre;           //!< predicted state (x'(k)): x(k)=A*x(k-1)
    cv::Mat statePost;          //!< corrected state (x(k)): x(k)=x'(k)+K(k)*(z(k)-H*x'(k))
    cv::Mat transitionMatrix;   //!< state transition matrix (A)
    cv::Mat measurementMatrix;  //!< measurement matrix (H) !!! Replaced by Jacobian in EKF
    cv::Mat processNoiseCov;    //!< process noise covariance matrix (Q)
    cv::Mat measurementNoiseCov;//!< measurement noise covariance matrix (R)
    cv::Mat errorCovPre;        //!< priori error estimate covariance matrix (P'(k)): P'(k)=A*P(k-1)*At + Q)
    cv::Mat gain;               //!< Kalman gain matrix (K(k)): K(k)=P'(k)*Ht*inv(H*P'(k)*Ht+R)
    cv::Mat errorCovPost;       //!< posteriori error estimate covariance matrix (P(k)): P(k)=(I-K(k)*H)*P'(k)
    
    // Mat getMeasurement();
    // Mat getMJacobian();

public:
    SuperiorKalman(ConfigParser config);
    cv::Mat predict();
    cv::Mat correct(cv::Mat measurement);
};

#endif

