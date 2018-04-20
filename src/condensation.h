#ifndef CONDENSATION_H
#define CONDENSATION_H

#include "config_parser.h"

#include "opencv2/core/core.hpp"

#include <ctime>
#include <vector>

/**
 * Class that represents a particle.
 */
class CParticle {
public:
    // x coordinate
    double x;
    // y coordinate
    double y;
    // speed in the direction of x axis
    double vx;
    // speed in the direction of y axis
    double vy;
    // acceleration in the direction of x axis
    double accx;
    // acceleration in the direction of y axis
    double accy;
    // importance weight of the particle
    double weight;
    // cumulative weight of the particle
    double c;
};

class CMeasurement {
public:
    double x;
    double y;
    double w;
    int maxAge;
    int lifes;
    CMeasurement(double x, double y, double w, int maxAge) : x(x), y(y), w(w), maxAge(maxAge), lifes(maxAge) {};
    void age() {
        if (lifes > 0) {
            lifes--;
        }
    };
    bool isDead() {
        return lifes <= 0;
    };
};

class Condensation {
    bool initialized;
    int id;
    
    int n;
    double xRange;
    double yRange;
    
    cv::Scalar balloonMean;
    
    CParticle *particles;
    std::vector<CMeasurement> measurements;
    
    cv::Mat_<float> pred;
    
    clock_t timeStamp;
    
    int findParticleByR(double r, int s, int e);
    void injectNewParticles();
    cv::Mat getStateEstimate(int mode);
    void applyDynamics();
public:
    Condensation(int ID) : initialized(false), id(ID) {};
    void init(ConfigParser config, double xRange, double yRange);
    void reinitialize();
    cv::Mat predict();
    cv::Mat correct();
    double estimateHit(cv::Mat img, cv::Mat mask);
    std::vector<CMeasurement>* getMeasurements() {return &measurements;}
    cv::Scalar getBalloonMean() {return balloonMean;}
    void drawParticles(cv::Mat *image);
};

#endif