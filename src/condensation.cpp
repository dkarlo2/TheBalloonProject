#include "condensation.h"
#include "util.h"

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <iostream>
#include <random>
#include <string>

#define M_PI 3.14159265358979323846

static default_random_engine generator;
static normal_distribution<double> *normal_pos;
static normal_distribution<double> *normal_vel;
static normal_distribution<double> *normal_acc;
static uniform_real_distribution<double> uniform_acc;

static int draw_number;

static int newParticles;
static double airRestistance;
static double gravity;
static double accReduction;
static double processSigmaPos;
static double processSigmaVel;
static double processSigmaAcc;
static double measurementSigma;
static double maxAcc;
static double randomHit;

static double darkCircleThreshold;

void Condensation::init(ConfigParser config, double xRange, double yRange) {
    n = config.getInt("nParticles");
    this->xRange = xRange;
    this->yRange = yRange;
    
    draw_number = config.getInt("particleDrawNumber");
    newParticles = config.getInt("nNewParticles");
    airRestistance = config.getDouble("airResistance");
    gravity = config.getDouble("gravity");
    accReduction = config.getDouble("accelerationReduction");
    processSigmaPos = config.getDouble("processSigmaPosition");
    processSigmaVel = config.getDouble("processSigmaVelocity");
    processSigmaAcc = config.getDouble("processSigmaAcceleration");
    measurementSigma = config.getDouble("measurementSigma");
    darkCircleThreshold = config.getDouble("darkCircleThreshold");
    maxAcc = config.getDouble("maxAcceleration");
    randomHit = config.getDouble("processRandomHit");
    
    const char* balloonFile = config.getString(concat("balloonImageFile", id+1));
    
    // store the balloon color mean
    cv::Mat balloonImage = cv::imread(balloonFile, CV_LOAD_IMAGE_COLOR);
    cv::Mat balloonImageGray;
    cvtColor(balloonImage, balloonImageGray, CV_BGR2GRAY);
    std::vector<cv::Point> locations;
    findNonZero(balloonImageGray, locations);
    cv::Mat mask = cv::Mat::zeros(balloonImage.size(), CV_8UC1);
    for (int i = 0; i < locations.size(); i++) {
        mask.at<char>(locations[i].y, locations[i].x) = 255;
    }
    balloonMean = mean(balloonImage, mask);
    
    normal_pos = new normal_distribution<double>(0, processSigmaPos);
    normal_vel = new normal_distribution<double>(0, processSigmaVel);
    normal_acc = new normal_distribution<double>(0, processSigmaAcc);

    srand(time(NULL));
    
    reinitialize();
    
    initialized = true;
}

void Condensation::reinitialize() {
    particles = new CParticle[n];
    double c = 0;
    for (int i = 0; i < n; i++) {
        particles[i].x = (double) rand() * xRange / RAND_MAX;
        particles[i].y = (double) rand() * yRange / RAND_MAX;
        particles[i].vx = 0;
        particles[i].vy = 0;
        particles[i].accx = 0;
        particles[i].accy = 0;
        particles[i].weight = 1. / n;
        c += 1. / n;
        particles[i].c = c;
    }

    timeStamp = clock();
}

int sgn(double d) {
    return d < 0 ? -1 : 1;
}

// move particles
void Condensation::applyDynamics() {
    clock_t now = clock();
    clock_t diff = now - timeStamp;
    timeStamp = now;

    double dt = (double) diff / CLOCKS_PER_SEC;

    for (int i = 0; i < n; i++) {
        particles[i].x += particles[i].vx * dt + normal_pos->operator()(generator);
        particles[i].y += particles[i].vy * dt + normal_pos->operator()(generator);
        particles[i].vx *= pow(1-airRestistance, dt);
        particles[i].vy *= pow(1-airRestistance, dt);
        particles[i].vx += particles[i].accx * dt /*+ normal_vel->operator()(generator)*/;
        particles[i].vy += particles[i].accy * dt + gravity * dt /*+ normal_vel->operator()(generator)*/;
        particles[i].accx *= pow(accReduction, dt);
        particles[i].accy *= pow(accReduction, dt);
        
        double probx = abs(xRange/2 - particles[i].x) / (xRange/2);
        double proby = abs(yRange/2 - particles[i].y) / (yRange/2);
        if (uniform_acc(generator) < probx*probx*probx) {
            int s = sgn(xRange/2 - particles[i].x);
            particles[i].accx += abs(normal_acc->operator()(generator)) * s;
        }
        if (uniform_acc(generator) < proby*proby*proby) {
            int s = sgn(yRange/2 - particles[i].y);
            particles[i].accy += abs(normal_acc->operator()(generator)) * s;
        }
        if (abs(particles[i].accx) > maxAcc) {
            particles[i].accx = sgn(particles[i].accx) * maxAcc;
        }
        if (abs(particles[i].accy) > maxAcc) {
            particles[i].accy = sgn(particles[i].accy) * maxAcc;
        }
        
        if (uniform_acc(generator) < randomHit) {
            particles[i].accx = 0;
            particles[i].accy = 0;
            particles[i].vx = normal_vel->operator()(generator);
            particles[i].vy = normal_vel->operator()(generator);
        }
    }
}

static double normal2d_pdf(double x, double y) {
    return 1./(2*M_PI*measurementSigma) * exp(-1./2/measurementSigma * (x*x + y*y));
}

cv::Mat Condensation::predict() {
    if (!initialized) {
        std::cerr << "Filter not initialized!" << std::endl;
        return cv::Mat_<float>(2, 1) << -1, -1;
    }
    
    CParticle* newParticles = new CParticle[n];
    
    double c = 0;
    for (int p = 0; p < n; p++) {
        double r = (double) rand() / (RAND_MAX+1);
        int m = findParticleByR(r, 0, n-1);
        newParticles[p] = particles[m];
        c += newParticles[p].weight;
        newParticles[p].c = c;
    }

    delete particles;

    particles = newParticles;

    applyDynamics();

    for (int p = 0; p < n; p++) {
        particles[p].weight /= c;
        particles[p].c /= c;
    }

    pred = getStateEstimate(0);
    return pred;
}

/*
 * Calculate the correlation between the found circle color mean and the stored balloon color mean,
 * and scale it to the [0,1] interval.
*/
double Condensation::estimateHit(cv::Mat img, cv::Mat mask) {
    cv::Scalar avg1 = mean(img, mask);
    cv::Scalar avg2 = balloonMean;

    if (avg1.val[0] < darkCircleThreshold && avg1.val[1] < darkCircleThreshold
            && avg1.val[2] < darkCircleThreshold) {
        return 0;
    }

    double mean1 = (avg1.val[0] + avg1.val[1] + avg1.val[2]) / 3;
    double mean2 = (avg2.val[0] + avg2.val[1] + avg2.val[2]) / 3;

    double sum12 = (avg1.val[0] - mean1) * (avg2.val[0] - mean2) + 
                    (avg1.val[1] - mean1) * (avg2.val[1] - mean2) +
                    (avg1.val[2] - mean1) * (avg2.val[2] - mean2);

    double sum11 = (avg1.val[0] - mean1) * (avg1.val[0] - mean1) + 
                    (avg1.val[1] - mean1) * (avg1.val[1] - mean1) +
                    (avg1.val[2] - mean1) * (avg1.val[2] - mean1);

    double sum22 = (avg2.val[0] - mean2) * (avg2.val[0] - mean2) + 
                    (avg2.val[1] - mean2) * (avg2.val[1] - mean2) +
                    (avg2.val[2] - mean2) * (avg2.val[2] - mean2);

    double corr = sum12 / sqrt(sum11 * sum22);
    
    return (corr + 1) / 2;
}

cv::Mat Condensation::correct() {
    if (!initialized) {
        cerr << "Filter not initialized!" << endl;
        return cv::Mat_<float>(2, 1) << -1, -1;
    }
    
    if (measurements.size() == 0) {
        return pred;
    }

    injectNewParticles();

    for (int p = 0; p < n; p++) {
        particles[p].weight = 0;
    }

    for (int i = 0; i < measurements.size(); i++) {
        double mx = measurements[i].x;
        double my = measurements[i].y;
        double mw = measurements[i].lifes * measurements[i].w / measurements[i].maxAge;
        
        for (int p = 0; p < n; p++) {
            double w = mw * normal2d_pdf(mx - particles[p].x, my - particles[p].y);
            particles[p].weight = max(particles[p].weight, w);
        }
    }

    double cTotal = 0;
    for (int p = 0; p < n; p++) {
        cTotal += particles[p].weight;
    }

    if (cTotal == 0) {
        reinitialize();
    } else {
        double c = 0;
        for (int p = 0; p < n; p++) {
            particles[p].weight /= cTotal;
            c += particles[p].weight;
            particles[p].c = c;
        }
    }

    return getStateEstimate(0);
}

// add random particles that fill find a circle if it suddenly appears somewhere else in the image
void Condensation::injectNewParticles() {
    for (int i = 0; i < newParticles; i++) {
        CParticle particle;
        particle.x = (double) rand() / RAND_MAX * xRange;
        particle.y = (double) rand() / RAND_MAX * yRange;
        particle.vx = normal_vel->operator()(generator);
        particle.vy = normal_vel->operator()(generator);
        particle.accx = 0;
        particle.accy = 0;
        
        double r = (double) rand() / (RAND_MAX+1);
        int m = findParticleByR(r, 0, n-1);
        particles[m] = particle;
    }
}

// function that finds a particle specified by a double value r in respect to its weight
int Condensation::findParticleByR(double r, int s, int e) {
    if (s == e) {
        return s;
    }
    int m = (s+e) / 2;
    if (particles[m].c < r) {
        return findParticleByR(r, m+1, e);
    } else if (particles[m].c >= r && (m == 0 || particles[m-1].c < r)) {
        return m;
    } else {
        return findParticleByR(r, s, m-1);
    }
}

// two versions of the balloon position estimation
cv::Mat Condensation::getStateEstimate(int mode) {
    if (mode == 0) {
        double cx = 0;
        double cy = 0;
        for (int p = 0; p < n; p++) {
            cx += particles[p].weight * particles[p].x;
            cy += particles[p].weight * particles[p].y;
        }
        return cv::Mat_<float>(2, 1) << cx, cy;
    } else if (mode == 1) {
        int i = -1;
        double maxWeight = 0;
        for (int p = 0; p < n; p++) {
            if (particles[p].weight > maxWeight) {
                maxWeight = particles[p].weight;
                i = p;
            }
        }
        return cv::Mat_<float>(2, 1) << particles[i].x, particles[i].y;
    }
}

void Condensation::drawParticles(cv::Mat *image) {
    if (!initialized) {
        return;
    }
    for (int i = 0; i < n; i += n/draw_number) {
        cv::Point partPt(particles[i].x, particles[i].y);
        drawCross((*image), partPt , cv::Scalar(255,0,255), CROSS_SIZE);
    }
}
