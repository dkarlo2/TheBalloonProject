#include "calibrate.h"
#include "circles.h"
#include "config_parser.h"
#include "plot.h"
#include "send_osc.h"
#include "superior_kalman.h"
#include "util.h"
#include "video_tracker.h"

#include "opencv2/highgui/highgui.hpp"

#include <chrono>
#include <iostream>
#include <string>
#include <vector>

#define ESC 27

static double xAxisRatio;
static double yAxisRatio;
static double zAxisRatio;

static BalloonPlot **plots;

static void process_estimated_states(int nBalloons, VideoTracker *tracker1, VideoTracker *tracker2,
            std::vector<SuperiorKalman> *supKalmans, MyOSCSender *sender) {
    for (int i = 0; i < nBalloons; i++) {
        double xe1 = tracker1->getStateX(i) * xAxisRatio;
        double ye1 = tracker1->getStateY(i) * yAxisRatio;
        double xe2 = tracker2->getStateX(i) * xAxisRatio;
        double ye2 = tracker2->getStateY(i) * yAxisRatio;

        double U = xe1;
        double V = (ye1 + ye2)/2;
        if (xe2 == xe1) {
            xe2 = xe1 + xAxisRatio;
        }
        double W = zAxisRatio / abs(xe2-xe1);

        supKalmans->operator[](i).predict();
        cv::Mat position = supKalmans->operator[](i).correct((cv::Mat_<float>(3,1) << U, V, W));
        float x = position.at<float>(0, 0);
        float y = position.at<float>(1, 0);
        float z = position.at<float>(2, 0);
        sender->sendPosition(i, x, y, z);
        // sender->sendPosition(i, U, V, W);
        
        // std::cout << "p1 " << U << ' ' << V << ' ' << W << " p2 " << x << ' ' << y << ' ' << z << std::endl;
        
        if (plots[i] != NULL) {
            plots[i]->update(x, y, z);
        }
    }
}

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cout << "Program expects exactly one argument, path to the configuration file." << std::endl;
        return -1;
    }

    ConfigParser config;
    config.parse(argv[1]);

    int calib = config.getInt("calibrate");
    if (calib) {
        calibrate(config);
    }

    int nBalloons = config.getInt("nBalloons");
    
    xAxisRatio = config.getDouble("xAxisRatio");
    yAxisRatio = config.getDouble("yAxisRatio");
    zAxisRatio = config.getDouble("zAxisRatio");
    
    init_circles(config);
    
    MyOSCSender sender(config);
    
    std::vector<SuperiorKalman> supKalmans(nBalloons, SuperiorKalman(config));
    
    VideoTracker tracker1(0);
    VideoTracker tracker2(1);
    
    tracker1.init(config);
    tracker2.init(config);
    
    double sw = tracker1.getScaledWidth();
    double sh = tracker1.getScaledHeight();
    if (abs(tracker2.getScaledWidth()-sw) >= 1 || abs(tracker2.getScaledHeight()-sh) >= 1) {
        std::cerr << "Video frames need to have the same size ratio." << std::endl;
        return -1;
    }
    
    plots = new BalloonPlot*[nBalloons];
    int balloonPoints = config.getInt("balloonPoints");
    for (int i = 0; i < nBalloons; i++) {
        if (config.getInt(concat("showBalloonPlot", i+1))) {
            plots[i] = new BalloonPlot(i, sw*xAxisRatio, sh*yAxisRatio, zAxisRatio/xAxisRatio, balloonPoints);
        } else {
            plots[i] = NULL;
        }
    }
    
    std::chrono::milliseconds timeStart;
    
    while (1) {
        timeStart = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());

        int t1 = tracker1.next_frame();
        int t2 = tracker2.next_frame();

        if (t1 == -1 || t2 == -1) {
            return -1;
        }

        if (t1 || t2) {
            break;
        }

        process_estimated_states(nBalloons, &tracker1, &tracker2, &supKalmans, &sender);
        
        int diff = (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()) - timeStart).count();
        // std::cout << "Time: " << diff << std::endl;
        
        bool exit = false;
        switch (cv::waitKey(1)) {
            case ESC:
                exit = true;
                break;
        }

        if (exit) {
            break;
        }
    }
    
    for (int i = 0; i < nBalloons; i++) {
        if (plots[i] != NULL) {
            delete plots[i];
        }
    }
    delete plots;
}