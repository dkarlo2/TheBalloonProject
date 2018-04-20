#ifndef PLOT_H
#define PLOT_H

#include <vector>

class BalloonPlot {
    int id;
    std::vector<double> xs;
    std::vector<double> ys;
    std::vector<double> zs;
    void *gp;
    int points;
public:
    BalloonPlot(int id, double xRange, double yRange, double zRange, int points);
    void update(double x, double y, double z);
    ~BalloonPlot();
};

#endif