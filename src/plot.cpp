#include "plot.h"
#include "util.h"

#include "gnuplot/gnuplot_i.hpp"

#include <iostream>

BalloonPlot::BalloonPlot(int id, double xRange, double yRange, double zRange, int points) {
    this->id = id;
    this->points = points;
    Gnuplot* gpp = new Gnuplot("lines");
    gp = (void*) gpp;
    gpp->set_title(concat("Balloon ", id+1));
    gpp->set_xlabel();
    gpp->set_ylabel();
    gpp->set_zlabel();
    gpp->set_xrange(0, xRange);
    gpp->set_yrange(0, yRange);
    gpp->set_zrange(0, zRange);
    gpp->showonscreen();
}

void BalloonPlot::update(double x, double y, double z) {
    xs.push_back(x);
    ys.push_back(y);
    zs.push_back(z);
    
    if (xs.size() > points) {
        xs.erase(xs.begin());
        ys.erase(ys.begin());
        zs.erase(zs.begin());
    }
    
    Gnuplot* gpp = (Gnuplot*) gp;
    gpp->reset_plot();
    gpp->plot_xyz(xs, ys, zs, "balloon movement");
}

BalloonPlot::~BalloonPlot() {
    Gnuplot* gpp = (Gnuplot*) gp;
    gpp->remove_tmpfiles();
    delete gpp;
}