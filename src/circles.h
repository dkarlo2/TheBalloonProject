#ifndef CIRCLES_H
#define CIRCLES_H

#include "condensation.h"
#include "config_parser.h"

#include "opencv2/core/core.hpp"

#include <vector>

void init_circles(ConfigParser config);

void update_circles(cv::Mat motionImg, cv::Mat originalImg, std::vector<Condensation*> filters, std::vector<cv::Rect> regions);

#endif