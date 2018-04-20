#ifndef UTIL_H
#define	UTIL_H

#define CROSS_SIZE 5 // 2
#define CIRCLE_SIZE 15 // 10
#define CROSS_T 1 // 1
#define CIRCLE_T 2

#define drawCross( img, center, color, d )                                 \
    line( img, cv::Point( center.x - d, center.y - d ), cv::Point( center.x + d, center.y + d ), color, CROSS_T, CV_AA, 0); \
    line( img, cv::Point( center.x + d, center.y - d ), cv::Point( center.x - d, center.y + d ), color, CROSS_T, CV_AA, 0 )

const char* concat(const char* pref, int id);

bool is_number(const char *ss);

#endif

