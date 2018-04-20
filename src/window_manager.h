/* 
 * File:   window_manager.h
 * Author: Karlo
 *
 * Created on October 28, 2015, 4:33 PM
 */

#ifndef WINDOW_MANAGER_H
#define	WINDOW_MANAGER_H

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <cstring>

/**
 * Class which represents a window and offers some methods to operate with it.
 */
class MyWindow {
    char* name;
    int x, y, maxW;
public:
    /**
     * 
     * @param N Window name.
     * @param X Upper left corner x coordinate.
     * @param Y Upper left corner y coordinate.
     * @param W Maximum width of the window. Bigger frames will be scaled to this width.
     */
    MyWindow(const char* N, int X, int Y, int W) : x(X), y(Y), maxW(W) {
        name = (char*) malloc(strlen(N) + 1);
        strcpy(name, N);
    }
    
    /**
     * Initializes the window with a desired size.
     * @param w Width of the window.
     * @param h Height of the window.
     */
    void init(int w, int h);
    
    /**
     * Shows the given image in the window.
     * @param img Image to show.
     */
    void showImage(cv::Mat img);
    
    /**
     * Destroys the window.
     */
    void destroy();
    
    /**
     * Sets the mouse callback for the window.
     * @param callback The callback function.
     * @param data Additional user data sent to the callback function.
     */
    void setMouseCallback(CvMouseCallback callback, void* data);
};

#endif	/* WINDOW_MANAGER_H */

