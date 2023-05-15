#pragma once

#include <iostream>
#include <opencv2/opencv.hpp>

#include <fstream>
#include <iostream>
#include <cmath>
#include <nlohmann/json.hpp>
#include <typeinfo>
#include <opencv2/core/utils/logger.hpp>

using namespace std;

class ColorFilter
{
private:
    //ColorFilter parametrs
    cv::Scalar p0;
    cv::Mat v;
    double t1, t2, R;
    cv::Mat ColorPoints;
public:
    void init();
    void get_ColorPoints(std::string& path);
    void constructColorFilter(); // Calculates coefficients (cylinder) from the passed points of the same color (pts)
    cv::Mat useColorFilter(cv::Mat& img, int& nx, int& ny);  // Applies previously obtained coefficients and builds an alpha mask on the image.

};
