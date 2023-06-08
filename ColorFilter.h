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
using namespace cv;
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
    void useColorFilter(Mat& img, Mat& d_maska);  // Applies previously obtained coefficients and builds an alpha mask on the image.
};
