#pragma once

#include "ColorFilter.h"
#include <opencv2/opencv.hpp>
#include <cmath>


using namespace cv;
using namespace std;

void ColorFilter::init() {
    v = cv::Mat(1, 3, CV_64F);
}
void ColorFilter::get_ColorPoints(std::string& path) {
    ColorPoints = imread(path, 1);
    if (ColorPoints.rows == 0 || ColorPoints.cols == 0) {
        cout << "Color example Not Found not found" << endl;
        exit(0);
    }
}
void ColorFilter::constructColorFilter()
{
    p0 = mean(ColorPoints);
    Mat di;
    ColorPoints.convertTo(di, CV_64FC3);
    di = di - p0;
    Mat D(3, 3, CV_64F, Scalar(0));
    for (int y = 0; y < di.rows; ++y)
    {
        D += (di.at<Vec3d>(y) * di.at<Vec3d>(y).t());
    }
    Mat U, s, Vt;
    SVD::compute(D, s, U, Vt);

    for (int x = 0; x < 3; ++x) {
        v.at<double>(0, x) = Vt.at<double>(2, x);
    }

    Mat ti, Ri;
    for (int x = 0; x < di.rows; ++x)
    {
        ti.push_back(di.at<Vec3d>(x).t() * v.at<Vec3d>(0, 0));
        Ri.push_back(cv::norm(ti.at<double>(x) * v.at<Vec3d>(0, 0) - di.at<Vec3d>(x)));
    }

    Mat ts, Rs;
    cv::sort(ti, ts, SORT_ASCENDING + SORT_EVERY_COLUMN);
    cv::sort(Ri, Rs, SORT_ASCENDING + SORT_EVERY_COLUMN);
    int t1_ind = ti.rows / 20;
    int t2_ind = ti.rows * 19 / 20;
    int R_ind = Ri.rows * 19 / 20;
    t1 = ts.at<double>(t1_ind);
    t2 = ts.at<double>(t2_ind);
    R = Rs.at<double>(R_ind);
}

void ColorFilter::useColorFilter(Mat& img, Mat& d_maska)
{
    int ny = img.rows;
    int nx = img.cols;
    int nc = 3;
    Mat pi;
    for (int y = 0; y < img.rows; ++y)
    {
        for (int x = 0; x < img.cols; ++x)
        {
            pi.push_back(img.at<Vec3b>(y, x));
        }
    }

    Mat di;
    pi.convertTo(di, CV_64FC3);
    di = di - p0;
    Mat t, dp;
    for (int x = 0; x < di.rows; ++x)
    {
        t.push_back(di.at<Vec3d>(x).t() * v.at<Vec3d>(0, 0));
        dp.push_back(cv::norm(t.at<double>(x) * v.at<Vec3d>(0, 0) - di.at<Vec3d>(x)));
    }
    Mat dt = cv::abs(t - (t1 + t2) / 2) - (t2 - t1) / 2;
    dt = cv::max(dt, 0);
    dp = cv::max(dp - R, 0);
    Mat d = dp + dt;
    Mat d_mask;
    d.convertTo(d_mask, CV_8U);
    //reshape()
    for (int y = 0; y < ny; ++y)
    {
        for (int x = 0; x < nx; ++x)
        {
            d_maska.at<uchar>(y, x) = d_mask.at<uchar>(y * nx + x);
        }
    }
}




/*
using namespace std;
using namespace cv;

void ColorFilter::constructColorFilter(Mat& pts, Mat& v, Scalar& p0, double& t1, double& t2, double& R) // Calculates coefficients (cylinder) from the passed points of the same color (pts)
{
    p0 = mean(pts);
    Mat di;
    pts.convertTo(di, CV_64FC3);
    di = di - p0;
    Mat D(3, 3, CV_64F, Scalar(0));
    for (int y = 0; y < di.rows; ++y)
    {
        D += (di.at<Vec3d>(y) * di.at<Vec3d>(y).t());
    }
    Mat U, s, Vt;
    SVD::compute(D, s, U, Vt);

    for (int x = 0; x < 3; ++x) {
        v.at<double>(0, x) = Vt.at<double>(2, x);
    }

    Mat ti, Ri;
    for (int x = 0; x < di.rows; ++x)
    {
        ti.push_back(di.at<Vec3d>(x).t() * v.at<Vec3d>(0, 0));
        Ri.push_back(cv::norm(ti.at<double>(x) * v.at<Vec3d>(0, 0) - di.at<Vec3d>(x)));
    }

    Mat ts, Rs;
    cv::sort(ti, ts, SORT_ASCENDING + SORT_EVERY_COLUMN);
    cv::sort(Ri, Rs, SORT_ASCENDING + SORT_EVERY_COLUMN);
    int t1_ind = ti.rows / 100;
    int t2_ind = ti.rows * 19 / 20;
    int R_ind = Ri.rows * 19 / 20;
    t1 = ts.at<double>(t1_ind);
    t2 = ts.at<double>(t2_ind);
    R = Rs.at<double>(R_ind);
}

*/