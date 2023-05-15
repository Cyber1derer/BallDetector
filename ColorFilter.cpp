#include "ColorFilter.h"
#include <opencv2/opencv.hpp>

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