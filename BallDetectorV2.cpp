﻿// BallDetectorV2.cpp: определяет точку входа для приложения.
//
#include <fstream>
#include <iostream>
#include <cmath>
#include "BallDetectorV2.h"
#include <opencv2/calib3d.hpp> // Undistort and Rodrigues 
#include <opencv2/imgproc.hpp> //FindCont
#include <opencv2/imgcodecs.hpp> //Imread
#include <opencv2/highgui.hpp> // Waitkey
#include <nlohmann/json.hpp>
#include <typeinfo>


using json = nlohmann::json; // synonim for data type nlohmann::json

using namespace cv;
using namespace std;
#include <opencv2/core/utils/logger.hpp>


void constructColorFilter(Mat& pts, Mat& v, Scalar& p0, double& t1, double& t2, double& R) // Calculates coefficients (cylinder) from the passed points of the same color (pts)
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

Mat useColorFilter(Mat& img, Mat& v, Scalar& p0, double& t1, double& t2, double& R, int& nx, int& ny)  // Applies previously obtained coefficients and builds an alpha mask on the image.
{
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
    Mat d_maska(ny, nx, CV_8U);
    for (int y = 0; y < ny; ++y)
    {
        for (int x = 0; x < nx; ++x)
        {
            d_maska.at<uchar>(y, x) = d_mask.at<uchar>(y * nx + x);
        }
    }
    return d_maska;
}


void writer(vector<Point2f>& text) {
    std::ofstream out;          // поток для записи
    out.open("C:\\log\\UndistortHome.txt"); // окрываем файл для записи
    //out.open("C://Users//Student//DenisV//kurs//BallDetectorV2\\Undistort.txt");
    if (out.is_open())
    {
        out << text;
    }

    std::cout << "File save" << std::endl;
}
void writer(vector<Point3f>& text) {
    std::ofstream out;          // поток для записи
    out.open("C:\\log\\UndistortHome.txt"); // окрываем файл для записи
    //out.open("C://Users//Student//DenisV//kurs//BallDetectorV2\\Undistort.txt");
    if (out.is_open())
    {
        out << text;
    }
    std::cout << "File save" << std::endl;
}

void converter2To3(vector<Point2f>& src, vector <Point3f>& dst ) {
    for (int i = 0; i < src.size(); ++i) {
        dst.push_back(Point3f (src[i].x, src[i].y,1));
    }
}
void normalizeVectors(vector<Point3f>& src, vector <Point3f>& dst) {
    for (int i = 0; i < src.size(); ++i)
    {
        double anorm = pow(pow(src[i].x, 2) + pow(src[i].y, 2) + pow(src[i].z, 2), 0.5);
        Point3f vi = src[i] / anorm;
        dst.push_back(vi);
    }
}




void makePlane(Point3f& ptr1, Point3f& ptr2, Point3f& ptr3, float* plane)
{
    float A, B, C, D, Anorm, Bnorm, Cnorm, Dnorm;
    float lambdaA = (ptr1.z - ptr2.z) * (ptr3.y - ptr1.y) - (ptr1.z - ptr3.z) * (ptr2.y - ptr1.y);
    float lambdaB = (ptr1.z - ptr3.z) * (ptr2.x - ptr1.x) - (ptr1.z - ptr2.z) * (ptr3.x - ptr1.x);
    float lambda = (ptr2.x - ptr1.x) * (ptr3.y - ptr1.y) - (ptr3.x - ptr1.x) * (ptr2.y - ptr1.y);
    if (lambda != 0)
    {
        A = lambdaA / lambda;
        B = lambdaB / lambda;
        C = 1.;
        D = -A * ptr1.x - B * ptr1.y - C * ptr1.z;
        Anorm = A / sqrt(A * A + B * B + 1);
        Bnorm = B / sqrt(A * A + B * B + 1);
        Cnorm = C / sqrt(A * A + B * B + 1);
        Dnorm = D / sqrt(A * A + B * B + 1);
    }
    else
    {
        if (ptr2.x - ptr1.x != 0)
        {
            A = (ptr1.y - ptr2.y) / (ptr2.x - ptr1.x);
            B = 1.;
            C = 0.;
            D = -A * ptr1.x - B * ptr1.y - C * ptr1.z;
            Anorm = A / sqrt(A * A + 1);
            Bnorm = B / sqrt(A * A + 1);
            Cnorm = C / sqrt(A * A + 1);
            Dnorm = D / sqrt(A * A + 1);
        }
        else
        {
            Anorm = 1;
            Bnorm = 0;
            Cnorm = 0;
            Dnorm = -ptr1.x;
        }
    }
    plane[0] = Anorm;
    plane[1] = Bnorm;
    plane[2] = Cnorm;
    plane[3] = Dnorm;
}
float upPlane(float* plane, float& k, Point3f& ptr)
{
    float a = plane[0];
    float b = plane[1];
    float c = plane[2];
    float d = plane[3];
    return a * (ptr.x - k * a) + b * (ptr.y - k * b) + c * (ptr.z - k * c) + d;
}

float downPlane(float* plane, float& k, Point3f& ptr)
{
    float a = plane[0];
    float b = plane[1];
    float c = plane[2];
    float d = plane[3];
    return a * (ptr.x + k * a) + b * (ptr.y + k * b) + c * (ptr.z + k * c) + d;
}
float counterPlane(float* plane, float& k, vector<Point3f>& ptr)
{
    int n = ptr.size();
    int coun = 0;
    for (int i = 0; i < n; ++i)
    {
        if (upPlane(plane, k, ptr[i]) <= 0 && downPlane(plane, k, ptr[i]) >= 0)
        {
            ++coun;
        }
    }
    float counter = float(coun) / n;
    return counter;
}
void findPlane(vector<Point3f>& ptr, float* max_plane, float& k, float& abs_counter, int& abs_iter)
{
    float max_counter = 0.;
    float counter = 0.;
    int iter = 0;
    int n = ptr.size();
    float plane[4];

    while (max_counter < abs_counter && iter < abs_iter)
    {
        Point3f ptr1 = ptr[iter];
        Point3f ptr2 = ptr[iter + n / 3];
        Point3f ptr3 = ptr[iter + n * 2 / 3];
        makePlane(ptr1, ptr2, ptr3, plane);
        counter = counterPlane(plane, k, ptr);
        //cout << counter << endl;
        if (counter > max_counter)
        {
            max_counter = counter;
            max_plane[0] = plane[0];
            max_plane[1] = plane[1];
            max_plane[2] = plane[2];
            max_plane[3] = plane[3];
        }
        ++iter;
    }
    cout << "max counter" << max_counter << endl;
    cout << "iter" << iter << endl;
}


void inPoint(float* plane, vector<Point3f>& ptr, vector<Point3f>& inptr, float& k)
{
    int n = ptr.size();
    for (int i = 0; i < n; ++i)
    {
        if (upPlane(plane, k, ptr[i]) <= 0 && downPlane(plane, k, ptr[i]) >= 0)
        {
            inptr.push_back(ptr[i]);
        }
    }
}

void inPointPaint(vector<Point3f>& MyCord, vector<Point2f>& PixCord, Mat& img, float* plane, float& k) { // Input va and plant
    for (int i = 0; i < MyCord.size(); ++i)
    {
        if (upPlane(plane, k, MyCord[i]) <= 0 && downPlane(plane, k, MyCord[i]) >= 0)
        {
            img.at<Vec3b>(PixCord[i])[0] = 0;
            img.at<Vec3b>(PixCord[i])[1] = 255;
            img.at<Vec3b>(PixCord[i])[2] = 0;
        }
        else {
             //img.at<Vec3b>(PixCord[i]) = (255,0,255,255); //int b =//int g =//int r =
            img.at<Vec3b>(PixCord[i])[0] = 255;
            img.at<Vec3b>(PixCord[i])[1] = 0;
            img.at<Vec3b>(PixCord[i])[2] = 0;
        }
    }
    //imshow("Obvodka", img);
    //imwrite("gran.bmp", img);
}
void BallPixSize(vector<Point2f> grad) {
    int maxX = 0;
    int minX = 2000;
    int maxY = 0;
    int minY = 2000;
    for (int i = 0; i < grad.size(); ++i) {
        if (grad[i].x > maxX) {
            maxX = grad[i].x;
        }
        if (grad[i].x < minX) {
            minX = grad[i].x;
        }
        if (grad[i].y > maxY) {
            maxY = grad[i].y;
        }
        if (grad[i].y < minY) {
            minY = grad[i].y;
        }
    }
    cout << "razmery  " << minX << "x" << maxX << "  " << minY << "x" << maxY << endl;
    cout << ((maxX - minX) + (maxY - minY)) / 2 << endl;
}
float SumErrPlane(vector<Point3f> iva_norm, float* max_plane) {
    float SumErr = 0;
    for (int i = 0; i < iva_norm.size(); ++i) {
        SumErr += abs (max_plane[0] * iva_norm[i].x + max_plane[1] * iva_norm[i].y + max_plane[2] * iva_norm[i].z + max_plane[3]);
        cout << SumErr << endl;
    }
    return SumErr;
}

int main(int argc, char* argv[])
{
    //cv::utils::logging::setLogLevel(cv::utils::logging::LOG_LEVEL_SILENT);

    long int timer1 = getTickCount();
    //read json file
    std::ifstream file("../../../intrinsics0.json");
    json  intrinsics = json::parse(file)["intrinsics"];
    file.close();
    //get verb from json
    std::vector<double> tempVerb1 = intrinsics["K"];
    //for (auto const& i : temp) {
        //std::cout << i << " ";
    //}

    cv::Matx<double, 3, 3> cameraMatrix(tempVerb1.data()); // initialize from plain array
    std::vector<double> tempVerb2 = intrinsics["distortion"];
    cv::Vec<double, 5> distCoeffs(tempVerb2.data());
    Mat pts = imread("..\\..\\..\\..\\BallDetectorData\\2DMoveData\\ColorCut.bmp", 1);
    if (pts.rows == 0 || pts.cols == 0) {
        cout << "Color example Not Found not found" << endl;
        return 0;
    }
    Scalar p0;
    Mat v(1, 3, CV_64F);
    double t1, t2, R;
    constructColorFilter(pts, v, p0, t1, t2, R); 
    vector<Point3f> resultsCord;
    for (int cikle = 0; cikle < 10; ++cikle) {
        //Mat img = imread("C:\\Users\\Student\\DenisV\\kurs\\OctBall\\BallDetector\\Data\\2DMoveData\\" + to_string(cikle) + ".bmp", 1);
        Mat img = imread("..\\..\\..\\..\\BallDetectorData\\2DMoveData\\5.png", 1);

        //erode(img, img, Mat(), Point(-1, -1), 5);
        //dilate(img, img, Mat(), Point(-1, -1), 5);
        if (img.rows == 0 || img.cols == 0) {
            cout << "Picture not found" << endl;
            return 0;
        }
        int ny = img.rows;
        int nx = img.cols;
        Mat Gray_mask = useColorFilter(img, v, p0, t1, t2, R, nx, ny);
        if (Gray_mask.rows == 0 || Gray_mask.cols == 0) {
            cout << "Object not found on the picture" << endl;
            return 0;
        }
        Mat BinaryMask(ny, nx, CV_8U, Scalar(0));
        //erode(Gray_mask, Gray_mask, Mat(), Point(-1, -1), 5);
        //dilate(Gray_mask, Gray_mask, Mat(), Point(-1, -1), 5);
        cv::threshold(Gray_mask, BinaryMask, 20, 255, cv::THRESH_BINARY_INV);
        imwrite("Gray.png", Gray_mask);
        imwrite("Bin.png", BinaryMask);
        vector < vector<Point> > gradcv;
        //cout << "Work point" << " nx   " << nx << "  ny  " << ny << endl;
        cv::findContours(BinaryMask, gradcv, noArray(), cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
        vector<Point2f> gradcvConv(gradcv[0].begin(), gradcv[0].end());

        //BallPixSize(gradcvConv);
        drawContours(img, gradcv, -1, (0, 255, 255), 1);
        vector<Point2f> grad2;
        //Mat cameraMatrix = (Mat_<double>(3, 3) << 2666.6666666666665, 0, 960.0, 0, 2666.6666666666665, 540.0, 0, 0, 1);
        vector<float> distCoeffs = { 0,0,0,0 };
        Mat P = (Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);//New "ideal" cameramatrix
        Mat Rx = (Mat_<double>(3, 3) << -1, 0, 0, 0, -1, 0, 0, 0, 1);
        cv::Mat rvecR(3, 1, CV_64F);//rodrigues rotation matrix
        cv::Rodrigues(Rx, rvecR);
        cv::undistortPoints(gradcvConv, grad2, cameraMatrix, distCoeffs, Rx, P = P); // точки без искажений и равные метрам
        vector <Point3f> a;
        converter2To3(grad2, a);
        vector <Point3f> v_norm;
        normalizeVectors(a, v_norm);
        //cout << v_norm << endl;
        vector<Point3f> iva_norm; // Points in finded plane
        float max_plane[4] = { 0., 0., 0., 0. };
        float k = 0.000008; // Distants between plane
        float abs_counter = 0.95;
        int abs_iter = v_norm.size() / 3; // 
        findPlane(v_norm, max_plane, k, abs_counter, abs_iter); // Find plane
        inPoint(max_plane, v_norm, iva_norm, k);
        inPointPaint(v_norm, gradcvConv, img, max_plane, k);
        //writer(iva_norm);
        //cout << "SumErr " << SumErrPlane(iva_norm, max_plane) << endl;
        //cout << " Plane Coef: " << max_plane[0] << "   " << max_plane[1] << "   " << max_plane[2] << "  " << max_plane[3] << endl;

        //find the cone apex angele 
        double theta;
        double normaNormali = pow(pow(max_plane[0], 2) + pow(max_plane[1], 2) + pow(max_plane[2], 2), 0.5);
        double sumarc = 0.0;
        for (int i = 0; i < iva_norm.size(); ++i) {
            sumarc += acos((max_plane[0] * iva_norm[i].x + max_plane[1] * iva_norm[i].y + max_plane[2] * iva_norm[i].z) / normaNormali);
        }
        theta = 2 * (sumarc / iva_norm.size());
        //cout << "theta = " << theta << endl;
        float RadiusBall = 1.0;
        double distans = RadiusBall / (sin(theta / 2));
        Point3f ballCoordinates;
        ballCoordinates.x = (distans / normaNormali) * max_plane[0];
        ballCoordinates.y = (distans / normaNormali) * max_plane[1];
        ballCoordinates.z = (distans / normaNormali) * max_plane[2];
        cout << " Image #" << cikle << endl;
        cout << "result = " << ballCoordinates << endl;
        //cout << "distance = " << distans << endl;
        resultsCord.push_back(ballCoordinates);

        //projection true coordinate 
        vector<cv::Point2f> ProjectPoints2;
        vector<cv::Point3f> TrueCoord;
        TrueCoord.push_back(Point3f(5.34507, 4.60824, 30.0));
        cv::Mat T(3, 1, CV_64F, Scalar(0)); // translation vector

        cv::projectPoints(TrueCoord, rvecR, T, cameraMatrix, distCoeffs, ProjectPoints2);
        cout << "ProjectPoints True coordinate:    " << ProjectPoints2[0] << endl;
        ProjectPoints2[0].x = round(ProjectPoints2[0].x);
        ProjectPoints2[0].y = round(ProjectPoints2[0].y);

        img.at<Vec3b>(ProjectPoints2[0])[2] = 0;  //sea color
        img.at<Vec3b>(ProjectPoints2[0]) [0] = 255;
        img.at<Vec3b>(ProjectPoints2[0])[1] = 255;

        //projection find coordinate 
        vector<cv::Point3f> FindCord;
        FindCord.push_back(Point3f(5.36955, 4.6314, 30.0885));
        cv::projectPoints(FindCord, rvecR, T, cameraMatrix, distCoeffs, ProjectPoints2);
        cout << "ProjectPoints find coordinate:    " << ProjectPoints2[0] << endl;
        ProjectPoints2[0].x = round(ProjectPoints2[0].x);
        ProjectPoints2[0].y = round(ProjectPoints2[0].y);

        img.at<Vec3b>(ProjectPoints2[0])[2] = 255; // yellow color
        img.at<Vec3b>(ProjectPoints2[0])[0] = 0;
        img.at<Vec3b>(ProjectPoints2[0])[1] = 255;




        imwrite("ConturAndCenterImg.png", img);
    }   
    writer(resultsCord);
    long int timer2 = getTickCount();
    double finalTime = (timer2 - timer1) / getTickFrequency();
    cout << "Programm complete " << finalTime << " sec" << endl;
    waitKey(0);
	return 0;
}
