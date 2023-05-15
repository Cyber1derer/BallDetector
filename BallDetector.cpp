// BallDetectorV2.cpp: определяет точку входа для приложения.
#include <fstream>
#include <iostream>
#include <cmath>
#include <opencv2/calib3d.hpp> // Undistort and Rodrigues 
#include <opencv2/imgproc.hpp> //FindCont
#include <opencv2/imgcodecs.hpp> //Imread
#include <opencv2/highgui.hpp> // Waitkey
#include <nlohmann/json.hpp>
#include <typeinfo>
#include <opencv2/core/utils/logger.hpp>


using json = nlohmann::json; // synonim for data type nlohmann::json

using namespace cv;
using namespace std;
#include "BallDetector.h"
#include "ColorFilter.h"

#include <opencv2/core/utils/logger.hpp>


void writer(vector<Point2f>& text) {
    std::ofstream out;          // поток для записи

    out.open("C:\\log\\BallDetectorLogPoint2f.txt"); // окрываем файл для записи

    if (out.is_open())
    {
        out << text;
    }

    std::cout << "File save" << std::endl;
}

void writer(vector<Point3f>& text) {
    std::ofstream out;          // поток для записи
    out.open("C:\\log\\BallDetectorLogPoint3f.txt"); // окрываем файл для записи
    if (out.is_open())
    {
        out << text;
    }
    std::cout << "File save" << std::endl;
}

void converter2To3(vector<Point2f>& src, vector <Point3f>& dst) {
    for (int i = 0; i < src.size(); ++i) {
        dst.push_back(Point3f(src[i].x, src[i].y, 1));
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

void findPlaneVersion2(vector<Point3f>& ptr, float* max_plane, float& k, float& abs_counter, int& abs_iter) //With random point
{
    float max_counter = 0.;
    float counter = 0.;
    int iter = 0;
    int n = ptr.size();
    float plane[4];

    Point3f ptr1 = rand() % n;

    while (max_counter < abs_counter && iter < abs_iter)
    {
        Point3f ptr2 = rand() % n;
        Point3f ptr3 = rand() % n;
        makePlane(ptr1, ptr2, ptr3, plane);
        counter = counterPlane(plane, k, ptr);
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

void inPointPaint(vector<Point3f>& MyCord, vector<Point2f>& PixCord, Mat& img, float* plane, float& k) { // Input va and plant   inPointPaint(v_norm, gradcvConv, sourceImage, max_plane, k);
    for (int i = 0; i < MyCord.size(); ++i)
    {
        if (upPlane(plane, k, MyCord[i]) <= 0 && downPlane(plane, k, MyCord[i]) >= 0)
        {
            img.at<Vec3b>(PixCord[i])[0] = 0;
            img.at<Vec3b>(PixCord[i])[1] = 255;
            img.at<Vec3b>(PixCord[i])[2] = 0;
        }
        else {
            img.at<Vec3b>(PixCord[i])[0] = 255;
            img.at<Vec3b>(PixCord[i])[1] = 0;
            img.at<Vec3b>(PixCord[i])[2] = 0;
        }
    }
}

float SumErrPlane(vector<Point3f> iva_norm, float* max_plane) {
    float SumErr = 0;
    for (int i = 0; i < iva_norm.size(); ++i) {
        SumErr += abs(max_plane[0] * iva_norm[i].x + max_plane[1] * iva_norm[i].y + max_plane[2] * iva_norm[i].z + max_plane[3]);
        cout << SumErr << endl;
    }
    return SumErr;
} 

vector<Point2f> contr(Mat img, bool& ROI, Point2i pxCenterBall, int& const cropSize, ColorFilter& colorFilter) {

    int ny = img.rows;
    int nx = img.cols;
    int offsetX = 0;
    int offsetY = 0;
    Mat origImage;

    if (ROI) {

        //------------------ check range out rect-crop
        if ((pxCenterBall.x - cropSize / 2) < 0) {
            pxCenterBall.x = cropSize / 2;
        }
        if ((pxCenterBall.x + cropSize / 2) >= nx) {
            pxCenterBall.x = nx - cropSize / 2;
        }
        if ((pxCenterBall.y - cropSize / 2) < 0) {
            pxCenterBall.y = cropSize / 2;
        }
        if ((pxCenterBall.y + cropSize / 2) >= ny) {
            pxCenterBall.y = ny - cropSize / 2;
        }
        //------------------------------

        cv::Rect roi((pxCenterBall.x - cropSize / 2), (pxCenterBall.y - cropSize / 2), cropSize, cropSize);

        img.copyTo(origImage);
        img = origImage(roi);

        ny = img.rows;
        nx = img.cols;
    }

    Mat Gray_mask = colorFilter.useColorFilter(img, nx, ny);
    Mat BinaryMask(ny, nx, CV_8U, Scalar(0));

    cv::threshold(Gray_mask, BinaryMask, 10, 255, cv::THRESH_BINARY_INV);

    vector < vector<Point> > gradcv;
    cv::findContours(BinaryMask, gradcv, noArray(), cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
    cout << "gradcv size: " << gradcv.size() << endl;

    if (gradcv.size() == 0) {
        cout << "Object (edge) do not detected. " << endl;
        if (ROI) {
            cout << "Try search on full image..... " << endl;
            ROI = false;
            return contr(origImage, ROI, pxCenterBall, cropSize, colorFilter);
        }
        cout << "Never - _ - " << endl;
        exit(0);
    }
    if (gradcv.size() > 1) {
        imwrite("Gray.png", Gray_mask);
        imwrite("Bin.png", BinaryMask);
        cout << "big grancv" << endl;
        exit(0);
        waitKey(0);
        waitKey(0);
    }

    vector<Point2f> gradcvConv(gradcv[0].begin(), gradcv[0].end());
    if (ROI) {
        //check edge crop
        for (int k = 0; k < gradcvConv.size(); ++k) {
            if (gradcvConv[k].x == 0 || gradcvConv[k].y == 0 || gradcvConv[k].x == 399 || gradcvConv[k].y == 399)
            {
                cout << "Crop image crop ball. Move window...Or find all range " << endl;
                ROI = false;
                return contr(origImage, ROI, pxCenterBall, cropSize, colorFilter);
                //ToDo move window
            }
        }

        for (int N = 0; N < gradcvConv.size(); ++N) { // return to global px cord
            gradcvConv[N].x += (pxCenterBall.x - cropSize / 2);
            gradcvConv[N].y += (pxCenterBall.y - cropSize / 2);
        }
    }
    ROI = true;
    return gradcvConv;
}

int main(int argc, char* argv[])
{

    

    //VideoCapture cap("..\\..\\..\\..\\BallDetectorData\\video\\video63Cycles.avi");
    VideoCapture cap("C:\\Users\\Vorku\\MyCodeProjects\\OctBall\\BallDetectorData\\video\\video63Cycles.avi");

    if (!cap.isOpened()) {
        cout << "Error opening video stream or file" << endl;
        return -1;
    }

    cv::utils::logging::setLogLevel(cv::utils::logging::LOG_LEVEL_SILENT);
    bool ROI = false;
    int cropSize = 400; //Only even number 

    vector<Point2i> pxCenterBall;

    float dt = 1.0 / 25.0;
    pxCenterBall.push_back(Point2i(200, 200));
    long int timer1 = getTickCount();

    std::string path = "..\\..\\..\\..\\BallDetectorData\\ColorCycles.bmp";

    ColorFilter colorFilter;

    colorFilter.init();
    colorFilter.get_ColorPoints(path);
    colorFilter.constructColorFilter();
    
    //RANSAC parameters
    float max_plane[4] = { 0.0, 0.0, 0.0, 0.0 };
    float k = 0.000008; // Distants between plane
    float abs_counter = 0.95;

    //Camera parameters
    Mat cameraMatrix = (Mat_<double>(3, 3) << 2666.666666666666, 0, 960, 0, 2666.666666666666, 540, 0, 0, 1);
    vector<float> distCoeffs = { 0,0,0,0 };
    Mat P = (Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);//New "ideal" cameramatrix
    Mat Rx = (Mat_<double>(3, 3) << -1, 0, 0, 0, -1, 0, 0, 0, 1); // Rotation matrix
    Mat T = (cv::Mat_<float>(3, 1) << 0, 0, 0); //Transpose matrix
    cv::Mat rvecR(3, 1, CV_64F);//rodrigues rotation matrix
    cv::Rodrigues(Rx, rvecR);

    Mat sourceImage;
    vector<Point3f> resultsCord;
    int cikle = 0;
    while (true) {
        cout << " Frame #" << cikle << endl;
        cap >> sourceImage;
        if (sourceImage.rows == 0 || sourceImage.cols == 0) {
            cout << "Picture not found or video end" << endl;
            break;
        }
        vector<Point2f> gradcvConv = contr(sourceImage, ROI, pxCenterBall.back(), cropSize, colorFilter);

        vector<Point2f> grad2;
        cv::undistortPoints(gradcvConv, grad2, cameraMatrix, distCoeffs, Rx, P = P); // точки без искажений и равные метрам
        vector <Point3f> a;
        converter2To3(grad2, a);
        vector <Point3f> v_norm;
        normalizeVectors(a, v_norm);
        vector<Point3f> iva_norm; // Points in finded plane
        int abs_iter = v_norm.size() / 3; // 
        findPlane(v_norm, max_plane, k, abs_counter, abs_iter); // Find plane
        inPoint(max_plane, v_norm, iva_norm, k);

        //find the cone apex angele 
        double theta;
        double normaNormali = pow(pow(max_plane[0], 2) + pow(max_plane[1], 2) + pow(max_plane[2], 2), 0.5);
        double sumarc = 0.0;
        for (int i = 0; i < iva_norm.size(); ++i) {
            sumarc += acos((max_plane[0] * iva_norm[i].x + max_plane[1] * iva_norm[i].y + max_plane[2] * iva_norm[i].z) / normaNormali);
        }
        theta = 2 * (sumarc / iva_norm.size());
        float RadiusBall = 1.0;
        double distans = RadiusBall / (sin(theta / 2));
        Point3f ballCoordinates;
        ballCoordinates.x = (distans / normaNormali) * max_plane[0];
        ballCoordinates.y = (distans / normaNormali) * max_plane[1];
        ballCoordinates.z = (distans / normaNormali) * max_plane[2];

        cout << "result = " << ballCoordinates << endl;
        resultsCord.push_back(ballCoordinates);



        // Find px center ball ------------------------------------------------------------------------------------------------------------
        vector<cv::Point2f> ProjectPointsFind;
        cv::projectPoints(resultsCord, rvecR, T, cameraMatrix, distCoeffs, ProjectPointsFind);
        ProjectPointsFind[cikle].x = round(ProjectPointsFind[cikle].x);
        ProjectPointsFind[cikle].y = round(ProjectPointsFind[cikle].y);
        pxCenterBall.push_back(Point2i(ProjectPointsFind[cikle].x, ProjectPointsFind[cikle].y));
        //----------------------------------------------------------------------------------------------------------------------------------
  

        char c = (char)cv::waitKey(1);
        if (c == 27)
            break;
        cout << endl;
        cv::waitKey(1);
        cikle += 1;
    }
    writer(resultsCord);

    long int timer2 = getTickCount();
    double finalTime = (timer2 - timer1) / getTickFrequency();
    cout << "Programm complete " << finalTime << " sec" << endl;
    waitKey(0);
    return 0;
}
