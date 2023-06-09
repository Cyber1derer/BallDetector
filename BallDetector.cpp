// BallDetectorV2.cpp: определяет точку входа для приложения.
#include <fstream>
#include <iostream>
#include <cmath>
#include <typeinfo>
#include <vector>
#include <algorithm>

#include <nlohmann/json.hpp>

#include <opencv2/calib3d.hpp> // Undistort and Rodrigues 
#include <opencv2/imgproc.hpp> //FindCont
#include <opencv2/imgcodecs.hpp> //Imread
#include <opencv2/highgui.hpp> // Waitkey
#include <opencv2/core/utils/logger.hpp>

#include <chrono>

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
    //cout << "max counter" << max_counter << endl;
    //cout << "iter" << iter << endl;
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
        //cout << SumErr << endl;
    }
    return SumErr;
} 

vector<Point2f> contr(Mat img, bool& ROI, Point2i pxCenterBall, int& const cropSize, ColorFilter& colorFilter) {
    int ny = img.rows;
    int nx = img.cols;
    int offsetX = 0;
    int offsetY = 0;
    Mat origImage;
    ROI = false;
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
    //cv::imwrite("Gray_mask.bmp", Gray_mask);

    //vector<float> colorBright = colorFilter.colorHist(Gray_mask);


    /*
    vector<int> colorBrightX;
    for (int i = 0; i < Gray_mask.cols; ++i) {
        int sum = 0;

        for (int j = 0; j < Gray_mask.rows; j++) {
            sum += 255 - Gray_mask.at<uchar>(j, i);
        }
        colorBrightX.push_back(sum);
    }

    vector<int> colorBrightY;
    for (int i = 0; i < Gray_mask.rows; ++i) {
        int sum = 0;

        for (int j = 0; j < Gray_mask.cols; j++) {
            sum += 255 - Gray_mask.at<uchar>(i, j);
        }
        colorBrightY.push_back(sum);
    }




    auto max_element_iter = std::max_element(colorBrightX.begin(), colorBrightX.end());
    size_t max_element_index = std::distance(colorBrightX.begin(), max_element_iter);

    //cout << "Hist max x " << *max_element_iter << endl;
    //cout << "Hist max index (x) " << max_element_index << endl;

    auto max_element_iterY = std::max_element(colorBrightY.begin(), colorBrightY.end());
    size_t max_element_indexY = std::distance(colorBrightY.begin(), max_element_iterY);
    */
    //cout << "Hist max y " << *max_element_iterY << endl;
    //cout << "Hist max index (y) " << max_element_indexY << endl;


    /*
    for (int i = 0; i < colorBright.size(); i++) {
        if (colorBright[i] < *max_element_iter*0.2) {
            colorBright[i] = 0;
        }
    }
    */

    
    //------------------------------Sobel
    cv::Mat src_gray;
    cv::Mat grad_x, grad_y, abs_grad_x, abs_grad_y;


    int ddepth = CV_8U;

    cv::cvtColor(img, src_gray, cv::COLOR_BGR2GRAY);
    //cv::imshow("Image gray", src_gray);
    cv::Sobel(src_gray, grad_x, ddepth, 1, 0);
    //cv::imshow("X-derivative", grad_x);

    cv::Sobel(src_gray, grad_y, ddepth, 0, 1);
    //cv::imshow("Y-derivative", grad_y);


    // Old metod find center RotatedRect ellipse = fitEllipse(gradcv[0]);
    // находим координаты центра эллипса
    Mat BinaryMask(ny, nx, CV_8U, Scalar(0));
    cv::threshold(Gray_mask, BinaryMask, 40, 255, cv::THRESH_BINARY_INV);
    //vector < vector<Point> > gradcv;
    //cv::findContours(BinaryMask, gradcv, noArray(), cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
    //cout << "gradcv size: " << gradcv.size() << endl;
    //vector<Point2f> gradcvConv(gradcv[0].begin(), gradcv[0].end());

    Moments moments = cv::moments(BinaryMask);
     // Access centroid coordinates
    double cx = moments.m10 / moments.m00;
    double cy = moments.m01 / moments.m00;

    // Print coordinates to console
    //cout << "Centroid coordinates: (" << cx << ", " << cy << ")" << endl;




    Point2f centerEllipse(cx, cy);
    // выводим координаты центра в консоль
    //std::cout << "Ellipse center: x = " << centerEllipse.x << ", y = " << centerEllipse.y << std::endl;
    // отображаем эллипс и его центр на изображении
    //ellipse(src, center, Scalar(0, 255, 0), 2);
    vector <Point2f> gradSobel;
    int quail = 30;
    double pixel;
    double v_norm;
    vector<double> pixelVec;

    // Calculate value for each pixel
    for (int i = 0; i < src_gray.rows; i++) {
        for (int j = 0; j < src_gray.cols; j++) { 
            // Do your calculation on the pixel at i,j
            //pixel = <(j - centerEllipse.x), (i - centerEllipse.y)> * <grad_x(i,j), grad_y>

             // Define two vectors
            v_norm = pow((pow((j - centerEllipse.x), 2) + pow(i - centerEllipse.y, 2)), 0.5);
            Vec2d fromCenter( (j - centerEllipse.x)/v_norm, (i - centerEllipse.y)/ v_norm);
            Vec2d SobelVec(grad_x.at<uchar>(i, j), grad_y.at<uchar>(i, j));
            // Compute the cross product
            pixel = abs(fromCenter.dot(SobelVec));

            // Update pixel in the image
            //imshow("src gray", src_gray);
            pixelVec.push_back(pixel);
            //waitKey();

            if (pixel > abs(quail)) {
                gradSobel.push_back(Point2i(j+1 , i+1 ));

            }

        }

    }

    //auto min = *std::min_element(pixelVec.begin(), pixelVec.end());
    //auto max = *std::max_element(pixelVec.begin(), pixelVec.end());

    //std::cout << "Minimum element: " << min << std::endl;
    //std::cout << "Maximum element: " << max << std::endl;

    // 
    //imshow("Sob x", grad_x);
    //imshow("Sob y", grad_y);

    //waitKey(1);


    


    //cout << "GradSoble" << gradSobel << endl;
    
    /*for (const auto& point : gradSobel) {
        img.at<Vec3b>(point)[0] = 0;
        img.at<Vec3b>(point)[1] = 255;
        img.at<Vec3b>(point)[2] = 0;
    } */
    //imwrite("img_with_SobelGrad.png", img);
    //imshow("ImgSobel", img);
    //waitKey(0);
    
    //---------------------------------------------Sobel end



    //vector<Point2f> gradcvConv(gradcv[0].begin(), gradcv[0].end());
    // 
    // 
    vector<Point2f> gradcvConv = gradSobel;
    
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
    namedWindow("Source window", WINDOW_NORMAL);
    resizeWindow("Source window", 1280, 720);

    //namedWindow("Crop window", WINDOW_NORMAL);
    //resizeWindow("Crop window", 1280, 720);

    //VideoCapture cap("..\\..\\..\\..\\BallDetectorData\\video\\video63Cycles.avi");
    VideoCapture cap("..\\..\\..\\..\\BallDetectorData\\video350.avi");

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
    long double timer1 = getTickCount();

    std::string path = "..\\..\\..\\..\\BallDetectorData\\ColorCycles.bmp";

    ColorFilter colorFilter;

    colorFilter.init();
    colorFilter.get_ColorPoints(path);
    colorFilter.constructColorFilter();
    
    //RANSAC parameters
    float max_plane[4] = { 0.0, 0.0, 0.0, 0.0 };
    float k =6*10e-7; // Distants between plane
    float abs_counter = 0.95;

    //Camera parameters
    //Mat cameraMatrix = (Mat_<double>(3, 3) << 2666.666666666666, 0, 960, 0, 2666.666666666666, 540, 0, 0, 1);
    Mat cameraMatrix = (Mat_<double>(3, 3) << 1777.7777777777778, 0, 640.0, 0, 1777.7777777777778, 512.0, 0, 0, 1); 
    vector<float> distCoeffs = { 0,0,0,0 };
    Mat P = (Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);//New "ideal" cameramatrix
    Mat Rx = (Mat_<double>(3, 3) << -1, 0, 0, 0, -1, 0, 0, 0, 1); // Rotation matrix
    Mat T = (cv::Mat_<float>(3, 1) << 0, 0, 0); //Transpose matrix
    cv::Mat rvecR(3, 1, CV_64F);//rodrigues rotation matrix
    cv::Rodrigues(Rx, rvecR);

    Mat sourceImage;
    vector<Point3f> resultsCord;
    int cycle = 0;
   // while (cikle < 16) {
    auto start = std::chrono::high_resolution_clock::now();
    while (cycle!=375) {
        //cout << " Frame #" << cikle << endl;
        cap >> sourceImage;
        //sourceImage = imread("..\\..\\..\\..\\BallDetectorData\\1000imgConstZ\\" + to_string(cycle) + ".bmp" , 1);
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

        if (false) {
            inPointPaint(v_norm, gradcvConv, sourceImage, max_plane, k);
            imwrite("RANSACWork.png", sourceImage);
        }

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
        ProjectPointsFind[cycle].x = round(ProjectPointsFind[cycle].x);
        ProjectPointsFind[cycle].y = round(ProjectPointsFind[cycle].y);
        pxCenterBall.push_back(Point2i(ProjectPointsFind[cycle].x, ProjectPointsFind[cycle].y));
        //----------------------------------------------------------------------------------------------------------------------------------
        //std::cout << "My px center: x = " << pxCenterBall.back().x << ", y = " << pxCenterBall.back().y << std::endl;

        // Draw center-------------------------------------
        sourceImage.at<Vec3b>(pxCenterBall.back())[2] = 0;  //some color
        sourceImage.at<Vec3b>(pxCenterBall.back())[0] = 255;
        sourceImage.at<Vec3b>(pxCenterBall.back())[1] = 0;
        circle(sourceImage, pxCenterBall.back(), 4, (200, 80, 200), -1);
        //Draw trajectory-----------------------------
        /* for (int k = 1; k < pxCenterBall.size() - 1; ++k) {
            cv::line(sourceImage, pxCenterBall[k], pxCenterBall[k + 1], (0, 0, 0), 3);
        } */

        //imshow("Source window", sourceImage);
        /*char c = (char)cv::waitKey(1);
        if (c == 27)
            break;
        cout << endl;
        //cv::waitKey(1); */
        cv::waitKey(1);
        cycle += 1;
    }
    writer(resultsCord);
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> elapsed = end - start;
    std::cout << "Elapsed time: " << elapsed.count() << " ms" << std::endl;
    waitKey(0);
    return 0;
}
