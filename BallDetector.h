#pragma once

#ifndef MY_CLASS_H // include guard
#define MY_CLASS_H

#include <iostream>
#include <opencv2/opencv.hpp>


void constructColorFilter(Mat& pts, Mat& v, Scalar& p0, double& t1, double& t2, double& R); // Calculates coefficients (cylinder) from the passed points of the same color (pts)
Mat useColorFilter(Mat& img, Mat& v, Scalar& p0, double& t1, double& t2, double& R, int& nx, int& ny);  // Applies previously obtained coefficients and builds an alpha mask on the image.

void converter2To3(vector<Point2f>& src, vector <Point3f>& dst);
void normalizeVectors(vector<Point3f>& src, vector <Point3f>& dst);
void findPlane(vector<Point3f>& ptr, float* max_plane, float& k, float& abs_counter, int& abs_iter); //Find best plane used RANSAC
void makePlane(Point3f& ptr1, Point3f& ptr2, Point3f& ptr3, float* plane); //Pose the plane on 3 points
float upPlane(float* plane, float& k, Point3f& ptr);
float downPlane(float* plane, float& k, Point3f& ptr);
float counterPlane(float* plane, float& k, vector<Point3f>& ptr);
void inPoint(float* plane, vector<Point3f>& ptr, vector<Point3f>& inptr, float& k); //Estimating the number of points between planes
void inPointPaint(vector<Point3f>& MyCord, vector<Point2f>& PixCord, Mat& img, float* plane, float& k); // Input va and plant 

float SumErrPlane(vector<Point3f> iva_norm, float* max_plane);

vector<Point2f> contr(Mat img, bool& ROI, Point2i pxCenterBall, Mat& const v, double& const t1, double& const t2, double& const R, Scalar& const p0, int& const cropSize); //Picture -> Pixel border coordinates

void writer(vector<Point2f>&text); //Save data to file
void writer(vector<Point3f>&text);


#endif /* MY_CLASS_H */
