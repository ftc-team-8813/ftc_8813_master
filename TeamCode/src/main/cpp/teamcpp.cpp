//
// Created by aidan on 9/8/18.
//

#include <jni.h>
#include <opencv2/opencv.hpp>
#include <vector>
extern "C"
{

using namespace cv;
using namespace std;

// Applies Sobel edge-detection to the Mat pointed to by the jlong
JNIEXPORT void JNICALL Java_org_firstinspires_ftc_teamcode_autonomous_test_opencv_OpenCVNative_test
        (JNIEnv *env, jobject instance, jlong mat_addr)
{
    Mat *mat = (Mat *) mat_addr;
    GaussianBlur(*mat, *mat, Size(3, 3), 0, 0, BORDER_DEFAULT);
    cvtColor(*mat, *mat, COLOR_BGR2GRAY);

    Mat grad_x, grad_y;
    Mat abs_grad_x, abs_grad_y;

    Sobel(*mat, grad_x, CV_16S, 1, 0, 1, 1, 0, BORDER_DEFAULT);
    Sobel(*mat, grad_y, CV_16S, 0, 1, 1, 1, 0, BORDER_DEFAULT);

    convertScaleAbs(grad_x, abs_grad_x);
    convertScaleAbs(grad_y, abs_grad_y);

    addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, *mat);
}

vector<Mat> outContours;
Mat outBestContour;
bool seen = false;
Point lastCenter;


JNIEXPORT void JNICALL Java_org_firstinspires_ftc_teamcode_common_util_sensors_vision_NativeGoldDetector_process
        (JNIEnv *env, jobject instance, jlong mat_addr)
{
    Mat *image = (Mat *) mat_addr;
    Mat hsv;
    cvtColor(*image, hsv, COLOR_BGR2GRAY);

    Mat mask;
    inRange(hsv, Scalar(10, 120, 40), Scalar(33, 255, 255), mask);

    outContours.clear();

    vector<Mat> contours;
    findContours(mask, contours, RETR_TREE, CHAIN_APPROX_SIMPLE);

    Mat bestContour;
    bool foundBest = false;
    double bestArea = 0;
    for (Mat contour : contours)
    {
        double perim = arcLength(contour, true);
        Mat poly;
        approxPolyDP(contour, poly, 0.015 * perim, true);

        outContours.push_back(poly);
        if (perim >= 100 && poly.total() >= 4 && poly.total() <= 6 && isContourConvex(poly))
        {
            double area = contourArea(poly);
            if (bestArea < area)
            {
                bestContour = poly;
                bestArea = area;
                foundBest = true;
            }
        }
    }

    if (foundBest)
    {
        outBestContour = bestContour;
        seen = true;
        Moments m = moments(bestContour);
        lastCenter = Point((int)(m.m10/ m.m00), (int)(m.m01 / m.m00));
    } else
    {
        seen = false;
    }
}

JNIEXPORT void JNICALL Java_org_firstinspires_ftc_teamcode_common_util_sensors_vision_NativeGoldDetector_draw
        (JNIEnv *env, jobject instance, jlong mat_addr)
{
    Mat *bgr = (Mat *) mat_addr;
    for (const Mat &contour : outContours)
    {
        double perim = arcLength(contour, true);
        int edgeCount = contour.total();
        bool convex = isContourConvex(contour);

        Scalar color(0, 127, 0);
        int thickness = 2;

        if (perim < 100)
        {
            thickness = 1;
        }
        if (edgeCount < 4 || edgeCount > 6)
        {
            color = Scalar(0, 0, 127);
        }
        if (convex)
        {
            color = Scalar(color[0] * 2, color[1] * 2, color[2] * 2);
        }
        vector<Mat> contourList;
        contourList.push_back(contour);
        polylines(*bgr, contourList, true, color, thickness);
    }
    if (seen)
    {
        vector<Mat> contourList;
        contourList.push_back(outBestContour);
        polylines(*bgr, contourList, true, Scalar(0, 255, 255), 2);
        Rect bound = boundingRect(outBestContour);
        rectangle(*bgr, bound, Scalar(255, 255, 0), 2);
        circle(*bgr, lastCenter, 5, Scalar(255, 0, 0));
        line(*bgr, Point(lastCenter.x - 20, lastCenter.y), Point(lastCenter.x + 20, lastCenter.y),
                Scalar(0, 0, 255));
        line(*bgr, Point(lastCenter.x, lastCenter.y - 20), Point(lastCenter.x, lastCenter.y + 20),
                Scalar(255, 0, 0));

    }
}

JNIEXPORT jint JNICALL Java_org_firstinspires_ftc_teamcode_common_util_sensors_vision_NativeGoldDetector_getX
        (JNIEnv *env, jobject instance)
{
    return lastCenter.x;
}

JNIEXPORT jint JNICALL Java_org_firstinspires_ftc_teamcode_common_util_sensors_vision_NativeGoldDetector_getY
        (JNIEnv *env, jobject instance)
{
    return lastCenter.y;
}

JNIEXPORT jboolean JNICALL Java_org_firstinspires_ftc_teamcode_common_util_sensors_vision_NativeGoldDetector_seen
        (JNIEnv *env, jobject instance)
{
    return (jboolean)seen;
}

} // extern "C"