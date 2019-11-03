//
// Created by aidan on 10/6/19.
//


#include <jni.h>
#include <opencv2/opencv.hpp>

typedef struct skystone
{
    int min_x;
    int max_x;
    int center_y;
} skystone;

typedef struct pair
{
    int a;
    int b;
} pair;

int sum(uchar *p, int length)
{
    int total = 0;
    for (int i = 0; i < length; i++)
    {
        total += *p;
        p++;
    }
    return total;
}

cv::Mat prev_hicontrast;

cv::Mat generate_high_contrast(cv::Mat input)
{
    cv::Mat output;
    cv::Mat bgr[3];

    cv::split(input, bgr);
    cv::add(bgr[2], bgr[1], output, cv::noArray(), CV_16S);
    cv::Mat blue;
    bgr[0].convertTo(blue, CV_16S);

    cv::subtract(output, blue * 2, output);

    output = (output / 4) + 128;
    output.convertTo(output, CV_8U);

    prev_hicontrast = output;

    return output;
}

pair find_crop_area(cv::Mat thresh, int step, double on_threshold, double off_threshold)
{
    int top = 0;
    int bot = 0;

    for (int y = thresh.rows - 1; y >= 0; y -= step)
    {
        if (bot == 0 && sum(thresh.ptr(y), thresh.cols) > thresh.cols * on_threshold)
        {
            bot = y;
        }
        else if (bot > 0 && sum(thresh.ptr(y), thresh.cols) < thresh.cols * off_threshold)
        {
            top = y;
            break;
        }
    }

    return {top, bot};
}

pair find_skystone(cv::Mat rotated, int step, float threshold)
{
    pair skystone_range = {-1, -1};
    pair current_range = {-1, -1};

    for (int y = 0; y < rotated.rows; y += step)
    {
        uchar *p = rotated.ptr(y);
        int x = 0;
        for (; x < rotated.cols; x++)
        {
            if (*p != 0) break;
            p++;
        }

        if (sum(p, rotated.cols - x)/255.0 < (rotated.cols - x) * threshold)
        {
            if (current_range.a < 0)
            {
                current_range = {y, y};
            }
            else
            {
                current_range.b = y;
            }
        }
        else
        {
            if (current_range.a >= 0)
            {
                if (current_range.b - current_range.a > skystone_range.b - skystone_range.a)
                {
                    skystone_range = {current_range.a, current_range.b};
                }

                current_range.a = -1;
            }
        }
    }

    if (current_range.a >= 0 && current_range.b - current_range.a > skystone_range.b - skystone_range.a)
    {
        skystone_range = current_range;
    }

    return skystone_range;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Java Interface
////////////////////////////////////////////////////////////////////////////////////////////////////

extern "C"
{

pair crop_area;
pair skystone_range;
bool found = false;

JNIEXPORT jint JNICALL Java_org_firstinspires_ftc_teamcode_autonomous_vision_SkystoneDetector_submit
    (JNIEnv *env, jobject instance, jlong mat_addr)
{
  cv::Mat img = *((cv::Mat *)mat_addr);

  if (img.rows == 0 || img.cols == 0)
  {
      found = false;
      return -1;
  }

  cv::Mat high_contrast = generate_high_contrast(img);

  cv::Mat thresh;
  cv::threshold(high_contrast, thresh, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

  pair area = find_crop_area(thresh, 5, 1.0/32, 1.0/32);

  // If we don't detect anything OR we detect the entire screen, we're just getting garbage
  if ((area.a == 0 && area.b == 0) || (area.a < 5 && area.b > img.rows - 5))
  {
      found = false;
      return 1;
  }

  cv::Mat cropped(thresh, cv::Rect(0, area.a, thresh.cols, area.b - area.a));

  cv::Mat rotated;
  cv::transpose(cropped, rotated);

  pair range = find_skystone(rotated, 10, 0.6);

  if (range.a < 0)
  {
      found = false;
      return 2;
  }

  crop_area = area;
  skystone_range = range;
  found = true;
  return 0;
}

JNIEXPORT jboolean JNICALL Java_org_firstinspires_ftc_teamcode_autonomous_vision_SkystoneDetector_detected
    (JNIEnv *env, jobject instance)
{
   return (jboolean)found;
}

JNIEXPORT jint JNICALL Java_org_firstinspires_ftc_teamcode_autonomous_vision_SkystoneDetector_get_1min_1x
    (JNIEnv *env, jobject instnace)
{
    return skystone_range.a;
}

JNIEXPORT jint JNICALL Java_org_firstinspires_ftc_teamcode_autonomous_vision_SkystoneDetector_get_1max_1x
    (JNIEnv *env, jobject instance)
{
    return skystone_range.b;
}

JNIEXPORT jint JNICALL Java_org_firstinspires_ftc_teamcode_autonomous_vision_SkystoneDetector_get_1min_1y
    (JNIEnv *env, jobject instance)
{
    return crop_area.a;
}

JNIEXPORT jint JNICALL Java_org_firstinspires_ftc_teamcode_autonomous_vision_SkystoneDetector_get_1max_1y
    (JNIEnv *env, jobject instance)
{
    return crop_area.b;
}

JNIEXPORT void JNICALL Java_org_firstinspires_ftc_teamcode_autonomous_vision_SkystoneDetector_draw
    (JNIEnv *env, jobject instance, jlong mat_addr, jboolean )
{
    cv::Mat mat = *((cv::Mat *)mat_addr);
    cv::line(mat, cv::Point(0, crop_area.a), cv::Point(mat.cols, crop_area.a), cv::Scalar(0, 0, 255), 2);
    cv::line(mat, cv::Point(0, crop_area.b), cv::Point(mat.cols, crop_area.b), cv::Scalar(255, 0, 0), 2);
    cv::line(mat, cv::Point(skystone_range.a, 0), cv::Point(skystone_range.a, mat.rows), cv::Scalar(0, 128, 128), 2);
    cv::line(mat, cv::Point(skystone_range.b, 0), cv::Point(skystone_range.b, mat.rows), cv::Scalar(0, 128, 128), 2);
}

} // extern "C"
