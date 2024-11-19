#pragma once
#include <opencv2/opencv.hpp>

class Canvas {
 public:
  Canvas(cv::Size size, cv::Rect rect, int len = 5);

  void Reset();

  cv::Mat Image() { return _canvas; }

  void Circle(cv::Point2f center, int radius,
              cv::Scalar color = cv::Scalar(0, 255, 0), int thickness = 1,
              int id = -1);
  void Rectangle(cv::Rect2f box, cv::Scalar color, int thickness = 1);
  void Polylines(std::vector<cv::Point2f> lanes,
                 cv::Scalar color = cv::Scalar(0, 255, 0));
  void Line(cv::Point2d point_1, cv::Point2d point_2,
            cv::Scalar color = cv::Scalar(0, 255, 0));

 private:
  cv::Mat _canvas;
  cv::Size _size;
  cv::Rect _geo_rect;
  float _each_len;
  double _rx, _ry, _x0, _y0;

  cv::Point2i Geo2canvas(cv::Point2f pt);
};
