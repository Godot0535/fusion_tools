#include "convas.h"

Canvas::Canvas(cv::Size size, cv::Rect rect, int len)
    : _size(size), _geo_rect(rect) {
  int w = size.width;
  int h = size.height;
  _canvas = cv::Mat(h, w, CV_8UC3, cv::Scalar(0, 0, 0));

  double left = rect.x;
  double top = rect.y;
  double right = rect.x + rect.width;
  double bottom = rect.y - rect.height;

  _rx = w / rect.width;
  _ry = h / rect.height;
  _x0 = left;
  _y0 = bottom;
  _each_len = len;
}

void Canvas::Reset() {
  _canvas.setTo(cv::Scalar(0, 0, 0));

  int w = _size.width;
  int h = _size.height;

  cv::line(_canvas, cv::Point(w / 2, 0), cv::Point(w / 2, h),
           cv::Scalar(100, 100, 100), 1);

  cv::Point2i pt = Geo2canvas(cv::Point2f(0, 0));
  cv::Point2i pt_ = Geo2canvas(cv::Point2f(_each_len, _each_len));
  int len_x = pt_.x - pt.x + 0.5, len_y = pt.y - pt_.y + 0.5;
  int y0 = static_cast<int>(pt.y + 0.5);
  cv::line(_canvas, cv::Point(0, y0), cv::Point(w, y0),
           cv::Scalar(100, 100, 100), 1);

  // int len_x = _each_len * _rx + 0.5, len_y = _each_len * _ry + 0.5;

  for (int i = 0; i < h; ++i) {
    if (i % len_y == 0) {
      cv::line(_canvas, cv::Point(w / 2, h - i), cv::Point(w / 2 + 2, h - i),
               cv::Scalar(100, 100, 100), 1);
      if ((i / len_y - 1) % 10 == 0) {
        cv::line(_canvas, cv::Point(w / 2, h - i), cv::Point(w / 2 + 2, h - i),
                 cv::Scalar(255, 255, 255), 1);
      }
    }
  }
  for (int i = 0; i < w; ++i) {
    if (i % len_x == 0) {
      cv::line(_canvas, cv::Point(i, y0), cv::Point(i, y0 - 2),
               cv::Scalar(100, 100, 100), 1);
    }
  }
}

void Canvas::Circle(cv::Point2f center, int radius, cv::Scalar color,
                    int thickness, int id) {
  cv::Point2i pt = Geo2canvas(center);
  cv::circle(_canvas, pt, radius, color, thickness);

  if (id != -1) {
    cv::putText(_canvas, std::to_string(id), cv::Point(pt.x + 3, pt.y - 3), 0,
                0.5, color);
  }
}

void Canvas::Rectangle(cv::Rect2f box, cv::Scalar color, int thickness) {
  cv::Point2f pt(box.x, box.y);
  cv::Point2i pt_ = Geo2canvas(pt);
  int w = box.width * _rx + 0.5, h = box.height * _ry + 0.5;
  cv::Rect2i box_(pt_.x - w / 2, pt_.y - h / 2, w, h);
  cv::rectangle(_canvas, box_, color, thickness);
}

void Canvas::Polylines(std::vector<cv::Point2f> lanes, cv::Scalar color) {
  std::vector<cv::Point> pts;
  for (const auto &pt : lanes) {
    pts.push_back(Geo2canvas(pt));
  }
  cv::polylines(_canvas, pts, false, color, 1);
}

void Canvas::Line(cv::Point2d point_1, cv::Point2d point_2, cv::Scalar color) {
  cv::line(_canvas, Geo2canvas(point_1), Geo2canvas(point_2), color, 2);
}

cv::Point2i Canvas::Geo2canvas(cv::Point2f pt) {
  int x = static_cast<int>((pt.x - _x0) * _rx + 0.5);
  int y = _size.height - static_cast<int>((pt.y - _y0) * _ry + 0.5);
  return cv::Point2i(x, y);
}
