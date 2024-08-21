#include "common/show/show.h"

#include "common/utils.h"

void Show::Reset(cv::Mat& image) {
  image_ = image;
  canvas_.Reset();
}

const cv::Mat& Show::Display() {
  cv::hconcat(image_, canvas_.Image(), display_);
  return display_;
}

void Show::Undistort() {
  cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 1143.398167, 0.0,
                          956.0743379, 0.0, 1169.03, 532.9676489, 0, 0, 1);
  cv::Mat distCoeffs = (cv::Mat_<double>(1, 5) << -0.356918618, 0.167589043,
                        -0.000186537, -0.00026963, -0.045005666);
  // cv::Size imageSize = image_.size();
  // cv::Mat newCameraMatrix;
  // cv::Rect validROI;
  // newCameraMatrix = cv::getOptimalNewCameraMatrix(
  //     cameraMatrix, distCoeffs, imageSize, 1.0, imageSize, &validROI);
  cv::Mat undistortedImage;
  // cv::undistort(image_, undistortedImage, cameraMatrix, distCoeffs,
  //               newCameraMatrix);
  cv::undistort(image_, undistortedImage, cameraMatrix, distCoeffs);
  image_ = undistortedImage;
}

void Show::AddInfo(ImageInfo& info) {
  char buf[64];
  snprintf(buf, sizeof(buf), "%s frame: %d", info.scene_name.c_str(),
           info.frame_id);
  cv::putText(canvas_.Image(), buf, cv::Point(30, 30), 0, 0.5,
              cv::Scalar(255, 255, 255));
}

void Show::DrawObjDetect(const DETECT_OBJS& objs, bool frontail) {
  for (size_t i = 0; i < objs.objs_num; i++) {
    cv::Rect2f det = cv::Rect_<float>(objs.detect_objs[i].box.left_top_u,
                                      objs.detect_objs[i].box.left_top_v,
                                      objs.detect_objs[i].box.right_bottom_u -
                                          objs.detect_objs[i].box.left_top_u,
                                      objs.detect_objs[i].box.right_bottom_v -
                                          objs.detect_objs[i].box.left_top_v);
    cv::rectangle(image_, det, cv::Scalar(0, 0, 255), 2);
    if (frontail && objs.detect_objs[i].sub_box_confidence > 0.0) {
      cv::Rect2f sub_det =
          cv::Rect_<float>(objs.detect_objs[i].sub_box.left_top_u,
                           objs.detect_objs[i].sub_box.left_top_v,
                           objs.detect_objs[i].sub_box.right_bottom_u -
                               objs.detect_objs[i].sub_box.left_top_u,
                           objs.detect_objs[i].sub_box.right_bottom_v -
                               objs.detect_objs[i].sub_box.left_top_v);
      cv::rectangle(image_, sub_det, cv::Scalar(0, 128, 128), 2);
    }
  }
}

void Show::DrawLaneDetect(const DETECT_LANES& lanes) {
  for (size_t i = 0; i < lanes.lanes_num; i++) {
    std::vector<cv::Point> points;
    auto ys =
        linspace(lanes.lanes_pxl[i].start_y0, lanes.lanes_pxl[i].end_y0, 100);
    for (auto& y : ys) {
      double x = lanes.lanes_pxl[i].c00 + lanes.lanes_pxl[i].c01 * pow(y, 1) +
                 lanes.lanes_pxl[i].c02 * pow(y, 2) +
                 lanes.lanes_pxl[i].c03 * pow(y, 3);
      cv::Point point(static_cast<int>(x), static_cast<int>(y));
      points.push_back(point);
    }
    cv::polylines(image_, points, false, cv::Scalar(0, 255, 0), 1);
    points.clear();
    if (lanes.lanes_pxl[i].param_num == 2) {
      auto ys_2 =
          linspace(lanes.lanes_pxl[i].start_y1, lanes.lanes_pxl[i].end_y1, 100);
      for (auto& y : ys_2) {
        double x = lanes.lanes_pxl[i].c10 + lanes.lanes_pxl[i].c11 * pow(y, 1) +
                   lanes.lanes_pxl[i].c12 * pow(y, 2) +
                   lanes.lanes_pxl[i].c13 * pow(y, 3);
        cv::Point point(static_cast<int>(x), static_cast<int>(y));
        points.push_back(point);
      }
    }
    cv::polylines(image_, points, false, cv::Scalar(0, 255, 0), 1);
  }
}

void Show::DrawLaneTrack(const TRACK_LANES& lanes) {
  char buf[64];
  snprintf(buf, sizeof(buf), "pitch: %f quality: %d", lanes.pitch,
           static_cast<int>(lanes.pitch_pl));
  cv::putText(canvas_.Image(), buf, cv::Point(30, 60), 0, 0.5,
              cv::Scalar(255, 255, 255));

  for (size_t i = 0; i < lanes.lanes_num; i++) {
    std::vector<cv::Point> points_pxl;
    auto ys =
        linspace(lanes.lanes_pxl[i].start_y0, lanes.lanes_pxl[i].end_y0, 100);
    for (auto& y : ys) {
      double x = lanes.lanes_pxl[i].c00 + lanes.lanes_pxl[i].c01 * pow(y, 1) +
                 lanes.lanes_pxl[i].c02 * pow(y, 2) +
                 lanes.lanes_pxl[i].c03 * pow(y, 3);
      cv::Point point(static_cast<int>(x), static_cast<int>(y));
      points_pxl.push_back(point);
    }
    cv::polylines(image_, points_pxl, false, cv::Scalar(0, 0, 255), 1);
    char id_buf[8];
    snprintf(id_buf, sizeof(id_buf), "%d", lanes.lanes_pxl[i].lane_id);
    cv::putText(image_, id_buf, points_pxl[50], cv::FONT_HERSHEY_DUPLEX, 0.7,
                cv::Scalar(0, 0, 255));

    std::vector<cv::Point2f> points_bev;
    auto ys_bev =
        linspace(lanes.lanes_wrd[i].start_y0, lanes.lanes_wrd[i].end_y0, 100);
    Eigen::VectorXd xs(100);
    for (size_t j = 0; j < ys_bev.rows(); j++) {
      double y = ys_bev[j];
      double x = lanes.lanes_wrd[i].c00 + lanes.lanes_wrd[i].c01 * pow(y, 1) +
                 lanes.lanes_wrd[i].c02 * pow(y, 2) +
                 lanes.lanes_wrd[i].c03 * pow(y, 3);
      points_bev.push_back(
          cv::Point2f(-static_cast<float>(x), static_cast<float>(y)));
    }
    canvas_.Polylines(points_bev, cv::Scalar(0, 0, 255));
  }
}

void Show::DrawTrackObjs(const TRACK_OBJS& objs, bool frontail) {
  for (size_t i = 0; i < objs.objs_num; i++) {
    cv::Rect2f track = cv::Rect_<float>(objs.track_objs[i].box.left_top_u,
                                        objs.track_objs[i].box.left_top_v,
                                        objs.track_objs[i].box.right_bottom_u -
                                            objs.track_objs[i].box.left_top_u,
                                        objs.track_objs[i].box.right_bottom_v -
                                            objs.track_objs[i].box.left_top_v);
    cv::rectangle(image_, track, cv::Scalar(255, 255, 0), 2);
    if (frontail && objs.track_objs[i].obj_info.sub_box_confidence > 0.0) {
      auto& det_info = objs.track_objs[i].obj_info;
      cv::Rect2f sub_det = cv::Rect_<float>(
          det_info.sub_box.left_top_u, det_info.sub_box.left_top_v,
          det_info.sub_box.right_bottom_u - det_info.sub_box.left_top_u,
          det_info.sub_box.right_bottom_v - det_info.sub_box.left_top_v);
      cv::rectangle(image_, sub_det, cv::Scalar(0, 128, 128), 2);
    }
    if (objs.track_objs[i].activate) {
      cv::Point2d p(-objs.track_objs[i].lateral_distance,
                    objs.track_objs[i].long_distance);
      canvas_.Circle(p, 3, cv::Scalar(0, 0, 255), -1);
      cv::putText(
          image_,
          std::to_string(objs.track_objs[i].track_id) + " " +
              std::to_string(objs.track_objs[i].long_distance).substr(0, 6) +
              " " +
              std::to_string(objs.track_objs[i].lateral_distance).substr(0, 6),
          cv::Point2f(objs.track_objs[i].box.left_top_u,
                      (objs.track_objs[i].box.left_top_v - 5.0)),
          0, 0.5, cv::Scalar(255, 255, 0));
    } else {
      cv::putText(
          image_,
          std::to_string(objs.track_objs[i].track_id) + " " +
              std::to_string(objs.track_objs[i].confidence).substr(0, 4),
          cv::Point2f(objs.track_objs[i].box.left_top_u,
                      (objs.track_objs[i].box.left_top_v - 5.0)),
          0, 0.5, cv::Scalar(255, 255, 0));
    }
  }
}

void Show::DrawMobileyeData(const MOBILEYE_OBSTACLES& objs,
                            const MOBILEYE_LANE_SHAPES& lanes) {
  for (auto& obj : objs) {
    cv::Rect2f box(obj.f_lateral_distance - 0.5, obj.f_long_distance - 0.5, 1,
                   1);
    canvas_.Rectangle(box, cv::Scalar(255, 0, 0), -1);
  }
  for (auto& lane : lanes) {
    std::vector<cv::Point2f> points;
    auto ys = linspace(lane.f_start, lane.f_end, 100);
    for (auto& y : ys) {
      double x = lane.f_c0 + lane.f_c1 * pow(y, 1) + lane.f_c2 * pow(y, 2) +
                 lane.f_c3 * pow(y, 3);
      cv::Point2f point(static_cast<float>(x), static_cast<float>(y));
      points.push_back(point);
    }
    canvas_.Polylines(points, cv::Scalar(255, 0, 0));
  }
}