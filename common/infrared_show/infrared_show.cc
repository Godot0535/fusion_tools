#include "common/infrared_show/infrared_show.h"

#include "common/utils.h"

void InfraredShow::Reset(cv::Mat& image) {
  image_ = image;
  canvas_.Reset();
}

const cv::Mat& InfraredShow::Display() {
  // cv::hconcat(image_, canvas_.Image(), display_);
  // return display_;
  return image_;
}

void InfraredShow::Undistort() {
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

void InfraredShow::AddInfo(ImageInfo& info) {
  char buf[64];
  snprintf(buf, sizeof(buf), "frame: %d", info.frame_id);
  // snprintf(buf, sizeof(buf), "%s frame: %d", info.scene_name.c_str(),
  //          info.frame_id);
  cv::putText(canvas_.Image(), buf, cv::Point(30, 30), 0, 0.5,
              cv::Scalar(255, 255, 255));
}

void InfraredShow::DrawObjDetect(const DETECT_OBJS& objs, bool frontail) {
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

void InfraredShow::DrawTrackObjs(const TRACK_OBJS& objs, bool frontail) {
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