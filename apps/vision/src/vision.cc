#include "apps/vision/include/vision.h"

#include <filesystem>

#include "common/utils.h"

namespace vision {
Vision::Vision() {
  config_.ReadConfig("../config/config.toml");
  if (config_.obj_detect) obj_detect_reader_.ReadData(config_.obj_detect_path);
  if (config_.obj_track) obj_track_reader_.ReadData(config_.obj_track_path);
  if (config_.lane_detect)
    lane_detect_reader_.ReadData(config_.lane_detect_path);
  if (config_.lane_track) lane_track_reader_.ReadData(config_.lane_track_path);
  if (config_.obj_ranging)
    obj_ranging_reader_.ReadData(config_.obj_ranging_path);
  if (config_.mobileye) mobileye_reader_.ReadData(config_.mobileye_path);

  std::filesystem::path image_path = config_.image_path;
  image_info_.scene_name = image_path.filename().string();
  image_info_.frame_id = 0;
  switch (config_.directory_mode) {
    case 0:  // 0-全是图片的文件夹
      GetFileNames(config_.image_path, image_files_);
      break;
    case 1: {  // 1-真值场景文件夹
      std::vector<std::string> truth_folders;
      GetAllFolders(config_.image_path, truth_folders);
      for (auto &folder : truth_folders) {
        image_files_.push_back(GetTruthImage(folder));
      }
      break;
    }
    default:
      break;
  }
  if (config_.video_save) {
    video_writer.open(config_.video_save_path,
                      cv::VideoWriter::fourcc('X', 'V', 'I', 'D'),
                      config_.video_fps, cv::Size(2400, 1080), true);
    if (!video_writer.isOpened()) {
      std::cerr << "Unable to open video file." << std::endl;
    }
  }
}

bool Vision::Run() {
  ProcessImage();

  if (config_.display_switch) {
    cv::namedWindow("Image", cv::WINDOW_NORMAL);
    cv::imshow("Image", show_.Display());
    cv::resizeWindow("Image", 1920, 864);
    char key = static_cast<char>(cv::waitKey(config_.interval));
    if (key == 'q') {
      return false;
    } else if (key == 'l') {
      image_info_.frame_id =
          image_info_.frame_id == 0 ? 0 : image_info_.frame_id - 1;
    } else {
      if (image_info_.frame_id == image_files_.size() - 1) {
        if (config_.interval != 0) return false;
      } else {
        image_info_.frame_id++;
      }
    }
  }

  return true;
}

bool Vision::SaveVideo() {
  ProcessImage();
  video_writer.write(show_.Display());
  if (image_info_.frame_id == image_files_.size() - 1) {
    return false;
  }
  image_info_.frame_id++;
  return true;
}

void Vision::ProcessImage() {
  image_ = cv::imread(image_files_[image_info_.frame_id]);
  show_.Reset(image_);
  show_.AddInfo(image_info_);

  if (config_.obj_detect) {
    const DETECT_OBJS &objs = obj_detect_reader_.GetData(image_info_.frame_id);
    show_.DrawObjDetect(objs, true);
  }
  if (config_.obj_track && !config_.obj_ranging) {
    const TRACK_OBJS &objs = obj_track_reader_.GetData(image_info_.frame_id);
    bool flag = !config_.obj_detect;
    show_.DrawTrackObjs(objs, flag);
  }
  if (config_.lane_detect) {
    const DETECT_LANES &lanes =
        lane_detect_reader_.GetData(image_info_.frame_id);
    show_.DrawLaneDetect(lanes);
  }
  if (config_.lane_track) {
    const TRACK_LANES &lanes = lane_track_reader_.GetData(image_info_.frame_id);
    show_.DrawLaneTrack(lanes);
  }
  if (config_.obj_ranging) {
    const TRACK_OBJS &objs = obj_ranging_reader_.GetData(image_info_.frame_id);
    bool flag = !config_.obj_detect;
    show_.DrawTrackObjs(objs, flag);
  }
  if (config_.mobileye) {
    const MOBILEYE_OBSTACLES &objs =
        mobileye_reader_.GetObjectsData(image_info_.frame_id);
    const MOBILEYE_LANE_SHAPES &lanes =
        mobileye_reader_.GetLanesData(image_info_.frame_id);
    show_.DrawMobileyeData(objs, lanes);
  }

  if (config_.undistort) show_.Undistort();
}

}  // namespace vision
