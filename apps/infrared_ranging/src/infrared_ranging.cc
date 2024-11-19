#include "apps/infrared_ranging/include/infrared_ranging.h"

#include <filesystem>
#include <fstream>

#include "common/utils.h"
namespace infrared_ranging {

InfraredRanging::InfraredRanging() {
  config_.ReadConfig("../config/config.toml");
  obj_track_reader_.ReadData(config_.infrared_track_path);

  mr::MonocularRangingConfig config;
  config.camera_parameters.Init();
  ranging_.SetParameter(config);

  if (config_.display_switch) {
    std::filesystem::path image_path = config_.image_path;
    image_info_.scene_name = image_path.filename().string();
    switch (config_.image_mode) {
      case 0:  // 0-全是图片的文件夹
        std::cout << config_.image_path << std::endl;
        GetFileNames(config_.image_path, image_files_);
        break;
      default:
        break;
    }
  }

  if (config_.video_save) {
    video_writer.open(config_.video_save_path,
                      cv::VideoWriter::fourcc('X', 'V', 'I', 'D'),
                      30, cv::Size(640, 512), true);
                      // 30, cv::Size(880, 512), true);
    if (!video_writer.isOpened()) {
      std::cerr << "Unable to open video file." << std::endl;
    }
  }
}

bool InfraredRanging::Run() {
  TRACK_OBJS track_objs = obj_track_reader_.GetData(frame_id_);

  std::vector<mr::TrackedObject> objs;
  for (size_t i = 0; i < track_objs.objs_num; i++) {
    mr::TrackedObject obj;
    const TRACK_OBJ_INFO &track_obj = track_objs.track_objs[i];
    obj.track_id = track_obj.track_id;
    obj.type = static_cast<int>(track_obj.obj_info.obj_category);
    obj.left = track_obj.box.left_top_u;
    obj.right = track_obj.box.right_bottom_u;
    obj.top = track_obj.box.left_top_v;
    obj.bottom = track_obj.box.right_bottom_v;
    objs.push_back(obj);
    std::cout << obj.track_id << " " << obj.type << " " << obj.left << " " << obj.right << " " << obj.top << " " << obj.bottom << std::endl;
  }
  ranging_.UpdateTracks(objs, -0.06981317007977318);

  auto &id_map = ranging_.GetMap();
  // std::string file_path =
  //     config_.infrared_ranging_path + "/" + IndexString(frame_id_) + ".txt";
  // std::ofstream file(file_path, std::ios::out | std::ios::trunc);
  for (size_t i = 0; i < track_objs.objs_num; i++) {
    const auto &measurer = id_map.find(track_objs.track_objs[i].track_id);
    if (measurer == id_map.end()) {
      continue;
    }
    if (measurer->second->IsActivate()) {
      // file << track_objs.track_objs[i].track_id << " "
      //      << measurer->second->GetVerticalDistance() << " "
      //      << measurer->second->GetHorizontalDistance() << " "
      //      << measurer->second->GetVerticalVelocity() << " "
      //      << measurer->second->GetHorizontalVelocity() << " "
      //      << measurer->second->GetWidth() << " "
      //      << measurer->second->GetHeight() << " "
      //      << measurer->second->GetVerticalDistanceProb() << " "
      //      << measurer->second->GetHorizontalDistanceProb() << " "
      //      << measurer->second->GetVerticalVelocityProb() << " "
      //      << measurer->second->GetHorizontalVelocityProb() << " "
      //      << track_objs.track_objs[i].box.left_top_u << " "
      //      << track_objs.track_objs[i].box.left_top_v << " "
      //      << track_objs.track_objs[i].box.right_bottom_u << " "
      //      << track_objs.track_objs[i].box.right_bottom_v << " "
      //      << track_objs.track_objs[i].confidence << " "
      //      << (int)track_objs.track_objs[i].obj_info.obj_category << " "
      //      << track_objs.track_objs[i].obj_info.box.left_top_u << " "
      //      << track_objs.track_objs[i].obj_info.box.left_top_v << " "
      //      << track_objs.track_objs[i].obj_info.box.right_bottom_u << " "
      //      << track_objs.track_objs[i].obj_info.box.right_bottom_v << " "
      //      << track_objs.track_objs[i].obj_info.confidence << " "
      //      << track_objs.track_objs[i].obj_info.sub_box.left_top_u << " "
      //      << track_objs.track_objs[i].obj_info.sub_box.left_top_v << " "
      //      << track_objs.track_objs[i].obj_info.sub_box.right_bottom_u << " "
      //      << track_objs.track_objs[i].obj_info.sub_box.right_bottom_v << " "
      //      << track_objs.track_objs[i].obj_info.sub_box_confidence << std::endl;

      track_objs.track_objs[i].activate = true;
      track_objs.track_objs[i].long_distance =
          measurer->second->GetVerticalDistance();
      // track_objs.track_objs[i].lateral_distance =
      //     measurer->second->GetHorizontalDistance();
    } else {
      track_objs.track_objs[i].activate = false;
    }
  }
  // file.close();

  if (config_.display_switch) {
    cv::Mat image = cv::imread(image_files_[frame_id_]);
    infrared_show_.Reset(image);
    image_info_.frame_id = frame_id_;
    infrared_show_.AddInfo(image_info_);
    infrared_show_.DrawTrackObjs(track_objs, false);
    // infrared_show_.Undistort();
    cv::namedWindow("Image", cv::WINDOW_NORMAL);
    cv::imshow("Image", infrared_show_.Display());
    cv::resizeWindow("Image", 640, 512);
    // cv::resizeWindow("Image", 880, 512);

    if (config_.video_save) {
      video_writer.write(infrared_show_.Display());
    }

    char key = static_cast<char>(cv::waitKey(config_.interval));
    if (key == 'q') {
      return false;
    }
  }

  if (obj_track_reader_.IsValid(++frame_id_)) return true;
  return false;
}

}  // namespace infrared_ranging