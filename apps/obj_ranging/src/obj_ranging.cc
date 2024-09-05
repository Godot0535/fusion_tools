#include "apps/obj_ranging/include/obj_ranging.h"

#include <filesystem>
#include <fstream>

#include "common/utils.h"

namespace obj_ranging {
ObjRanging::ObjRanging() {
  config_.ReadConfig("../config/config.toml");
  obj_track_reader_.ReadData(config_.obj_track_path);
  lane_detect_reader_.ReadData(config_.lane_detect_path);
  lane_track_reader_.ReadData(config_.lane_track_path);

  mr::MonocularRangingConfig config;
  // 泾河/宝鸡场景库相对mobileye外参
  config.camera_parameters.extrinsic << 0.02615814, 0.12711313,
      0.99154338, -0.52679688, -0.99958545, 0.01527944, 0.02441147, 0.25954332,
      -0.01204725, -0.99177095, 0.12746104, 2.22195916, 0., 0., 0., 1.0;
  // 2024-5路测外参
  // config.camera_parameters.camera_extrinsic << -0.0241012, -0.05497147,
  //     0.99819701, -0.16086017, -0.99952542, -0.01783679, -0.02511556,
  //     -0.01273738, 0.01918527, -0.9983286, -0.0545155, 2.22733529, 0., 0.,
  //     0., 1.;
  // 2024-6路测外参
  // config.camera_parameters.camera_extrinsic << -0.04548597, 0.03935016,
  //     0.99818966, -0.1732951, -0.99873795, -0.02309334, -0.04460058,
  //     0.00985542, 0.0212965, -0.99895859, 0.04035092, 2.195997, 0., 0.,
  //     0., 1.;
  config.camera_parameters.Init();
  ranging_.SetParameter(config);

  if (config_.display_switch) {
    std::filesystem::path image_path = config_.image_path;
    image_info_.scene_name = image_path.filename().string();
    switch (config_.image_mode) {
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
    mobileye_reader_.ReadData(config_.mobileye_path);
  }
}

bool ObjRanging::Run() {
  std::cout << std::endl;
  TRACK_OBJS track_objs = obj_track_reader_.GetData(frame_id_);
  const DETECT_LANES &detect_lanes = lane_detect_reader_.GetData(frame_id_);
  const TRACK_LANES &track_lanes = lane_track_reader_.GetData(frame_id_);
  mr::DoubleLine left_det, right_det;
  for (size_t i = 0; i < detect_lanes.lanes_num; i++) {
    if (detect_lanes.lanes_pxl[i].lane_id == 1) {
      left_det.param_num = detect_lanes.lanes_pxl[i].param_num;
      left_det.first.start_y = detect_lanes.lanes_pxl[i].start_y0;
      left_det.first.end_y = detect_lanes.lanes_pxl[i].end_y0;
      left_det.first.c0 = detect_lanes.lanes_pxl[i].c00;
      left_det.first.c1 = detect_lanes.lanes_pxl[i].c01;
      left_det.first.c2 = detect_lanes.lanes_pxl[i].c02;
      left_det.first.c3 = detect_lanes.lanes_pxl[i].c03;
      left_det.second.start_y = detect_lanes.lanes_pxl[i].start_y1;
      left_det.second.end_y = detect_lanes.lanes_pxl[i].end_y1;
      left_det.second.c0 = detect_lanes.lanes_pxl[i].c10;
      left_det.second.c1 = detect_lanes.lanes_pxl[i].c11;
      left_det.second.c2 = detect_lanes.lanes_pxl[i].c12;
      left_det.second.c3 = detect_lanes.lanes_pxl[i].c13;
    } else if (detect_lanes.lanes_pxl[i].lane_id == 2) {
      right_det.param_num = detect_lanes.lanes_pxl[i].param_num;
      right_det.first.start_y = detect_lanes.lanes_pxl[i].start_y0;
      right_det.first.end_y = detect_lanes.lanes_pxl[i].end_y0;
      right_det.first.c0 = detect_lanes.lanes_pxl[i].c00;
      right_det.first.c1 = detect_lanes.lanes_pxl[i].c01;
      right_det.first.c2 = detect_lanes.lanes_pxl[i].c02;
      right_det.first.c3 = detect_lanes.lanes_pxl[i].c03;
      right_det.second.start_y = detect_lanes.lanes_pxl[i].start_y1;
      right_det.second.end_y = detect_lanes.lanes_pxl[i].end_y1;
      right_det.second.c0 = detect_lanes.lanes_pxl[i].c10;
      right_det.second.c1 = detect_lanes.lanes_pxl[i].c11;
      right_det.second.c2 = detect_lanes.lanes_pxl[i].c12;
      right_det.second.c3 = detect_lanes.lanes_pxl[i].c13;
    }
  }

  mr::Line left_bev, right_bev;
  for (size_t i = 0; i < track_lanes.lanes_num; i++) {
    if (track_lanes.lanes_wrd[i].lane_id == 1) {
      left_bev.start_y = track_lanes.lanes_wrd[i].start_y0;
      left_bev.end_y = track_lanes.lanes_wrd[i].end_y0;
      left_bev.c0 = track_lanes.lanes_wrd[i].c00;
      left_bev.c1 = track_lanes.lanes_wrd[i].c01;
      left_bev.c2 = track_lanes.lanes_wrd[i].c02;
      left_bev.c3 = track_lanes.lanes_wrd[i].c03;
    } else if (track_lanes.lanes_wrd[i].lane_id == 2) {
      right_bev.start_y = track_lanes.lanes_wrd[i].start_y0;
      right_bev.end_y = track_lanes.lanes_wrd[i].end_y0;
      right_bev.c0 = track_lanes.lanes_wrd[i].c00;
      right_bev.c1 = track_lanes.lanes_wrd[i].c01;
      right_bev.c2 = track_lanes.lanes_wrd[i].c02;
      right_bev.c3 = track_lanes.lanes_wrd[i].c03;
    }
  }
  ranging_.UpdateSlope(left_det, right_det, left_bev, right_bev);

  std::vector<mr::TrackedObject> objs;
  for (size_t i = 0; i < track_objs.objs_num; i++) {
    // if (track_objs.track_objs[i].track_id != 1) continue;
    mr::TrackedObject obj;
    const TRACK_OBJ_INFO &track_obj = track_objs.track_objs[i];
    obj.track_id = track_obj.track_id;
    obj.type = static_cast<int>(track_obj.obj_info.obj_category);
    obj.left = track_obj.box.left_top_u;
    obj.right = track_obj.box.right_bottom_u;
    obj.top = track_obj.box.left_top_v;
    obj.bottom = track_obj.box.right_bottom_v;
    obj.sub_flag = track_obj.obj_info.sub_box_confidence > 0.0 ? true : false;
    // obj.sub_flag = false;
    obj.sub_left = track_obj.obj_info.sub_box.left_top_u;
    obj.sub_top = track_obj.obj_info.sub_box.left_top_v;
    obj.sub_right = track_obj.obj_info.sub_box.right_bottom_u;
    obj.sub_bottom = track_obj.obj_info.sub_box.right_bottom_v;
    objs.push_back(obj);
  }
  ranging_.UpdateTracks(objs, track_lanes.pitch,
                        static_cast<int>(track_lanes.pitch_pl));

  auto &id_map = ranging_.GetMap();
  // std::cout << "-----------------------------" << std::endl;
  // std::cout << "obj num: " << track_objs.objs_num << std::endl;
  std::string file_path =
      config_.obj_ranging_path + "/" + IndexString(frame_id_) + ".txt";
  std::ofstream file(file_path, std::ios::out | std::ios::trunc);
  for (size_t i = 0; i < track_objs.objs_num; i++) {
    const auto &measurer = id_map.find(track_objs.track_objs[i].track_id);
    if (measurer == id_map.end()) {
      continue;
    }
    if (measurer->second->IsActivate()) {
      file << track_objs.track_objs[i].track_id << " "
           << measurer->second->GetVerticalDistance() << " "
           << measurer->second->GetHorizontalDistance() << " "
           << measurer->second->GetVerticalVelocity() << " "
           << measurer->second->GetHorizontalVelocity() << " "
           << measurer->second->GetWidth() << " "
           << measurer->second->GetHeight() << " "
           << measurer->second->GetVerticalDistanceProb() << " "
           << measurer->second->GetHorizontalDistanceProb() << " "
           << measurer->second->GetVerticalVelocityProb() << " "
           << measurer->second->GetHorizontalVelocityProb() << " "
           << track_objs.track_objs[i].box.left_top_u << " "
           << track_objs.track_objs[i].box.left_top_v << " "
           << track_objs.track_objs[i].box.right_bottom_u << " "
           << track_objs.track_objs[i].box.right_bottom_v << " "
           << track_objs.track_objs[i].confidence << " "
           << (int)track_objs.track_objs[i].obj_info.obj_category << " "
           << track_objs.track_objs[i].obj_info.box.left_top_u << " "
           << track_objs.track_objs[i].obj_info.box.left_top_v << " "
           << track_objs.track_objs[i].obj_info.box.right_bottom_u << " "
           << track_objs.track_objs[i].obj_info.box.right_bottom_v << " "
           << track_objs.track_objs[i].obj_info.confidence << " "
           << track_objs.track_objs[i].obj_info.sub_box.left_top_u << " "
           << track_objs.track_objs[i].obj_info.sub_box.left_top_v << " "
           << track_objs.track_objs[i].obj_info.sub_box.right_bottom_u << " "
           << track_objs.track_objs[i].obj_info.sub_box.right_bottom_v << " "
           << track_objs.track_objs[i].obj_info.sub_box_confidence << std::endl;

      track_objs.track_objs[i].activate = true;
      track_objs.track_objs[i].long_distance =
          measurer->second->GetVerticalDistance();
      track_objs.track_objs[i].lateral_distance =
          measurer->second->GetHorizontalDistance();
    } else {
      track_objs.track_objs[i].activate = false;
    }
  }
  file.close();

  if (config_.display_switch) {
    cv::Mat image = cv::imread(image_files_[frame_id_]);
    show_.Reset(image);
    image_info_.frame_id = frame_id_;
    show_.AddInfo(image_info_);
    show_.DrawLaneDetect(detect_lanes);
    show_.DrawLaneTrack(track_lanes);
    show_.DrawTrackObjs(track_objs, true);
    show_.DrawMobileyeData(mobileye_reader_.GetObjectsData(frame_id_),
                           mobileye_reader_.GetLanesData(frame_id_));
    // show_.Undistort();
    cv::namedWindow("Image", cv::WINDOW_NORMAL);
    cv::imshow("Image", show_.Display());
    cv::resizeWindow("Image", 1920, 864);
    char key = static_cast<char>(cv::waitKey(config_.interval));
    if (key == 'q') {
      return false;
    }
  }

  if (obj_track_reader_.IsValid(++frame_id_) &&
      lane_detect_reader_.IsValid(frame_id_) &&
      lane_track_reader_.IsValid(frame_id_))
    return true;
  return false;
}

}  // namespace obj_ranging