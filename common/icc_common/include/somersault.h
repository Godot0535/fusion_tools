#pragma once
#include "common.h"
#include "j3_common.h"
#include "percept_fusion_output_define.h"
#include <opencv2/core/core.hpp>

#define SENSOR_NAME_MAX_SIZE 128

/**
 * 统一雷达数据结构表示
 */
struct RadarMeta {
  int track_id;
  int type;
  double length;
  double width;
  double long_distance;
  double lateral_distance;
  double long_velocity;
  double lateral_velocity;
  double long_acceleration;
  double lateral_acceleration;
  double confidence;
  double theta;
  double long_distance_rms;
  double lateral_distance_rms;
  double long_velocity_rms;
  double lateral_velocity_rms;
  int motion_pattern;
  int measure_status;
};

constexpr int per_radar_frame_max_num = 100;

struct RadarFrame {
  int radar_num;
  uint64_t time_stamp;
  char sensor_name[SENSOR_NAME_MAX_SIZE];
  RadarMeta radar_meta[per_radar_frame_max_num];
};

typedef struct _SOMERSAULT_IMAGE_DATA {
  uint64_t frame_id;
  uint64_t ads_time_stamp;       // 板上自增时间戳
  cv::Mat data;
} SOMERSAULT_IMAGE_DATA, *LP_SOMERSAULT_IMAGE_DATA;

constexpr int fusion_cycle_max_sensor_num = 4;
typedef struct _SOMERSAULT_RESULT {
  PERCEP_FUSION_RESULTS percep_fusion_results;
  J3_RESULTS j3_results;
  int vehicle_num;
  VEHICLE_KINETIC_INFO vehicle_kinetic_group[fusion_cycle_max_sensor_num];
  int radar_frame_num;
  RadarFrame radar_frames[fusion_cycle_max_sensor_num];
} SOMERSAULT_RESULT, *LP_SOMERSAULT_RESULT;