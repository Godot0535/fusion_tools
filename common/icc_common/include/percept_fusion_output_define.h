#pragma once
#include "common.h"
#include "percept_lane_output_define.h"
#include "percept_obstacle_output_define.h"

typedef struct _SYNC_RESULTS {
  DATA_HEADER data_header;
  DETECT_LANES detect_lanes;
  TRACK_LANES track_lanes;
  WUKONG_LANES pnc_lanes;
  TRACK_OBJS track_objs;
} SYNC_RESULTS, *LP_SYNC_RESULTS;

// 8. 后融合数据
/// @brief 融合障碍物
typedef struct FUSION_OBJ_INFO {
  unsigned int track_id;        // 融合目标的track ID
  float distance_x;             // 融合目标的纵向距离
  float distance_y;             // 融合目标的横向距离
  float distance_z;             // 融合目标的高度
  float velocity_x;             // 融合目标的纵向相对速度
  float velocity_y;             // 融合目标的横向相对速度
  float acceleration_x;         // 融合目标的纵向相对加速度
  float box_size_x;             // 融合目标的长度
  float box_size_y;             // 融合目标的宽度
  float box_size_z;             // 融合目标的高度
  float box_angle;              // 融合目标的航向角
  OBJ_CATEGORY obj_category;    // 融合目标的类型
  unsigned int detection_flag;  // 融合目标的探测属性
  MotionPattern motion_pattern; // 融合目标的运动属性
  unsigned int camera_id;       // 融合目标对应的摄像头目标ID
  unsigned int radar_id;        // 融合目标对应的雷达目标ID
  float obj_age;                // 融合目标的跟踪时长
  TrackStatus track_status;     // 融合目标的跟踪状态
  float confidence;             // 融合目标的置信度
} FUSION_OBJ_INFO, *LP_FUSION_OBJ_INFO;

/// @brief 融合障碍物数组
typedef struct _FUSION_OBJS {
  DATA_HEADER data_header;
  int objs_num;
  FUSION_OBJ_INFO fusion_objs[OBJS_MAX_NUM];
} FUSION_OBJS, *LP_FUSION_OBJS;

/// @brief 感知融合结果
typedef struct _PERCEP_FUSION_RESULTS {
  SYNC_RESULTS sync_results;
  WUKONG_OBJS fusion_objs;
} PERCEP_FUSION_RESULTS, *LP_PERCEP_FUSION_RESULTS;