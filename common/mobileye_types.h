#pragma once
#include <vector>
#include <cstdint>

enum class OBSTACLE_TYPE {
  sedancar = 0,          // 汽车
  truck = 1,             // 卡车
  bicycle = 2,           // 自行车
  motorcycle = 3,        // 摩托电动车
  people = 4,            // 人
  zoon = 5,              // 动物
  general_obstacle = 6,  // 一般障碍物
  indeterminacy = 7,     // 不确定
};

struct MOBILEYE_OBSTACLE {
  int obj_id;                      // id
  OBSTACLE_TYPE e_obs_type;        // 类型
  float f_lateral_distance;        // 横向距离
  float f_long_distance;           // 纵向距离
  float f_obj_width;               // 物体宽度
  float f_obj_length;              // 物体长度
  float f_obj_height;              // 物体高度
  float f_relative_long_velocity;  // 纵向速度
  float f_relative_lat_velocity;   // 横向速度
};

struct MOBILEYE_LANE_SHAPE {
  uint64_t iAdsTimeStamp;
  int lane_type;
  float f_confidence;
  float f_start;
  float f_end;
  int role;
  float f_c0;
  float f_c1;
  float f_c2;
  float f_c3;
};

typedef std::vector<MOBILEYE_OBSTACLE> MOBILEYE_OBSTACLES;
typedef std::vector<MOBILEYE_LANE_SHAPE> MOBILEYE_LANE_SHAPES;