//
// Created by lee on 24-6-13.
//

#pragma once
#include "common.h"
#include "percept_obstacle_output_define.h"
/// @brief RCAN数据

struct TRACK_LIST_STATUS {
  uint8_t Object_NofObjects;       // 轨迹个数
  uint16_t Object_MeasCounter;     // 目标跟踪观测次数
  uint8_t Object_InterfaceVersion; // 接口版本
};

/// @brief 通用信息
struct GENERAL_INFORMATION {
  uint8_t ID;      // 目标id
  float DistLon;   // 纵向距离
  float DistLat;   // 横向距离
  float VrelLon;   // 纵向速度
  uint8_t DynProp; // 运动类型
  float VrelLat;   // 横向速度
};

/// @brief 扩展信息
struct EXTENDED_INFORMATION {
  uint8_t ID;         // 目标id
  float CornerRelLen; // 非静态目标的长度参数，用来计算目标的长
  float OrientationAngle; // 航向角
  uint8_t Class;          // 目标类别
  float CornerRelWid; // 非静态目标的宽度参数，用来计算目标的宽
  float Length;       // 目标长度
  float Width;        // 目标宽度
  float ArelLon;      // 纵向加速度
  float ArelLat;      // 横向加速度
};

/// @brief 质量信息
struct QUALITY_INFORMATION {
  uint8_t ID;          // 目标id
  float SNR;           // 信噪比
  uint8_t DisLon_RMS;  // 纵向距离均方根
  uint8_t DisLat_RMS;  // 横向距离均方根
  uint8_t VrelLon_RMS; // 纵向速度均方根
  uint8_t ProbOfExist; // 目标存在概率
  uint8_t VrelLat_RMS; // 横向速度均方根
  uint8_t MeasState;   // 测量状态
  uint8_t age;         // 轨迹历史存在的帧数
  uint8_t count;       // 轨迹得到确认的帧数
};

// 标定状态和标定参数
struct CSRADAR_CALIBRATION_PARAMS {
  bool status;            // false:未标定完成，true:已标定完成
  double extrinsic[4][4]; // 外参
};

struct CSRADAR_OBSTACLE {
  DATA_HEADER data_header;                   // 数据头
  CSRADAR_CALIBRATION_PARAMS params;         // 标定参数
  RADAR_STATUS Radar_Status;                 // 雷达状态
  TRACK_LIST_STATUS Track_List_Status;       // 跟踪状态
  GENERAL_INFORMATION General_Information;   // 通用信息
  EXTENDED_INFORMATION Extended_Information; // 扩展信息
  QUALITY_INFORMATION Quality_Information;   // 质量信息
};