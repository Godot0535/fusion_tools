//
// Created by lee on 24-6-13.
//

#pragma once

#include "common.h"
#include <stdint.h>

// 车道线相关
#define LANES_MAX_NUM 10

/// @brief 变道信息
enum class LANE_CHANGE {
  NO_LANE_CHANGE,    // 未变道
  LANE_CHANGE_LEFT,  // 向左变道
  LANE_CHANGE_RIGHT, // 向右变道
};

/// @brief 车道线基础信息
struct LANE_EVENT {
  uint8_t NumberOfLanes;      // 车道线数量
  uint8_t AssignedLaneNumber; // 车道线编号
  LANE_CHANGE LaneChange;
};

/// @brief 临时车道线信息
struct LANE_TEMP {
  float C0First;         // C0
  float C1First;         // C1
  float C2First;         // C2
  float C3First;         // C3
  float C0FirstVariance; // C0方差
  float C1FirstVariance; // C1方差
  float C2FirstVariance; // C2方差
  float C3FirstVariance; // C3方差
  uint8_t Id;
  float FirstStartPoint; // 纵向起点
  float FirstEndPoint;   // 纵向终点
  float C0Sec;           // 第二段C0
  float C1Sec;           // 第二段C1
  float C2Sec;           // 第二段C2
  float C3Sec;           // 第二段C3
  float C0SecVariance;   // 第二段C0方差
  float C1SecVariance;   // 第二段C1方差
  float C2SecVariance;   // 第二段C2方差
  float C3SecVariance;   // 第二段C3方差
  float SecStartPoint;   // 第二段纵向起点
  float SecEndPoint;     // 第二段纵向终点
  bool IsSecValid;       // 第二段车道线是否有效
  bool Valid;
  float MeasQly;
  float MdlQly;
  uint8_t Checksum; // 校验和
  uint8_t Counter;  // 计数器
  uint32_t TimeStamp;
};

/// @brief 路沿跟踪信息
enum class ROAD_EDGE_TRACK_STS {
  INVALID,               // 无效
  TRACKED,               // 跟踪
  CLOSE_RANGE_PREDICTED, // 近距离预测
  FAR_RANGE_PREDICTED,   // 远距离预测
  PREDICTED,             // 预测
};

/// @brief 左路沿信息
typedef struct ROAD_EDGE_META {
  float C0;                  // C0
  float C1;                  // C1
  float C2;                  // C2
  float C3;                  // C3
  float C0Variance;          // C0方差
  float C1Variance;          // C1方差
  float C2Variance;          // C2方差
  float C3Variance;          // C3方差
  uint8_t Id;                // 路沿编号（0：；1：；2：；3：）
  float LongDistanceToStart; // 纵向起点
  float LongDistanceToEnd;   // 纵向终点
  float MdlQly;
  float MeasQly;
  uint8_t Checksum; // 校验和
  uint8_t Counter;  // 计数器
  uint32_t TimeStamp;
  ROAD_EDGE_TRACK_STS TrackSts; // 路沿跟踪状态
  bool Valid;
} ROAD_EDGE_LEFT, ROAD_EDGE_RIGHT;

/// @brief 第二段车道线是否有效
enum class IS_SECOND_VALID {
  NOT_VALID,
  VALID,
};

/// @brief 车道线颜色
enum class LINE_COLOR {
  UNKNOWN, // 未知
  WHITE,   // 白色
  YELLOW,  // 黄色
  RED,     // 红色
  ORANGE,  // 橙色
  GREEN,   // 绿色
  BLUE,    // 蓝色
};

/// @brief 车道线类型
enum class LINE_TYPE {
  SOLID,         // 实线
  DASHED,        // 虚线
  DOUBLE_SOLID,  // 双实线
  DOUBLE_DASHED, // 双虚线
  SOLID_DASHED,  // 实虚线
  DASHED_SOLID,  // 虚实线
  FISHBONE,      // 鱼骨线
  STOP_LINE,     // 停止线
  UNKNOWN,       // 未知
};

/// @brief pitch估计质量标志
enum class PITCH_QUALITY {
  POOL = 0,      // 差
  MEDIUM = 1,    // 中
  GOOD = 2,      // 良
  EXCELLENT = 3, // 优
};

/// @brief 车道线跟踪信息
enum class LANE_TRACK_STS {
  NO_MARKING_DETECTED,           // 未检测到
  TRACKED_DETECTED_MARKING,      // 跟踪检测到
  CLOSE_RANGE_PREDICTED_MARKING, // 近距离预测到
  FAR_RANGE_PREDICTED_MARKING,   // 远距离预测
  PREDICTED_MARKING,             // 预测到
};

/// @brief 车道线信息
typedef struct LANE_META {
  float C0Minus;              // 后向C0
  float C1Minus;              // 后向C1
  float C2Minus;              // 后向C2
  float C3Minus;              // 后向C3
  float C0MinusVariance;      // 后向C0方差
  float C1MinusVariance;      // 后向C1方差
  float C2MinusVariance;      // 后向C2方差
  float C3MinusVariance;      // 后向C3方差
  float MinusStartPoint;      // 后向纵向起点
  float MinusEndPoint;        // 后向纵向终点
  float C0First;              // C0
  float C1First;              // C1
  float C2First;              // C2
  float C3First;              // C3
  float C0FirstVariance;      // C0方差
  float C1FirstVariance;      // C1方差
  float C2FirstVariance;      // C2方差
  float C3FirstVariance;      // C3方差
  float FirstStartPoint;      // 纵向起点
  float FirstEndPoint;        // 纵向终点
  float C0Sec;                // 第二段C0
  float C1Sec;                // 第二段C1
  float C2Sec;                // 第二段C2
  float C3Sec;                // 第二段C3
  float C0SecVariance;        // 第二段C0方差
  float C1SecVariance;        // 第二段C1方差
  float C2SecVariance;        // 第二段C2方差
  float C3SecVariance;        // 第二段C3方差
  float SecStartPoint;        // 第二段纵向起点
  float SecEndPoint;          // 第二段纵向起点
  IS_SECOND_VALID IsSecValid; // 第二段车道线是否有效
  float PriStartPoint;
  float PriEndPoint;
  uint8_t Id; // 车道线编号（0：；1：；2：；3：）
  bool Valid;
  LINE_COLOR Color;        // 车道线颜色
  LINE_TYPE Type;          // 车道线类型
  LANE_TRACK_STS TrackSts; // 车道线跟踪信息
  float MeasQly;
  float MdlQly;
  float LineWidth;  // 车道线宽
  uint8_t Checksum; // 校验和
  uint8_t Counter;  // 计数器
  bool IsVerified;
  uint32_t TimeStamp;
  float TypChgPoint;
  LINE_TYPE TypAftChgPoint;
} LANE_LEFT, LANE_RIGHT, LANE_SECOND_LEFT, LANE_SECOND_RIGHT;

struct WUKONG_LANES {
  LANE_TEMP lane_temp;
  ROAD_EDGE_LEFT road_edge_left;
  ROAD_EDGE_RIGHT road_edge_right;
  LANE_LEFT lane_left;
  LANE_RIGHT lane_right;
  LANE_SECOND_LEFT lane_second_left;
  LANE_SECOND_RIGHT lane_second_right;
};

/// @brief 车道线类型定义
enum class CATEGORY {
  UNKNOWN = 0,            // 未知
  STOP_LINE = 1,          // 停车线
  SOLID_LINE = 2,         // 实线
  DOTTED_LINE = 3,        // 虚线
  DOUBLE_SOLID_LINE = 4,  // 双实线
  DOUBLE_DASHED_LINE = 5, // 双虚线
  DASHED_SOLID_LINE = 6,  // 实虚线
  CURB = 7,               // 路沿
};

/// @brief 车道线颜色定义
enum class COLOR {
  UNKNOWN = 0, // 未知
  WHITE = 1,   // 白色
  YELLOW = 2,  // 黄色
  BLUE = 3,    // 蓝色
  ORANGE = 4,  // 橙色
};

typedef struct _DETECT_LANE_INFO {
  int lane_id;
  int param_num;
  double start_y0;   // 车道起始y
  double end_y0;     // 车道终点y
  double start_y1;   // 车道起始y
  double end_y1;     // 车道终点y
  double c00;        // 常系数
  double c01;        // y^1系数
  double c02;        // y^2系数
  double c03;        // y^3系数
  double c10;        // 常系数
  double c11;        // y^1系数
  double c12;        // y^2系数
  double c13;        // y^3系数
  CATEGORY category; // 车道线类型
  COLOR color;       // 车道线颜色
} DETECT_LANE_INFO, *LP_DETECT_LANE_INFO;

/// @brief 检测车道线
typedef struct _DETECT_LANES {
  DATA_HEADER data_header;
  int lanes_num;                             // 车道线数量, 最大10
  DETECT_LANE_INFO lanes_pxl[LANES_MAX_NUM]; // 车道线信息(pixel)
  DETECT_LANE_INFO lanes_wrd[LANES_MAX_NUM]; // 车道线信息(world)
} DETECT_LANES, *LP_DETECT_LANES;

/// @brief 车道线检测状态
enum class MEASURING_STATE {
  UNKNOWN = 0,    // 未知
  NEW = 1,        // 新增
  PARALLEL = 2,   // 平行
  UNPARALLEL = 3, // 非平行
};

typedef struct _TRACK_LANE_INFO {
  int lane_id;
  int param_num;
  double start_y0;   // 车道起始y
  double end_y0;     // 车道终点y
  double c00;        // 常系数
  double c01;        // y^1系数
  double c02;        // y^2系数
  double c03;        // y^3系数
  double start_y1;   // 车道起始y
  double end_y1;     // 车道终点y
  double c10;        // 常系数
  double c11;        // y^1系数
  double c12;        // y^2系数
  double c13;        // y^3系数
  CATEGORY category; // 车道线类型，由检测给出，暂无，预留
  COLOR color;       // 车道线颜色，由检测给出，暂无，预留
  float quality;    // 车道线检测质量，由检测给出，暂无，预留
  int side;         // 车道左右侧区分标志位，0-左 1-右
  float confidence; // 车道线置信度，根据跟踪误差给出
  MEASURING_STATE
  measuring_state; // 车道线检测状态，判断平行/非平行/新增/未知，目前强制平行
  int crossing;    // 跨越车道线标志位，判断是否换道
} TRACK_LANE_INFO, *LP_TRACK_LANE_INFO;

/// @brief 跟踪车道线
typedef struct _TRACK_LANES {
  DATA_HEADER data_header;
  float pitch;
  float width;
  float offset;
  PITCH_QUALITY pitch_pl;
  int lanes_num;                            // 车道线数量, 最大10
  TRACK_LANE_INFO lanes_pxl[LANES_MAX_NUM]; // 车道线信息(pixel)
  TRACK_LANE_INFO lanes_wrd[LANES_MAX_NUM]; // 车道线信息(world)
} TRACK_LANES, *LP_TRACK_LANES;