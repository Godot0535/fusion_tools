#ifndef _INCLUDE_J3_COMMON_H_
#define _INCLUDE_J3_COMMON_H_

#include "common.h"

/// @brief J3感知，目标输出

// @brief 目标类型
enum class J3_OBJ_TYPE {
  INVALID,
  VEHICLE,
  PEDESTRIAN,
  CYCLIST,
  GENERAL_OBJECT,
};

// @brief 目标的探测属性
enum class J3_OBJ_DETECTION_FLAG {
  DETECTED,
  DETECTING,
};

// @brief 目标的运动属性
enum class J3_MOTION_STATUS {
  INVALID,
  UNKNOWN,
  MOVING,
  STATIONARY,
  STOPPED,
  MOVING_SLOWLY,
};

// @brief 目标的跟踪状态
enum class J3_TRACK_STATUS {
  INVALID,
  NEW,
  MEASURED,
  COASTED,
};

// @brief 目标的运动状态
enum class J3_MOTION_CATEGORY {
  INVALID,
  UNDEFINED,
  PASSING,
  PASSING_IN,
  PASSING_OUT,
  CLOSE_CUT_IN,
  MOVING_IN,
  MOVING_OUT,
  CROSSING,
  LTAP,
  RTAP,
  MOVING,
  PRECEEDING,
  ONCOMING,
};

// @brief 转向灯状态
enum class J3_BLINKER_INFO {
  UNAVAILABLE,
  OFF,
  LEFT,
  RIGHT,
  BOTH,
};

// @brief 刹车灯状态
enum class J3_BRAKE_LIGHTS {
  OFF,
  ON,
};

// @brief Cutin状态
enum class J3_CUTIN_FLAG {
  NOT_CUT_IN,
  CUT_IN,
  CLOSE_CUT_IN,
};

// @brief 行人朝向
enum class J3_PED_ORIENTATION {
  UNKNOWN,
  BACK,
  FRONT,
  LEFT,
  LEFT_FRONT,
  LEFT_BACK,
  RIGHT,
  RIGHT_FRONT,
  RIGHT_BACK,
};

// @brief 车辆类型
enum class J3_VEHICLE_SUBTYPE {
  UNKNOWN,           // 未知
  BUS,               // 巴士
  SMALL_MEDIUM_CAR,  // 小型中型车
  TRUCKS,            // 卡车
  MOTORS,            // Tri-cycle
  SPECIAL_VEHICLE,   // 特种车
  TINY_CAR,          // 微型车
  LORRY,             // 卡车
};

// @brief 目标所在车道
enum class J3_OBJ_LANE_ASSIGNMENT {
  UNKNOWN,
  LEFT_LEFT,
  LEFT,
  HOST,
  RIGHT,
  RIGHT_RIGHT,
};

struct J3_OBJ_INFO {
  uint8_t CIPV_ID;
  uint8_t MCP_ID;
  float CIPV_TTC;
  float MCP_TTC;
  float Distance_X;                            // 目标纵向距离
  float Distance_Y;                            // 目标横向距离
  float Velocity_X;                            // 目标纵向相对速度
  float Velocity_Y;                            // 目标横向相对速度
  float Acceleration_X;                        // 目标纵向相对加速度
  float BoxSize_X;                             // 目标长度
  float BoxSize_Y;                             // 目标宽度
  float BoxAngle;                              // 目标航向角
  J3_OBJ_TYPE Type;                            // 目标类型
  J3_OBJ_DETECTION_FLAG Ddetection_Flag;       // 目标的探测属性
  J3_MOTION_STATUS Motion_Status;              // 目标的运动属性
  uint8_t ID_Camera;                           // 目标对应的摄像头ID
  float Age;                                   // 目标的跟踪时长
  J3_TRACK_STATUS Track_Status;                // 目标的跟踪状态
  uint8_t Confidence;                          // 目标的置信度
  J3_MOTION_CATEGORY Motion_Category;          // 目标运动状态
  J3_BLINKER_INFO Blinker_Info;                // 转向灯状态
  J3_BRAKE_LIGHTS Brake_Lights;                // 刹车灯状态
  float Cutin_Distance;                        // Cutin距离
  J3_CUTIN_FLAG Cutin_Flag;                    // Cutin状态
  J3_PED_ORIENTATION Ped_Orientation;          // 行人朝向
  J3_VEHICLE_SUBTYPE Veh_Subtype;              // 车辆类型
  J3_OBJ_LANE_ASSIGNMENT Obj_Lane_Assignment;  // 目标所在车道
  bool CIPV_Flag;                              // 目标是否CIPV
};

struct J3_OBJS {
  uint8_t quantity;  // 有效目标数量
  J3_OBJ_INFO objs[20];
};

/// @brief J3感知，车道线输出

/// @brief 检测状态
enum class J3_MEASURING_STATE {
  Unknown,
  New,
  Measured_Parallel,
  Predicted_Parallel,
  Measured_Unparallel,
  Predicted_Unparallel,
  Not_valid,
};

/// @brief 车道线类型
enum class J3_LINE_TYPE {
  Unknown,
  Solid,
  RoadEdge,
  Dashed,
  DoubleLane_DashedSolid,
  DoubleLane_SolidDashed,
  DoubleLane_DoubleDashed,
  DoubleLane_DoubleSolid,
  LineRamp_Solid,
  LineRamp_Dashed,
  ShadedArea,
  DecelerationSolidLine,
  DecelerationDashedLine,
  LaneVirtualMarking,
  Invalid,
};

/// @brief 车道线颜色
enum class J3_LINE_COLOR {
  Unknown,
  White,
  Yellow_Orange_Red,
  Blue_Green,
};

/// @brief 车道线质量
enum class J3_LINE_QUALITY {
  Unknown,
  Low,
  High,
  Very_High,
};

/// @brief 车道线信息
typedef struct J3_LANE_META {
  unsigned int Confidence;             // 置信度
  J3_MEASURING_STATE Measuring_State;  // 检测状态
  J3_LINE_TYPE Type;                   // 车道线类型
  J3_LINE_COLOR Color;                 // 车道线颜色
  J3_LINE_QUALITY Quality;             // 车道线质量
  bool Crossing;                       // 跨越车道线标志位
  float Rangestart;                    // 车道线检测起始距离
  float Rangeend;                      // 车道线检测终止距离
  float C0;                            // C0
  float C1;                            // C1
  float C2;                            // C2
  float C3;                            // C3
} J3_LANE_LEFT, J3_LANE_RIGHT, J3_LANE_SECOND_LEFT, J3_LANE_SECOND_RIGHT;

struct J3_LANES {
  J3_LANE_LEFT j3_lane_left;
  J3_LANE_RIGHT j3_lane_right;
  J3_LANE_SECOND_LEFT j3_lane_second_left;
  J3_LANE_SECOND_RIGHT j3_lane_second_right;
};

/// @brief J3 Traffic Sign

enum class J3_TSR_RELEVANCY {
  Undefined,
  IN_HOSTLANE_SIGN,     // Reserved
  NOT_IN_HOSTLANE,      // Reserved
  HIGHWAY_EXIT_SIGN,    // Reserved
  SIGN_ON_TURN,         // Reserved
  FAR_IRRELEVANT_SIGN,  // Reserved
  TSR_RAMP_RELEVANT,
};

enum class J3_TSR_FILTER_TYPE {
  Not_filtered,
  Irrelevant_to_the_host_driver,
  On_vehicle_or_truck,
  Embedded,  // Reserved
  RESERVED0,
  RESERVED1,
  RESERVED2,
  RESERVED3,
};

enum class J3_TSR_APPROVED_FLAG {
  Not_Approved,
  Approved,
};

struct J3_TRAFFIC_SIGN {
  unsigned int ID;                    // TSR ID
  unsigned int Confidence;            // TSR置信度
  unsigned int TSRTrafficSignType;    // TSR类别
  unsigned int TSRRoadMarkType;       // 预留，0=Default
  J3_TSR_RELEVANCY Relevancy;         // TSR相关性
  J3_TSR_FILTER_TYPE FilterType;      // 预留，TSR过滤原因
  J3_TSR_APPROVED_FLAG ApprovedFlag;  // 预留
  float Position_X;                   // TSR纵向距离
  float Position_Y;                   // TSR横向距离
  float Position_Z;                   // TSR高度
};

struct J3_TRAFFIC_SIGNS {
  unsigned int num;  // TSR识别数量
  J3_TRAFFIC_SIGN j3_traffic_signs[8];
};

/// @brief J3感知结果
struct J3_RESULTS {
  DATA_HEADER data_header;
  uint64_t image_time_stamp;  // 摄像头图像时间戳
  uint64_t SPI_time_stamp;    // 地平线SPI时间戳
  uint32_t frame_index;       // 摄像头图像index
  J3_LANES j3_lanes;
  J3_OBJS j3_objs;
  J3_TRAFFIC_SIGNS j3_traffic_signs;
};

#endif  // __J3_COMMON_H__