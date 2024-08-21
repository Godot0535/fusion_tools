//
// Created by lee on 24-6-13.
//

#pragma once
#include "common.h"
#include <stdint.h>

// 障碍物检测相关
#define OBJ_NAME_MAX_SIZE 16 // 标签名字最大长度
#define OBJS_MAX_NUM 100     // 单张图片最大目标数量
#define OBJ_CLASS_MAX_NUM 30 // 最大障碍物类别数

struct RADAR_STATUS {
  bool Reserved1;                        // 预留
  bool Communication_Bus_Off;            // 通信总线关闭
  bool Lost_Communication_With_Camera;   // 与相机失去通信
  bool MCU_Temperature_too_high;         // MCU温度过高
  bool MCU_Temperature_too_low;          // MCU温度过低
  bool MCU_Voltage_too_high;             // MCU电压过高
  bool MCU_Voltage_too_low;              // MCU电压过低
  bool RF_Voltage1_too_high;             // 射频电压过高
  bool RF_Voltage1_too_low;              // 射频电压过低
  bool Reserved2;                        // 预留
  bool Reserved3;                        // 预留
  bool Battery_Voltage_too_high;         // 电池电压过高
  bool Battery_Voltage_too_low;          // 电池电压过低
  bool RF_Temperature_too_high;          // 射频温度过高
  bool Reserved4;                        // 预留
  bool MCU_Master_Clock_Error;           // MCU主时钟故障
  bool MCU_Error;                        // MCU故障
  bool MCU_Memory_Error;                 // MCU内存故障
  bool Reserved5;                        // 预留
  bool RF_Sensor_fault;                  // 射频传感器故障
  bool Missing_Calibration;              // 标定缺失
  bool Invalid_Vehicle_Speed_signal;     // 车速信号无效
  bool Invalid_Yaw_Rate_signal;          // 横摆角速度信号无效
  bool Invalid_Steering_Angle_signal;    // 方向盘转角信号无效
  bool Reserved6;                        // 预留
  bool Reserved7;                        // 预留
  bool Radar_block;                      // 雷达模块阻塞
  bool Radar_Mount_Position_Shift_Error; // 雷达安装位置偏移错误
  bool Reserved8;                        // 预留
};

/// @brief 静态障碍物类型
enum class STATIC_OBJ_TYPE {
  UNKNOWN,                                // 未知
  CONE,                                   // 锥桶
  POLE,                                   // 杆
  REFLECTOR,                              // 反光镜
  SIGN,                                   // 标志
  BARREL,                                 // 桶
  TRASHCAN,                               // 垃圾桶
  STROLLER,                               // 婴儿推车
  WHEELCHAIR,                             // 轮椅
  SUPERMARKET_CART,                       // 购物车
  BOX,                                    // 箱子
  SUITCASE,                               // 行李箱
  WATERSAFETY_BARRIER,                    // 水马
  SAFETY_CRASH_BUCKET,                    // 防撞桶
  TRAFFIC_DUMMIES,                        // 交通假人
  SIMULATIONAL_POLICE_CAR_WARNING_DEVICE, // 仿真警车警示装置
  GUARDRAIL,                              // 护栏
  BARRICADE,                              // 障碍栏
  JERSEY_BARRIER,                         // 新泽西护栏
  RESERVED1,                              // 预留1
  RESERVED2,                              // 预留2
};

/// @brief 静态障碍物跟踪状态信息
enum class STATIC_OBJ_TRCK_STS {
  INVALID,     // 无效
  NOT_TRACKED, // 未跟踪
  INITIALIZED, // 已初始化
  TRACKED,     // 跟踪的
  PREDICTED,   // 预测的
};

/// @brief 静态障碍物
struct STATIC_OBJ {
  uint8_t Checksum;                // 校验和
  uint8_t Counter;                 // 计数器
  uint8_t Id;                      // Track id
  float DetectionConfidence;       // 检测置信度
  uint8_t DetectionHistory;        // 检测历史
  float Width;                     // 目标宽度
  float Height;                    // 目标高度
  float LatPos;                    // 横向距离
  float LongPos;                   // 纵向距离
  STATIC_OBJ_TYPE Type;            // 目标类型
  float StdDevLatPos;              // 横向距离标准方差
  float StdDevLongPos;             // 纵向距离标准方差
  uint32_t TimeStamp;              // 时间戳
  STATIC_OBJ_TRCK_STS TrackStatus; // 跟踪状态信息
  float VertPos;                   // 垂直位置
  bool isVerified;
};

/// @brief 前向4D雷达状态
struct FRONT_FOUR_D_RADAR_STS {
  uint8_t Object_NofObjects;       // 轨迹个数
  RADAR_STATUS Radar_Status;       // 雷达状态
  uint8_t Object_InterfaceVersion; // 接口版本
};

/// @brief 目标估计
struct OBJ_ESTIMATION_GROUP {
  float PosnLgt; // 纵向位置
  float PosnLat; // 横向位置
  float Spd;     // 速度
  float VLgt;    // 纵向速度
  float VLat;    // 横向速度
  float A;       // 加速度
  float ALgt;    // 纵向加速度
  float ALat;    // 横向加速度
  float AgDir;   // 航向角
  float Crvt;    // 历史轨迹曲率
};

/// @brief 目标类型
enum class OBJ_TYPE {
  UNKNOWN,    // 未知
  CAR,        // 汽车
  MOTORCYCLE, // 摩托车
  TRUCK,      // 卡车
  PEDESTRAIN, // 行人
  TRICYCLE,   // 三轮车
  ANIM,       // 动物
  BICYCLE,    // 自行车
};

/// @brief 转向灯状态
enum class INDCR_TURN {
  INDCR_TURN_NOINDCN, // 未打转向灯
  INDCR_TURN_LE,      // 打左转向灯
  INDCR_TURN_RI,      // 打右转向灯
  INDCR_TURN_UKWN,    // 未知
};

/// @brief 灯状态
enum class LI_STS {
  LI_STS_UKWN, // 未知
  LI_STS_OFF,  // 灯关闭
  LI_STS_ON,   // 灯开启
};

/// @brief 目标运动模式
enum class OBJ_MTN_PAT {
  OBJ_MTN_PAT_UKWN,
  OBJ_MTN_PAT_STATY,          // 静止
  OBJ_MTN_PAT_MOVG_FROM_SELF, // 远离
  OBJ_MTN_PAT_MOVG_TO_SELF,   // 靠近
};

/// @brief 目标历史运动模式
enum class OBJ_MTN_PAT_HIST {
  OBJ_MTN_PAT_HIST_UKWN,
  OBJ_MTN_PAT_HIST_NOT_MOVG,             // 之前是静止
  OBJ_MTN_PAT_HIST_PREV_MOVMT_FROM_SELF, // 之前是远离
  OBJ_MTN_PAT_HIST_PREV_MOVMT_TO_SELF,   // 之前是靠近
};

/// @brief 目标信息
struct OBJ_INFO_GROUP {
  OBJ_TYPE Type;
  float Width;
  float Length;
  float Height;
  INDCR_TURN IndcrTurn;          // 转向灯
  LI_STS IndcrBrkLi;             // 制动灯
  LI_STS IndcrHzrdLi;            // 双闪
  OBJ_MTN_PAT MtnPat;            // 运动模式
  OBJ_MTN_PAT_HIST MtnPatHist;   // 历史运动模式
  float DstLatFromMidOfLaneSelf; // 距离车道中心的横向距离
};

/// @brief 数据状态
enum class SNSR_DATA_STS {
  SNSR_DATA_STS_INVLD,    // 无效的数据
  SNSR_DATA_STS_FUSN,     // 融合的数据
  SNSR_DATA_STS_NEW,      // 新数据
  SNSR_DATA_STS_PRED_NEW, // 预测的新数据
  SNSR_DATA_STS_UPD_NEW,  // 更新的新数据
  SNSR_DATA_STS_UPD,      // 更新的数据
  SNSR_DATA_STS_PRED,     // 预测的数据
};

enum class OBJ_PPTY_MDL_OF_MTN0 {
  OBJ_PPTY_MDL_OF_MTN0_MDL_ACON,
  OBJ_PPTY_MDL_OF_MTN0_MDL_BICYCLE,
};

enum class OBJ_PPTY_TRFC_SCENO0 {
  OBJ_PPTY_TRFC_SCENO0_NONE,
  OBJ_PPTY_TRFC_SCENO0_TURN_ACRSS_PAH,
};

enum class RELBL4 {
  RELBL4_NOT_RELBL, // 不可靠
  RELBL4_COAST_RELBL,
  RELBL4_BRK_SPPRT_RELBL,
  RELBL4_BRKG_RELBL,
};

enum class RELBL1 {
  RELBL1_NOT_RELBL, // 不可靠
  RELBL1_RELBL,     // 可靠
};

enum class OBJ_PPTY1_TRFC_JAM_ASSI_QLY0 {
  OBJ_PPTY1_TRFC_JAM_ASSI_QLY0_NOT_RELBL,  // 不可靠
  OBJ_PPTY1_TRFC_JAM_ASSI_QLY0_RELBL,      // 可靠
  OBJ_PPTY1_TRFC_JAM_ASSI_QLY0_HIGH_RELBL, // 高可靠性
};

enum class RELBL3 {
  RELBL3_NOT_RELBL,      // 不可靠
  RELBL3_SOON_NOT_RELBL, // 很快不可靠
  RELBL3_LOW_RELBL,      // 低可靠性
  RELBL3_HIGHER_RELBL,   // 更高可靠性
  RELBL3_HIGHEST_RELBL,  // 最高可靠性
};

/// @brief 目标属性
struct OBJ_PPTY_GROUP {
  uint8_t Idn;                   // Track ID
  uint8_t VisnId;                // 视觉ID
  SNSR_DATA_STS Sts;             // 数据状态
  OBJ_PPTY_MDL_OF_MTN0 MdlOfMtn; // 移动的目标物是个点还是自行车模型
  OBJ_PPTY_TRFC_SCENO0 TrfcSceno;                 // 穿越自己当前路径
  RELBL4 CllsnMtgtnByBrkgPrimQly;                 // AEB主目标的质量
  RELBL1 CllsnMtgtnByBrkgSecQly;                  // AEB第二目标的质量
  RELBL1 CllsnWarnFwdQly;                         // FCW目标的质量
  OBJ_PPTY1_TRFC_JAM_ASSI_QLY0 ObjTrfcJamAssiQly; // TJA目标的质量A
  RELBL3
  DstLatFromMidOfLaneSelfQly; // 目标物距离自车当前行驶车道线中心线的距离的质量
  float PosnLgtStdDe; // 纵向位置标准差
  float PosnLatStdDe; // 横向位置标准差
  float AgDirStdDe;   // 航向角标准差
  float SpdStdDe;     // 速度标准差
  float AStdDe;       // 加速度标准差
  uint8_t FusnSrc;    // 融合源
  uint32_t TimeStamp; // 时间
  uint32_t Age;       // 存在时间
  float ExistConf;    // 存在置信度
  float ClassConf;    // 类型置信度
};

struct OBJ_GROUP {
  OBJ_ESTIMATION_GROUP ObjEstimation; // 目标估计
  OBJ_INFO_GROUP ObjInfo;             // 目标信息
  OBJ_PPTY_GROUP ObjPpty;             // 目标属性
};

struct WUKONG_OBJS {
  uint8_t quantity;          // 有效目标数量
  OBJ_GROUP fusion_objs[32]; // 融合目标
};

enum BOOL {
  FALSE,
  TRUE,
};
// BOOL FrntRdrObjE2Efail;
// BOOL FrntRdrObjTO;

/// @brief 目标类别定义
enum class OBJ_CATEGORY {
  // 自研
  CAR = 0,
  TRUCK = 1,
  MOTORCYCLE = 2,
  BICYCLE = 3,
  TRICYCLE = 4,
  BUS = 5,
  PEDESTRAIN = 6,
  DYNAMIC_OTHERS = 7,
  CONE_DRUM = 100,
  STATIC_CYCLE = 101,
  INDICATOR_SIGN = 102,
  STATIC_OTHERS = 103,
  // radar410
  TWO_WHEEL_VEHILCE = 1000,  // 两轮车
  FOUR_WHEEL_VEHILCE = 1001, // 四轮车
  PERSON = 1002,             // 行人
  UNKNOW = 1999,
};

enum class MotionPattern {
  UNKNOWN = 0, // 未知
  STATIONARY,  // 静止
  STOPPED,     // 停止（运动后）
  MOVING,      // 运动
  CROSSING,
  RESERVED,
};

enum class TrackStatus {
  INVALID = 0, // 无效的
  NEW,         // 新轨迹
  MEASURED,    // 当前有测量关联
  COASTED,     // 当前无测量关联
};

/// @brief 2D_BOX定义
typedef struct _BOX_RECT {
  uint16_t left_top_u;
  uint16_t left_top_v;
  uint16_t right_bottom_u;
  uint16_t right_bottom_v;
} BOX_RECT, *LP_BOX_RECT;

/// @brief 3D检测框定义
typedef struct _BOX_3D_RECT {
  int16_t x;       // 车辆坐标系下目标中心点x坐标
  int16_t y;       // 车辆坐标系下目标中心点y坐标
  int16_t z;       // 车辆坐标系下目标中心点z坐标
  uint16_t width;  // 车辆坐标系下目标宽度
  uint16_t height; // 车辆坐标系下目标高度
  uint16_t length; // 车辆坐标系下目标长度
  uint16_t yaw;    // 车辆坐标系下目标航向角
  bool truncated; // 车辆坐标系下目标阶段属性(0,1)，0为未截断，1为截断
  bool occluded; // 车辆坐标系下目标遮挡属性(0,1)，0为未遮挡，1为遮挡
  uint16_t left_top_u; // 像素坐标系下目标2D检测框左上角x坐标
  uint16_t left_top_v; // 像素坐标系下目标2D检测框左上角y坐标
  uint16_t right_bottom_u; // 像素坐标系下目标2D检测框右下角x坐标
  uint16_t right_bottom_v; // 像素坐标系下目标2D检测框右下角y坐标
  // 标志位，判断2D属性是否预测、3D属性是否预测
  // 以及是否都预测(0,1,2)0表示都预测，1表示3D未预测,2表示2D未预测
  int16_t flag;
} BOX_3D_RECT, *LP_BOX_3D_RECT;

/// @brief 检测障碍物，单个和多个
typedef struct _DETECT_OBJ_INFO {
  char name[OBJ_NAME_MAX_SIZE];
  BOX_RECT box;
  float confidence;
  OBJ_CATEGORY obj_category;
  BOX_3D_RECT box_3d_rect;
  // Add vehicle front and rear support
  BOX_RECT sub_box;
  float sub_box_confidence;
} DETECT_OBJ_INFO, *LP_DETECT_OBJ_INFO;

typedef struct _DETECT_OBJS {
  DATA_HEADER data_header;
  int objs_num;
  DETECT_OBJ_INFO detect_objs[OBJS_MAX_NUM];
} DETECT_OBJS, *LP_DETECT_OBJS;

/// @brief 跟踪障碍物，单个和多个
typedef struct _TRACK_OBJ_INFO {
  int track_id;
  DETECT_OBJ_INFO obj_info; // 目标检测，已和刘一鸣沟通，先保留
  BOX_RECT box;             // track的像素坐标
  int frame_lost;
  bool activate;                 // 是否被测距模块激活
  float confidence;              // 置信度
  float lateral_distance;        // 横向距离
  float long_distance;           // 纵向距离
  float lateral_velocity;        // 横向速度
  float long_velocity;           // 纵向速度
  float distance_uncertainty[2]; // 距离误差概率 0-纵向
  float velocity_uncertainty[2]; // 速度误差概率 0-纵向
  float width;                   // 宽度
  float height;                  // 高度
} TRACK_OBJ_INFO, *LP_TRACK_OBJ_INFO;

typedef struct _TRACK_OBJS {
  DATA_HEADER data_header;
  int objs_num;
  TRACK_OBJ_INFO track_objs[OBJS_MAX_NUM];
} TRACK_OBJS, *LP_TRACK_OBJS;
