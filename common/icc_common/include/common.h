/**
 * @Version 1.3.0
 * @Date 2023.12.26
 * @Desciption 包含公共使用的多个结构体，具体说明如下:
 *      1. 数据公用定义
 *           数据来源
 *           数据帧头
 *           编码类型
 *           二维点定义
 *           三维点定义
 *      2. 多种视频数据
 *           佑驾原始视频流头定义
 *           摄像头获取的vx_image数据
 *           回灌工具的视频流数据
 *           磐石存储视频流结构体
 *      3. CAN数据定义
 *           标准CAN
 *           FD_CAN
 *      4. 车道线数据
 *           检测车道线
 *           跟踪车道线
 *      5. 障碍物数据
 *           检测障碍物
 *           跟踪障碍物
 *      6. 同步数据
 *      7. 后融合数据
 *      8. 其他数据
 *           PITCH定义
 *           CLIP_AND_PAD
 *           X6000_CHASSIS_INFO
 *           VEHICLE_KINETIC_INFO
 *
 **/

#ifndef _INCLUDE_COMMON_H_
#define _INCLUDE_COMMON_H_

#ifdef __aarch64__
#include <VX/vx.h>
#include <VX/vxu.h>
#endif

#include <dirent.h>
#include <linux/can.h>
#include <sys/stat.h>

#include <cstdint>
#include <iostream>

#define OBJ_TIME_LOG (0)
#define LANE_TIME_LOG (0)
#define OBJ_WRITE_TXT (0)
#define OBJ_WRITE_IMG (0)
#define LANE_WRITE_TXT (0)
#define LANE_WRITE_IMG (0)
#define ICC_WRITE_IMG (0)
#define ICC_WRITE_TXT (0)
#define CAMERA_WRITE_TXT (0)

#define NV12_HEIGHT (1080 * 3 / 2)
#define NV12_WIDTH (1920 * 2)
#define CAMERA_WIDTH (1920 * 2)
#define CAMERA_HEIGHT (1080)
#define NUM_CHANNELS (3)

#define MAX_PATH_LEN (260)
#define MAX_STR_LEN (8)
#define MAX_CAN_LEN (8)
#define MAX_FRAME_LEN (1 * 1024 * 1024)
#define MAGIC_NUMBER (0x01020304)

// 版本号
#define VERSION_NAME_MAX_SIZE 10 // 标签名字最大长度

// polygon最大点数
#define POLYGON_POINTS_MAX_NUM 24 // 标签名字最大长度

#define IMAGE_WIDTH 1920
#define IMAGE_HEIGHT 1080
#define NV12_SIZE (IMAGE_WIDTH * IMAGE_HEIGHT * 3 / 2)

// 1. 数据公用定义
/// @brief 数据来源
enum class DATA_SRC {
  FRONT_REAR_CAMERA,
  MOBILEYE,
  RADAR410,
  CHASSIS,
  IMU,
  DETECT_LANES,
  DETECT_OBJS,
  TRACK_LANES, // 不用存储
  TRACK_OBJS,  // 不用存储
  SYNC_RESULTS,
  FUSION_OBJS,
  FUSION_LANES,
  PERCEP_FUSION_RESULTS,
};

/// @brief 数据帧头
typedef struct _DATA_HEADER {
  uint32_t magic_num;           // 0x01020304 etc
  uint32_t version;             // 版本号
  uint64_t ads_time_stamp;      // 板上自增时间戳
  uint64_t utc_time_stamp_sec;  // UTC时间戳整数
  uint64_t utc_time_stamp_nsec; // UTC时间戳小数
  uint64_t frame_id;            // id
  DATA_SRC data_src;            // 数据源
} DATA_HEADER, *LP_DATA_HEADER;

/// @brief 编码类型
enum class ENCODE_TYPE {
  MJPEG,
  H264,
  H265,
  JPEG,
  BMP,
};

/// @brief 二维点定义
typedef struct _POINT2D {
  double x;
  double y;
} POINT2D, *LP_POINT2D;

/// @brief 三维点定义
typedef struct _POINT3D {
  double x;
  double y;
  double z;
} POINT3D, *LP_POINT3D;

// 2. 多种视频数据
/// @brief 佑驾原始视频流头定义
struct frame_head_data_s {
  int32_t Height;
  int32_t Width;
  int32_t SendTimeHigh;
  int32_t SendTimeLow;
  int32_t FrameType; // 0：MJPEG 1：H264 2：H265
  int32_t DataSize;
  uint32_t Seq;
  uint32_t Sec;
  uint32_t Nsec;
};
#ifdef __aarch64__

/// @brief 摄像头获取的vx_image数据
typedef struct _CAMERA_DATA {
  uint64_t frame_id;
  uint64_t ads_time_stamp; // 板上自增时间戳
  vx_image image;
} CAMERA_DATA, *LP_CAMERA_DATA;
#endif

/// @brief 回灌工具的视频流数据
typedef struct _MH264_DATA {
  char *data;
  int size;
} MH264_DATA, *LP_MH264_DATA;

/// @brief 磐石存储视频流结构体
typedef struct _H264_DATA {
  DATA_HEADER data_header;
  int32_t height; // 图像高度
  int32_t width;  // 图像宽度
  int32_t send_time_high;
  int32_t send_time_low;
  ENCODE_TYPE encode_type; // 编码类型
  int32_t data_size;       // 数据长度
  uint32_t seq;            // 序列号
  uint32_t sec;
  uint32_t nsec;
  char
      *p_data; // 码流数据，使用时申请空间，结束后释放。板上不建议使用shared_ptr
} H264_DATA, *LP_H264_DATA;

// 3. CAN数据定义
/// @brief 标准CAN
typedef struct _CAN_DATA {
  DATA_HEADER data_header;
  can_frame frame;
} CAN_DATA, *LP_CAN_DATA;

/// @brief FD_CAN
typedef struct _CAN_FD_DATA {
  DATA_HEADER data_header;
  canfd_frame frame;
} CAN_FD_DATA, *LP_CAN_FD_DATA;

/// @brief 版本信息定义
typedef struct _VERSION {
  char lanes_model_version[VERSION_NAME_MAX_SIZE];
  char objs_model_version[VERSION_NAME_MAX_SIZE];
  char detection_tda4_version[VERSION_NAME_MAX_SIZE];
  char recorder_server_version[VERSION_NAME_MAX_SIZE];
} VERSION, *LP_VERSION;

// 8. 其他数据

/// @brief 俯仰角
typedef struct _PITCH {
  DATA_HEADER data_header;
  float data;
} PITCH, *LP_PITCH;

/// @brief 添加裁减pading数据结构
typedef struct _CLIP_AND_PAD {
  int clip_w_left;
  int clip_w_right;
  int clip_h_top;
  int clip_h_bottom;
  int pad_w_left;
  int pad_w_right;
  int pad_h_top;
  int pad_h_bottom;
  float expand_scale;
} CLIP_AND_PAD, *LP_CLIP_AND_PAD;

/// @brief x6000底盘信息
typedef struct _X6000_CHASSIS_INFO {
  float relative_speed_fl_wheel;
  float relative_speed_fr_wheel;
  float relative_speed_r2l_wheel;
  float relative_speed_r1l_wheel;
  float relative_speed_r2r_wheel;
  float relative_speed_r1r_wheel;
  float vehicle_speed;
  float lateral_acceleration;
  float yaw_rate;
  float longitudinal_acceleration;
  float steering_angle;
  float steering_angle_speed;
  float break_pedal_position;
} X6000_CHASSIS_INFO, *LP_X6000_CHASSIS_INFO;

typedef struct _VEHICLE_KINETIC_INFO {
  uint64_t time_stamp;
  double angle_v;       // 角速度
  double v;             // 速度
  POINT3D acceleration; // x,y方向加速度
} VEHICLE_KINETIC_INFO, *LP_VEHICLE_KINETIC_INFO;

struct YUVInfoStruct {
  int32_t Height;
  int32_t Width;
  int32_t DataSize;
  uint64_t timestamp;
  uint64_t tick;
  uint64_t frame_id;
};

/// @brief 磐石存储YUV数据定义
typedef struct _YUV_DATA {
  DATA_HEADER data_header;
  YUVInfoStruct camera_info;
  char data[NV12_SIZE];
} YUV_DATA, *LP_YUV_DATA;

#endif // _INCLUDE_COMMON_H_
