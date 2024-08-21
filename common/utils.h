#pragma once
#include <glog/logging.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <string>
#include <vector>

// 读取指定路径下的所有文件
void GetFileNames(const std::string &files_path,
                  std::vector<std::string> &filenames);

// 读取指定路径下的所有文件
void GetFiles(std::string path, std::vector<std::string> &filenames);

// 读取指定路径下的所有检测真值文件
void GetTruthNames(std::string &truth_path,
                   std::vector<std::string> &truth_names);

// 读取指定路径下的所有非隐藏文件夹名称并排序
void GetAllFolders(const std::string &path,
                   std::vector<std::string> &folder_names);

// 读取指定路径下的目标检测真值文件(不存在返回"-")
std::string GetTruthDetect(std::string &path);

// 读取指定路径下的车道线真值文件(不存在返回"-")
std::string GetTruthLane(std::string &path);

// 读取指定真值路径下的图片文件(不存在返回"-")
std::string GetTruthImage(std::string &path);

// 读取指定真值路径下的mobileye文件(不存在返回"-")
std::string GetMobileye(std::string &path);

// 读取指定真值路径下的radar文件(不存在返回"-")
std::string GetRadar(std::string &path);

// 读取指定路径下的chassis文件(不存在返回"-")
std::string GetChassis(std::string &path);

// 读取指定路径下的含有指定字符串的文件路径(不存在返回"-")
std::string GetTxtWithStr(std::string &path, std::string &str);
std::string GetJsonWithStr(std::string &path, std::string &str);

// 将帧号转为字符串
std::string IndexString(int index);

// std::vector<std::string> ReadFolder(const std::string &,
//                                     bool filename_compare_flag = false);

// Eigen::VectorXd JsonToEigenVector(const Json::Value &);

// std::string Replace(const std::string &, const std::string &,
//                     const std::string &);

// // 获取一个字符串里的时间戳
// uint64_t GetTimestamp(std::string &);

// void Reflection(const base::POINT_3D &, base::POINT_2D &);

// Eigen::VectorXd Polyfit(const Eigen::VectorXd &, const Eigen::VectorXd &,
//                         const int);

Eigen::VectorXd linspace(double, double, const int);

// struct folder_sort {
//   std::string Extra(const std::string &filename) {
//     // 找到最后一个下划线的位置
//     size_t underscore_pos = filename.rfind('/');

//     CHECK(underscore_pos != std::string::npos);

//     std::string result = filename.substr(underscore_pos + 1);

//     return result;
//   }
//   bool operator()(const std::string &s1, const std::string &s2) {
//     int i1 = std::stoi(Extra(s1));
//     int i2 = std::stoi(Extra(s2));
//     return i1 < i2;
//   }
// };

// struct detect_filename_compare {
//   std::string Extra(const std::string &filename) {
//     // 找到最后一个下划线的位置
//     size_t underscore_pos = filename.rfind('_');
//     // 找到 ".txt" 的位置
//     size_t txt_pos = filename.find(".txt");

//     CHECK(underscore_pos != std::string::npos && txt_pos !=
//     std::string::npos);

//     std::string result =
//         filename.substr(underscore_pos + 1, txt_pos - underscore_pos - 1);

//     return result;
//   }

//   bool operator()(const std::string &s1, const std::string &s2) {
//     std::string order1 = Extra(s1);
//     std::string order2 = Extra(s2);
//     return order1 < order2;
//   }
// };

struct filename_compare {
  std::string Extra(const std::string &filename) {
    // 找到最后一个下划线的位置
    size_t underscore_pos = filename.rfind('_');
    // 找到 "." 的位置
    size_t txt_pos = filename.rfind('.');

    CHECK(underscore_pos != std::string::npos && txt_pos != std::string::npos);

    std::string result =
        filename.substr(underscore_pos + 1, txt_pos - underscore_pos - 1);

    return result;
  }

  bool operator()(const std::string &s1, const std::string &s2) {
    std::string order1 = Extra(s1);
    std::string order2 = Extra(s2);
    return order1 < order2;
  }
};

void waitForChar(char targetChar);