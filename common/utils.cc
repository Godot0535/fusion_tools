#include "utils.h"

#include <dirent.h>
#include <string.h>

#include <algorithm>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>

void GetFileNames(const std::string &files_path,
                  std::vector<std::string> &filenames) {
  DIR *dir;
  struct dirent *ptr;
  if ((dir = opendir(files_path.c_str())) == NULL) {
    perror("Open dir error...");
    closedir(dir);
    return;
  }

  while ((ptr = readdir(dir)) != NULL) {
    if (strcmp(ptr->d_name, ".") == 0 || strcmp(ptr->d_name, "..") == 0)
      continue;
    else if (ptr->d_type == 8) {
      std::string strFile;
      strFile = files_path;
      strFile += "/";
      strFile += ptr->d_name;
      filenames.push_back(strFile);
    } else {
      continue;
    }
  }
  closedir(dir);

  std::sort(filenames.begin(), filenames.end());
}

void GetFiles(std::string path, std::vector<std::string> &filenames) {
  DIR *pDir;
  struct dirent *ptr;
  if (!(pDir = opendir(path.c_str()))) {
    std::cout << "Folder doesn't Exist!" << std::endl;
    closedir(pDir);
    return;
  }
  while ((ptr = readdir(pDir)) != 0) {
    if (strcmp(ptr->d_name, ".") != 0 && strcmp(ptr->d_name, "..") != 0) {
      filenames.push_back(path + "/" + ptr->d_name);
    }
  }
  closedir(pDir);
}

void GetTruthNames(std::string &truth_path,
                   std::vector<std::string> &truth_names) {
  DIR *dir;
  struct dirent *ptr;
  if ((dir = opendir(truth_path.c_str())) == NULL) {
    perror("Open dir error...");
    closedir(dir);
    return;
  }

  while ((ptr = readdir(dir)) != NULL) {
    if (strcmp(ptr->d_name, ".") == 0 || strcmp(ptr->d_name, "..") == 0)
      continue;
    else if (ptr->d_type == 4) {
      std::string strFile;
      strFile = truth_path;
      strFile += "/";
      strFile += ptr->d_name;
      std::vector<std::string> truth_file;
      GetFiles(strFile + "/det_label", truth_file);
      if (truth_file.size() == 0) {
        truth_names.push_back("-");
      } else {
        truth_names.push_back(truth_file[0]);
      }

    } else {
      continue;
    }
  }
  closedir(dir);

  std::sort(truth_names.begin(), truth_names.end());
}

void GetAllFolders(const std::string &path,
                   std::vector<std::string> &folder_names) {
  DIR *dir;
  dirent *entry;

  if ((dir = opendir(path.c_str())) == NULL) {
    perror(path.c_str());
    closedir(dir);
    return;
  }

  folder_names.clear();
  while ((entry = readdir(dir)) != NULL) {
    if (entry->d_type == DT_DIR && entry->d_name[0] != '.') {
      folder_names.push_back(path + '/' + entry->d_name);
    }
  }
  closedir(dir);

  std::sort(folder_names.begin(), folder_names.end());
}

std::string GetTruthDetect(std::string &path) {
  DIR *dir;
  dirent *entry;

  if ((dir = opendir(path.c_str())) == NULL) {
    perror(path.c_str());
    closedir(dir);
    return "-";
  }

  std::string det_path = "-";
  while ((entry = readdir(dir)) != NULL) {
    std::string file_path = path + '/' + entry->d_name;
    if (file_path.find("label_det_2d") != std::string::npos) {
      det_path = file_path;
      break;
    }
  }
  closedir(dir);
  return det_path;
}

std::string GetTruthLane(std::string &path) {
  DIR *dir;
  dirent *entry;

  if ((dir = opendir(path.c_str())) == NULL) {
    perror(path.c_str());
    closedir(dir);
    return "-";
  }

  std::string lane_path = "-";
  while ((entry = readdir(dir)) != NULL) {
    std::string file_path = path + '/' + entry->d_name;
    if (file_path.find("label_lane") != std::string::npos) {
      lane_path = file_path;
      break;
    }
  }
  closedir(dir);
  return lane_path;
}

std::string GetTruthImage(std::string &path) {
  DIR *dir;
  dirent *entry;

  if ((dir = opendir(path.c_str())) == NULL) {
    perror(path.c_str());
    closedir(dir);
    return "-";
  }

  std::string img_path = "-";
  while ((entry = readdir(dir)) != NULL) {
    std::string file_path = path + '/' + entry->d_name;
    std::filesystem::path fs_path(file_path);
    std::string extension = fs_path.extension().string();

    if (extension == ".png") {
      img_path = file_path;
      break;
    }
  }
  closedir(dir);
  return img_path;
}

std::string GetMobileye(std::string &path) {
  DIR *dir;
  dirent *entry;

  if ((dir = opendir(path.c_str())) == NULL) {
    perror(path.c_str());
    closedir(dir);
    return "-";
  }

  std::string det_path = "-";
  while ((entry = readdir(dir)) != NULL) {
    std::string file_path = path + '/' + entry->d_name;
    if (file_path.find("mobileye_X6000") != std::string::npos) {
      det_path = file_path;
      break;
    }
  }
  closedir(dir);
  return det_path;
}

std::string GetChassis(std::string &path) {
  DIR *dir;
  dirent *entry;

  if ((dir = opendir(path.c_str())) == NULL) {
    perror(path.c_str());
    closedir(dir);
    return "-";
  }

  std::string det_path = "-";
  while ((entry = readdir(dir)) != NULL) {
    std::string file_path = path + '/' + entry->d_name;
    if (file_path.find("chassis") != std::string::npos) {
      det_path = file_path;
      break;
    }
  }
  closedir(dir);
  return det_path;
}

std::string GetTxtWithStr(std::string &path, std::string &str) {
  DIR *dir;
  dirent *entry;

  if ((dir = opendir(path.c_str())) == NULL) {
    perror(path.c_str());
    closedir(dir);
    return "-";
  }

  std::string target_path = "-";
  while ((entry = readdir(dir)) != NULL) {
    std::string file_path = path + '/' + entry->d_name;
    if (file_path.find(str) != std::string::npos) {
      if (file_path.find(".txt") != std::string::npos) {
        target_path = file_path;
        break;
      }
    }
  }
  closedir(dir);
  return target_path;
}

std::string GetJsonWithStr(std::string &path, std::string &str) {
  DIR *dir;
  dirent *entry;

  if ((dir = opendir(path.c_str())) == NULL) {
    perror(path.c_str());
    closedir(dir);
    return "-";
  }

  std::string target_path = "-";
  while ((entry = readdir(dir)) != NULL) {
    std::string file_path = path + '/' + entry->d_name;
    if (file_path.find(str) != std::string::npos) {
      if (file_path.find(".json") != std::string::npos) {
        target_path = file_path;
        break;
      }
    }
  }
  closedir(dir);
  return target_path;
}

std::string GetRadar(std::string &path) {
  DIR *dir;
  dirent *entry;

  if ((dir = opendir(path.c_str())) == NULL) {
    perror(path.c_str());
    closedir(dir);
    return "-";
  }

  std::string radar_path = "-";
  while ((entry = readdir(dir)) != NULL) {
    std::string file_path = path + '/' + entry->d_name;
    if (file_path.find("radar_410") != std::string::npos) {
      radar_path = file_path;
      break;
    }
  }
  closedir(dir);
  return radar_path;
}

// eg: int 543 -> string 000543
std::string IndexString(int index) {
  std::ostringstream oss;
  oss << std::setfill('0') << std::setw(6) << index;
  return oss.str();
}

std::vector<std::string> ReadFolder(const std::string &image_path,
                                    bool filename_compare_flag) {
  std::vector<std::string> image_names;
  auto dir = opendir(image_path.c_str());

  if ((dir) != NULL) {
    struct dirent *entry;
    entry = readdir(dir);
    while (entry) {
      auto temp = entry->d_name;
      if (strcmp(entry->d_name, "") == 0 || strcmp(entry->d_name, ".") == 0 ||
          strcmp(entry->d_name, "..") == 0) {
        entry = readdir(dir);
        continue;
      }
      image_names.push_back(temp);
      entry = readdir(dir);
    }
  }
  closedir(dir);
  if (filename_compare_flag) {
    std::sort(image_names.begin(), image_names.end(), filename_compare());
  } else {
    std::sort(image_names.begin(), image_names.end());
  }

  return image_names;
}

// Eigen::VectorXd JsonToEigenVector(const Json::Value &jsonArray) {
//   Eigen::VectorXd line(4);
//   if (jsonArray.size() == 2) {
//     line << jsonArray[0].asDouble(), jsonArray[1].asDouble(), 0, 0;
//   } else if (jsonArray.size() == 4) {
//     line << jsonArray[0].asDouble(), jsonArray[1].asDouble(),
//         jsonArray[2].asDouble(), jsonArray[3].asDouble();
//   } else {
//     // Handle error or skip invalid points
//     std::cerr << "Invalid point format in the array." << std::endl;
//   }
//   return line;
// }

std::string Replace(const std::string &str, const std::string &old_str,
                    const std::string &new_str) {
  std::string result = str;
  size_t pos = 0;
  while ((pos = result.find(old_str, pos)) != std::string::npos) {
    result.replace(pos, old_str.length(), new_str);
    pos += new_str.length();
  }
  return result;
}

/*
 * @brief 从字符串中提取出第一个超过8位的连续数字字段
 */
uint64_t GetTimestamp(std::string &str) {
  std::string sub;
  uint64_t timestamp = 0;
  for (int i = 0; i < str.length(); i++) {
    char tmp = str[i];
    if (tmp == '_' || tmp == '/') {  // summary
      if (sub.length() > 8) {
        timestamp = std::stoll(sub);
        return timestamp;
      } else {
        sub.clear();
        continue;
      }
    }
    if (tmp >= '0' && tmp <= '9') {
      sub.push_back(tmp);
    }
  }
  return timestamp;
}

// void Reflection(const base::POINT_3D &point3d, base::POINT_2D &point2d) {
//   std::vector<cv::Point3f> objectPoints;
//   objectPoints.emplace_back(point3d.x, point3d.y, point3d.z);  // 3D点

//   // 相机内参矩阵
//   cv::Mat cameraMatrix =
//       (cv::Mat_<double>(3, 3) << 1143.398167, 0.0, 956.0743379, 0.0,
//        1150.813096, 532.9676489, 0.0, 0., 1.0);
//   // 畸变参数 k1,k2,p1,p2,k3
//   cv::Mat distCoeffs = (cv::Mat_<double>(5, 1) << -0.48619105, 0.49547514,
//                         -0.01511549, -0.01506603, 0);

//   cv::Mat rotationMatrix =
//       (cv::Mat_<double>(3, 3) << 0.0261690, -0.9993552, -0.0245839,
//       0.1366833,
//        0.0279384, -0.9902207, 0.9902691, 0.0225529, 0.1373263);

//   cv::Mat rvec;  // 旋转向量
//   cv::Rodrigues(rotationMatrix, rvec);

//   cv::Mat tvec =
//       (cv::Mat_<double>(3, 1) << 0.230022, 1.05782, -0.0453504);  // 平移向量

//  场景切片真值 畸变参数 k1,k2,p1,p2,k3
//  cv::Mat distCoeffs = (cv::Mat_<double>(5, 1) <<
//  -0.356918618,0.167589043,-0.000186537,-0.00026963,0 );
//
//  cv::Mat rotationMatrix = (cv::Mat_<double>(3, 3) <<0.0261581,  -0.999585
//  ,-0.0120472 ,
//      0.127114,  0.0152795 ,  -0.99177 ,
//      0.991543,  0.0244115,    0.12746);
//
//
//
//  cv::Mat rvec;  // 旋转向量
//  cv::Rodrigues(rotationMatrix, rvec);
//
//  cv::Mat tvec = (cv::Mat_<double>(3, 1) << 0.299984, 2.26667, 0.232795); //
//  平移向量

// // 2D点的输出矩阵
// std::vector<cv::Point2f> imagePoints;

// // 使用projectPoints方法将3D点投影到2D图像上
// cv::projectPoints(objectPoints, rvec, tvec, cameraMatrix, distCoeffs,
//                   imagePoints);

// // 打印2D点的坐标
// cv::Point2f pt = imagePoints[0];
// point2d.x = pt.x;
// point2d.y = pt.y;
// }

Eigen::VectorXd Polyfit(const Eigen::VectorXd &x, const Eigen::VectorXd &y,
                        const int degree) {
  int num_points = x.size();

  Eigen::MatrixXd A(num_points, degree + 1);
  for (int i = 0; i < num_points; ++i) {
    for (int j = 0; j < degree + 1; ++j) {
      A(i, j) = std::pow(x(i), degree - j);
    }
  }

  Eigen::VectorXd coefficients = A.householderQr().solve(y);

  return coefficients;
}

Eigen::VectorXd linspace(double start, double end, const int num_points) {
  Eigen::VectorXd ret(num_points);

  for (int i = 0; i < num_points; ++i) {
    ret(i) = start + i * (end - start) / (num_points - 1.0);
  }

  return ret;
}

void waitForChar(char targetChar) {
  char inputChar;
  do {
    std::cin.get(inputChar);
  } while (inputChar != targetChar);
}