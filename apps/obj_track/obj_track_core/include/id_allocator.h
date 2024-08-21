#pragma once
#include <iostream>
#include <queue>
#include <unordered_map>
#include <chrono>


class IdAllocator {
  double del_thresh_{1.0};
  std::queue<int> pool_;
  std::unordered_map<int, std::chrono::steady_clock::time_point> released_ids;

  void DelInvalidId();

 public:
  IdAllocator(int size, double del_thresh);
  ~IdAllocator() = default;

  int GetNewId();
  bool InvalidateId(int id);


};