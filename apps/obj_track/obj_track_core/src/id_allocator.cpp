#include "../include/id_allocator.h"

IdAllocator::IdAllocator(int size, double del_thresh) {
  del_thresh_ = del_thresh;
  for (int i = 0; i < size; ++i) {
    pool_.push(i);
  }
}

int IdAllocator::GetNewId() {
  DelInvalidId();
  if (pool_.empty()) {
    return -1;  // No available IDs
  }

  // Get an ID from pool_
  int id = pool_.front();  // Get the front available ID
  pool_.pop();             // Remove it from pool_
  return id;
}

bool IdAllocator::InvalidateId(int id) {
  released_ids[id] =
      std::chrono::steady_clock::now();  // Record the release time of ID
  return true;
}

void IdAllocator::DelInvalidId() {
  // Check for released IDs
  auto current_time = std::chrono::steady_clock::now();
  std::vector<int> ids_to_remove;
  for (auto& pair : released_ids) {
    auto time_diff = std::chrono::duration_cast<std::chrono::seconds>(
                        current_time - pair.second)
                        .count();
    if (time_diff >= del_thresh_) {  // Example: IDs released after 5 seconds
      pool_.push(pair.first);       // Return the released ID to pool_
      ids_to_remove.push_back(
          pair.first);  // Record the ID to remove from released_ids
    }
  }
  for (auto id : ids_to_remove) {
    released_ids.erase(id);
  }
}
