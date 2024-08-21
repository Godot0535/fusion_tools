//
// Created by lee on 24-5-31.
//
#include "include/id_allocator.h"
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>

TEST_CASE("IdAllocator") {
  IdAllocator pool(3, 1.0); // Initialize pool with 10 IDs

  // Test get and release
  int id1 = pool.GetNewId();
  int id2 = pool.GetNewId();
  int id3 = pool.GetNewId();

  CHECK_EQ(id1, 0);
  CHECK_EQ(id2, 1);
  CHECK_EQ(id3, 2);

  pool.InvalidateId(id2);

  int id4 = pool.GetNewId();
  CHECK_EQ(id4, -1);
  sleep(1);
  int id5 = pool.GetNewId();
  CHECK_EQ(id5, 1);
}



