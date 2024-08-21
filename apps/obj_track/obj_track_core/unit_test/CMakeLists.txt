
add_executable(id_allocator_test id_allocator_test.cc
  ../src/id_allocator.cpp
)


target_include_directories(id_allocator_test PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/..
)