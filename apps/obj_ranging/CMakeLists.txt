cmake_minimum_required(VERSION 3.10)
set(PROJ_NAME obj_ranging_app)
project(${PROJ_NAME})

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

set(INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include
                 ${PROJECT_SOURCE_DIR}/obj_ranging_core/include
                 ${PROJECT_SOURCE_DIR}/../..
                 ${EIGEN3_INCLUDE_DIR}
                 ${OpenCV_INCLUDE_DIRS}
                 )
include_directories(${INCLUDE_DIRS})

aux_source_directory(${PROJECT_SOURCE_DIR}/src OBJ_RANGING_SRC)
aux_source_directory(${PROJECT_SOURCE_DIR}/obj_ranging_core/src CORE_SRC)
aux_source_directory(${PROJECT_SOURCE_DIR}/../../common COMMON_SRC)
aux_source_directory(${PROJECT_SOURCE_DIR}/../../common/logger LOGGER_SRC)
aux_source_directory(${PROJECT_SOURCE_DIR}/../../common/show SHOW_SRC)
aux_source_directory(${PROJECT_SOURCE_DIR}/../../reader/obj_track OBJ_TRACK_SRC)
aux_source_directory(${PROJECT_SOURCE_DIR}/../../reader/lane_detect LANE_DETECT_SRC)
aux_source_directory(${PROJECT_SOURCE_DIR}/../../reader/lane_track LANE_TRACK_SRC)
aux_source_directory(${PROJECT_SOURCE_DIR}/../../reader/mobileye MOBILEYE_SRC)
set(SOURCES ${OBJ_RANGING_SRC}
            ${CORE_SRC}
            ${COMMON_SRC}
            ${LOGGER_SRC}
            ${SHOW_SRC}
            ${OBJ_TRACK_SRC}
            ${LANE_DETECT_SRC}
            ${LANE_TRACK_SRC}
            ${MOBILEYE_SRC}
            main.cc
)

set_source_files_properties(${CORE_SRC} PROPERTIES COMPILE_DEFINITIONS "MODULE_NAME=\"ranging\"")

set(LIBS ${OpenCV_LIBS}
         glog
         jsoncpp
)

add_executable(${PROJ_NAME} ${SOURCES})
target_link_libraries(${PROJ_NAME} ${LIBS})