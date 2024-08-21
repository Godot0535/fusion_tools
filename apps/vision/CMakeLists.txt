cmake_minimum_required(VERSION 3.10)
set(PROJ_NAME vision_app)
project(${PROJ_NAME})

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

set(INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include
                 ${PROJECT_SOURCE_DIR}/../..
                 ${OpenCV_INCLUDE_DIRS}
                 )
include_directories(${INCLUDE_DIRS})

aux_source_directory(src VISION_SRC)
aux_source_directory(${PROJECT_SOURCE_DIR}/../../common COMMON_SRC)
aux_source_directory(${PROJECT_SOURCE_DIR}/../../common/show SHOW_SRC)
aux_source_directory(${PROJECT_SOURCE_DIR}/../../reader/obj_detect OBJ_DETECT_SRC)
aux_source_directory(${PROJECT_SOURCE_DIR}/../../reader/obj_track OBJ_TRACK_SRC)
aux_source_directory(${PROJECT_SOURCE_DIR}/../../reader/lane_detect LANE_DETECT_SRC)
aux_source_directory(${PROJECT_SOURCE_DIR}/../../reader/lane_track LANE_TRACK_SRC)
aux_source_directory(${PROJECT_SOURCE_DIR}/../../reader/obj_ranging OBJ_RANGING_SRC)
aux_source_directory(${PROJECT_SOURCE_DIR}/../../reader/mobileye MOBILEYE_SRC)
set(SOURCES ${VISION_SRC}
            ${COMMON_SRC}
            ${SHOW_SRC}
            ${OBJ_DETECT_SRC}
            ${OBJ_TRACK_SRC}
            ${LANE_DETECT_SRC}
            ${LANE_TRACK_SRC}
            ${OBJ_RANGING_SRC}
            ${MOBILEYE_SRC}
            main.cc
)

set(LIBS ${OpenCV_LIBS}
         glog
         jsoncpp
         )

add_executable(${PROJ_NAME} ${SOURCES})
target_link_libraries(${PROJ_NAME} ${LIBS})

set(SAVE_SOURCES ${VISION_SRC}
                 ${COMMON_SRC}
                 ${SHOW_SRC}
                 ${OBJ_DETECT_SRC}
                 ${OBJ_TRACK_SRC}
                 ${LANE_DETECT_SRC}
                 ${LANE_TRACK_SRC}
                 ${OBJ_RANGING_SRC}
                 ${MOBILEYE_SRC}
                 save.cc
)

add_executable(save_app ${SAVE_SOURCES})
target_link_libraries(save_app ${LIBS})