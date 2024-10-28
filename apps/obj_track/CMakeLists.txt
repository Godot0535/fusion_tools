cmake_minimum_required(VERSION 3.10)
set(PROJ_NAME obj_track_app)
project(${PROJ_NAME})

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(OpenCV REQUIRED)

set(INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include
                 ${PROJECT_SOURCE_DIR}/obj_track_core/include
                 ${PROJECT_SOURCE_DIR}/../..
                 ${OpenCV_INCLUDE_DIRS}
                 )
include_directories(${INCLUDE_DIRS})

aux_source_directory(${PROJECT_SOURCE_DIR}/src OBJ_TRACK_SRC)
aux_source_directory(${PROJECT_SOURCE_DIR}/obj_track_core/src CORE_SRC)
aux_source_directory(${PROJECT_SOURCE_DIR}/../../common COMMON_SRC)
aux_source_directory(${PROJECT_SOURCE_DIR}/../../reader/obj_detect OBJ_DETECT_SRC)
aux_source_directory(${PROJECT_SOURCE_DIR}/../../reader/infrared_GT INFRARED_GT_SRC)
set(SOURCES ${OBJ_TRACK_SRC}
            ${CORE_SRC}
            ${COMMON_SRC}
            ${OBJ_DETECT_SRC}
            ${INFRARED_GT_SRC}
            main.cc
)

set(LIBS ${OpenCV_LIBS}
         glog
)

add_executable(${PROJ_NAME} ${SOURCES})
target_link_libraries(${PROJ_NAME} ${LIBS})

set(INFRARED ${OBJ_TRACK_SRC}
             ${CORE_SRC}
             ${COMMON_SRC}
             ${OBJ_DETECT_SRC}
             ${INFRARED_GT_SRC}
             infrared.cc
)

set(LIBS ${OpenCV_LIBS}
         glog
)

add_executable(infrared_track_app ${INFRARED})
target_link_libraries(infrared_track_app ${LIBS})