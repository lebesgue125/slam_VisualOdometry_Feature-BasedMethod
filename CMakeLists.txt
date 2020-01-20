cmake_minimum_required(VERSION 3.15)
project(VO1)

set(CMAKE_CXX_STANDARD 14)

find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(Sophus REQUIRED)
find_package(OpenCV REQUIRED)

message(status "Pangolin_INCLUDE_DIRS-------" ${Pangolin_INCLUDE_DIRS})
message(status "EIGEN3_INCLUDE_DIR--------" ${EIGEN3_INCLUDE_DIR})
message(status "Sophus_INCLUDE_DIRS-------" ${Sophus_INCLUDE_DIRS})

include_directories(
        ${EIGEN3_INCLUDE_DIR}
        ${Pangolin_INCLUDE_DIRS}
        ${Sophus_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
)


set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR})

add_executable(orb computeORB.cpp)
target_link_libraries(orb
        #${EIGEN3_LIBS}
        #${Pangolin_LIBRARIES}
        # ${Sophus_LIBRARIES}
        ${OpenCV_LIBS}
        )


add_executable(e2rt E2Rt.cpp)
target_link_libraries(e2rt
        ${EIGEN3_LIBS}
        #${Pangolin_LIBRARIES}
        # ${Sophus_LIBRARIES}
        #${OpenCV_LIBS}
        )

add_executable(gnba GN-BA.cpp)
target_link_libraries(gnba
        ${EIGEN3_LIBS}
        #${Pangolin_LIBRARIES}
        # ${Sophus_LIBRARIES}
        #${OpenCV_LIBS}
        )

add_executable(trajICP trajectoryICP.cpp)
target_link_libraries(trajICP
        ${EIGEN3_LIBS}
        ${Pangolin_LIBRARIES}
        # ${Sophus_LIBRARIES}
        ${OpenCV_LIBS}
        )