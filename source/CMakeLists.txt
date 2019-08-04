# Find packages
find_package(Sophus REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(state SHARED
    State.cpp
)

target_include_directories(state PUBLIC
    ${PROJECT_SOURCE_DIR}/include
    ${SOPHUS_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIR}
)

add_library(measurements SHARED
    measurements/ImuMeasurement.cpp
    measurements/LidarMeasurement.cpp
    measurements/PlaneFitMeasurement.cpp
    measurements/GlobalUpdateMeasurement.cpp
)

target_include_directories(measurements PUBLIC
    ${PROJECT_SOURCE_DIR}/include
    ${SOPHUS_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(measurements
    state
)

add_library(manifold_ukf SHARED
    Estimator.cpp
    kalman/History.cpp
    kalman/Prediction.cpp
    kalman/Extrinsics.cpp
    kalman/updates/LidarUpdate.cpp
    kalman/updates/PlanefitUpdate.cpp
    kalman/updates/GlobalUpdate.cpp
)

target_include_directories(manifold_ukf PUBLIC
    ${PROJECT_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIR}
    ${SOPHUS_INCLUDE_DIR}
    ${YAMLCPP_INCLUDE_DIRS}
)

target_link_libraries(manifold_ukf
    state
    measurements
    ${YAMLCPP_LIBRARY}
)
