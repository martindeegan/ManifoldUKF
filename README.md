# Manifold UKF

Unscented Kalman filter on matrix lie groups implementation. Was implemented for MAAV (Michigan Autonomous Aerial Vehicles) during the 2018-2019 school year.


## Filter Details

The filter was developed for a quadcopter with an IMU, and a localizer update (Visual/Lidar SLAM or Odometry). Other update measurements supported are a downward facing lidar and a downward facing depth cloud (plane fitting a flat floor). It is designed to run at the frequency of the IMU. A history buffer with configurable length keeps track of when IMU measurements and other update measurements are received and the filter will replay measurements if a delayed measurement is received. Updates have extrinsics support as well as individual compatability outlier rejection.

The filter executable is not included in this project as it contains too much MAAV specific code.

https://www.youtube.com/watch?v=VQ0MW_FqMHI
