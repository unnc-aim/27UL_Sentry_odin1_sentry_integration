# Reused components

- PCL NormalEstimation: https://pointclouds.org/documentation/classpcl_1_1_normal_estimation.html — BSD license. Used through the installed public API; no copied implementation.
- Eigen: https://eigen.tuxfamily.org/ — provided through PCL.
- NumPy and SciPy Rotation: installed numerical libraries, BSD licenses.
- ROS2 Humble / Nav2: existing installed packages and navigation launch.
- odin_sentry_bringup: existing mount, configuration helper, raw filtering and device adapter.

No third-party repository is vendored. grid_map was considered; system installation requires administrator credentials on this host. The implementation uses the already-installed PCL surface estimator instead.
