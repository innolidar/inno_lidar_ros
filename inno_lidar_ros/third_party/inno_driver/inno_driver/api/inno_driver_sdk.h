#pragma once
#include "inno_driver.h"
#if defined(_WIN32)
  #define LIDAR_API __declspec(dllexport)
#else
  #define LIDAR_API __attribute__((visibility("default")))
#endif

#include <inno_driver/common/inno_common.hpp>
#include <inno_driver/msg/data_types.hpp>
#include <inno_driver/msg/point_types.hpp>
#include <inno_driver/msg/imu_types.hpp>
#include <inno_driver/common/error_code.hpp>

template<typename PointT>
LIDAR_API InnoDriver<PointT>* CreatorDriver();