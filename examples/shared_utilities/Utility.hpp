/**
 * @file    Utility.hpp
 *
 * @author  btran
 *
 * Copyright (c) organization
 *
 */

#pragma once

#include <string>

#include <pcl/visualization/pcl_visualizer.h>

#include <livox/lvx.hpp>

namespace lvx
{
bool writeToPcd(const LidarDataDistributeController::Ptr& lddc, const std::string& outputPath);

bool visualizePointcloud(const LidarDataDistributeController::Ptr& lddc, pcl::visualization::PCLVisualizer& viewer,
                         const int spinOnceTime = 50, const ColorCoding colorCoding = ColorCoding::REFLECTIVITY);
}  // namespace lvx
