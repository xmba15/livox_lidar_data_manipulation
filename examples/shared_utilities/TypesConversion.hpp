/**
 * @file    TypesConversion.hpp
 *
 * @author  btran
 *
 * Copyright (c) organization
 *
 */

#pragma once

#include <pcl/io/pcd_io.h>

#include <livox/lvx.hpp>

namespace lvx
{
template <typename LIVOX_POINT_TYPE, typename PCL_POINT_TYPE>
typename pcl::PointCloud<PCL_POINT_TYPE>::Ptr
toPclPointCloudPtr(const typename LivoxPointCloud<LIVOX_POINT_TYPE>::Ptr& lvxPointCloud,
                   const ColorCoding colorCoding = ColorCoding::NONE);
}  // namespace lvx
