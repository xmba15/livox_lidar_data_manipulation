/**
 * @file    TypesConversion.cpp
 *
 * @author  btran
 *
 * Copyright (c) organization
 *
 */

#include "TypesConversion.hpp"

namespace lvx
{
template <>
pcl::PointCloud<pcl::PointXYZRGB>::Ptr
toPclPointCloudPtr<LivoxPointXYZRTL, pcl::PointXYZRGB>(const LivoxPointCloud<LivoxPointXYZRTL>::Ptr& lvxPointCloud,
                                                       const ColorCoding colorCoding)
{
    using PCLPointCloudType = pcl::PointXYZRGB;
    using PCLPointCloud = pcl::PointCloud<PCLPointCloudType>;
    using PCLPointCloudPtr = PCLPointCloud::Ptr;

    PCLPointCloudPtr cloud(new PCLPointCloud);
    cloud->header.frame_id = lvxPointCloud->m_header.m_frameId;
    cloud->header.seq = lvxPointCloud->m_header.m_seq;
    cloud->header.stamp = lvxPointCloud->m_header.m_stamp;

    cloud->height = lvxPointCloud->m_height;
    cloud->width = lvxPointCloud->m_width;
    cloud->is_dense = lvxPointCloud->m_isDense;

    cloud->points.reserve(lvxPointCloud->m_points.size());

    for (const auto& elem : lvxPointCloud->m_points) {
        PCLPointCloudType point;
        point.x = elem.x;
        point.y = elem.y;
        point.z = elem.z;

        std::array<uint8_t, 3> rgb;
        switch (colorCoding) {
            case ColorCoding::REFLECTIVITY: {
                rgb = colorCodingReflectivityRGB(elem.reflectivity);
                break;
            }
            case ColorCoding::DEPTH: {
                rgb = colorCodingReflectivityRGB(elem.x);
                break;
            }
            default:
                throw std::runtime_error("Not supported color coding scheme\n");
        }

        point.r = rgb[0];
        point.g = rgb[1];
        point.b = rgb[2];
        cloud->points.emplace_back(point);
    }

    return cloud;
}

template <>
pcl::PointCloud<pcl::PointXYZI>::Ptr
toPclPointCloudPtr<LivoxPointXYZRTL, pcl::PointXYZI>(const LivoxPointCloud<LivoxPointXYZRTL>::Ptr& lvxPointCloud,
                                                     const ColorCoding colorCoding)
{
    using PCLPointCloudType = pcl::PointXYZI;
    using PCLPointCloud = pcl::PointCloud<PCLPointCloudType>;
    using PCLPointCloudPtr = PCLPointCloud::Ptr;

    PCLPointCloudPtr cloud(new PCLPointCloud);
    cloud->header.frame_id = lvxPointCloud->m_header.m_frameId;
    cloud->header.seq = lvxPointCloud->m_header.m_seq;
    cloud->header.stamp = lvxPointCloud->m_header.m_stamp;

    cloud->height = lvxPointCloud->m_height;
    cloud->width = lvxPointCloud->m_width;
    cloud->is_dense = lvxPointCloud->m_isDense;

    cloud->points.reserve(lvxPointCloud->m_points.size());

    for (const auto& elem : lvxPointCloud->m_points) {
        PCLPointCloudType point;
        point.x = elem.x;
        point.y = elem.y;
        point.z = elem.z;
        point.intensity = elem.reflectivity;
        cloud->points.emplace_back(point);
    }

    return cloud;
}
}  // namespace lvx
