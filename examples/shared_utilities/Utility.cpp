/**
 * @file    Utility.cpp
 *
 * @author  btran
 *
 * Copyright (c) organization
 *
 */


#include "Utility.hpp"
#include "TypesConversion.hpp"

namespace lvx
{
bool writeToPcd(const LidarDataDistributeController::Ptr &lddc, const std::string &outputPath)
{
    using PCLPointCloudType = pcl::PointXYZI;
    using PCLPointCloud = pcl::PointCloud<PCLPointCloudType>;
    using PCLPointCloudPtr = PCLPointCloud::Ptr;

    if (!lddc->m_lds)
    {
        DEBUG_LOG("No lidar data source\n");
        return false;
    }

    DEBUG_LOG("Start writing data\n");

    for (uint32_t lidarId = 0; lidarId < lddc->m_lds->m_lidarCount; ++lidarId)
    {
        LidarDevice *lidar = &lddc->m_lds->m_lidars[lidarId];
        LidarDataQueue *pQueue = &lidar->data;

        if ((lidar->connectState != CONNECT_STATE_SAMPLING) || (pQueue == nullptr))
        {
            continue;
        }

        DEBUG_LOG("Writing data from lidar %d\n", lidarId);
        auto livoxPointCLouds = lddc->pollLidarPointCloudData(lidarId, lidar);

        for (const auto &elem : livoxPointCLouds)
        {
            auto pclPointCloud = toPclPointCloudPtr<lvx::LivoxPointXYZRTL, PCLPointCloudType>(elem);

            std::stringstream ss;
            ss << outputPath << "/" << pclPointCloud->header.frame_id << "_" << pclPointCloud->header.stamp << ".pcd";
            pcl::io::savePCDFile(ss.str(), *pclPointCloud);
        }

        lddc->pollLidarImuData(lidarId, lidar);
    }

    if (lddc->m_lds->isRequestExit())
    {
        lddc->prepareExit();
    }

    return true;
}

bool visualizePointcloud(const LidarDataDistributeController::Ptr &lddc, pcl::visualization::PCLVisualizer &viewer,
                         const int spinOnceTime, const ColorCoding colorCoding)
{
    using PCLPointCloudType = pcl::PointXYZRGB;
    using PCLPointCloud = pcl::PointCloud<PCLPointCloudType>;
    using PCLPointCloudPtr = PCLPointCloud::Ptr;

    if (!lddc->m_lds)
    {
        DEBUG_LOG("No lidar data source\n");
        return false;
    }

    DEBUG_LOG("Start writing data\n");

    for (uint32_t i = 0; i < lddc->m_lds->m_lidarCount; i++)
    {
        uint32_t lidarId = i;
        LidarDevice *lidar = &lddc->m_lds->m_lidars[lidarId];
        LidarDataQueue *pQueue = &lidar->data;

        if ((lidar->connectState != CONNECT_STATE_SAMPLING) || (pQueue == nullptr))
        {
            continue;
        }

        DEBUG_LOG("Writing data from lidar %d\n", lidarId);
        auto livoxPointClouds = lddc->pollLidarPointCloudData(lidarId, lidar);

#if ENABLE_DEBUG
        int j = 0;
#endif
        for (const auto &elem : livoxPointClouds)
        {
            DEBUG_LOG("Package %d\n", j++);
            auto pclPointCloud = toPclPointCloudPtr<LivoxPointXYZRTL, PCLPointCloudType>(elem, colorCoding);

            viewer.removePointCloud();
            viewer.addPointCloud<PCLPointCloudType>(pclPointCloud);
            viewer.spinOnce(spinOnceTime);
        }

        lddc->pollLidarImuData(lidarId, lidar);
    }

    if (lddc->m_lds->isRequestExit())
    {
        lddc->prepareExit();
    }

    return true;
}
} // namespace lvx
