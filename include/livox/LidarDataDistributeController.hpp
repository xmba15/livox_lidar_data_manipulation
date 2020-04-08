/**
 * @file    LidarDataDistributeController.hpp
 *
 * @author  livox sdk, btran
 *
 * Copyright (c) organization
 *
 */

#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "LidarDataSource.hpp"

namespace lvx
{
class LidarDataDistributeController
{
 public:
    LidarDataDistributeController(const int dataSrc, const double frq);
    ~LidarDataDistributeController();

    int registerLds(LidarDataSource* lds);

    void prepareExit();

    std::vector<LivoxPointCloud<LivoxPointXYZRTL>::Ptr> pollLidarPointCloudData(const uint8_t handle,
                                                                                LidarDevice* const lidar);

    std::vector<LivoxImu::Ptr> pollLidarImuData(const uint8_t handle, LidarDevice* const lidar);

    static bool createOutputDir(const std::string& outputPath);

    LidarDataSource* m_lds;

    using Ptr = std::shared_ptr<LidarDataDistributeController>;
    using ConstPtr = std::shared_ptr<const LidarDataDistributeController>;

 private:
    LivoxPointCloud<LivoxPointXYZRTL>::Ptr getLidarPointCloudData(LidarDataQueue* queue, const uint32_t packetNum,
                                                                  const uint8_t handle);

    LivoxImu::Ptr getLidarImuData(LidarDataQueue* queue, const uint32_t packetNum, const uint8_t handle);
    std::string getLidarSpecificHeaderStr(const uint8_t handle) const;

 private:
    uint8_t m_dataSrc;

    double m_publishFrq;

    int32_t m_publishIntervalMs;
};
}  // namespace lvx
