/**
 * @file    LidarDataDistributeController.cpp
 *
 * @author  livox sdk, btran
 *
 * Copyright (c) organization
 *
 */

#include <experimental/filesystem>

#include <livox/lvx.hpp>

namespace lvx
{
LidarDataDistributeController::LidarDataDistributeController(const int dataSrc, const double frq)
    : m_lds(nullptr), m_dataSrc(dataSrc), m_publishFrq(frq), m_publishIntervalMs(1000 / frq)
{
}

LidarDataDistributeController::~LidarDataDistributeController()
{
    DEBUG_LOG("lddc exit\n");

    if (m_lds) {
        m_lds->prepareExit();
        // m_lds is designed to be static object, so no delete here
        m_lds = nullptr;
    }
}

int LidarDataDistributeController::registerLds(LidarDataSource* lds)
{
    if (m_lds == nullptr) {
        m_lds = lds;
        return 0;
    } else {
        return -1;
    }
}

void LidarDataDistributeController::prepareExit(void)
{
    if (m_lds) {
        m_lds->prepareExit();
        // m_lds is designed to be static object, so no delete here
        m_lds = nullptr;
    }
}

LivoxPointCloud<LivoxPointXYZRTL>::Ptr LidarDataDistributeController::getLidarPointCloudData(LidarDataQueue* queue,
                                                                                             const uint32_t packetNum,
                                                                                             const uint8_t handle)
{
    using LivoxPointCloudType = LivoxPointCloud<LivoxPointXYZRTL>;

    uint64_t timestamp = 0;
    uint64_t lastTimestamp = 0;
    uint32_t publishedPacket = 0;

    typename LivoxPointCloudType::Ptr livoxCloud(new LivoxPointCloudType);

    livoxCloud->m_header.m_frameId = this->getLidarSpecificHeaderStr(handle);
    livoxCloud->m_height = 1;
    livoxCloud->m_width = 0;

    uint8_t dataSource = m_lds->m_lidars[handle].dataSrc;
    StoragePacket storagePacket;

    while (publishedPacket < packetNum) {
        queue_util::queueProPop(queue, &storagePacket);

        LivoxEthPacket* rawPacket = reinterpret_cast<LivoxEthPacket*>(storagePacket.rawData);

        uint32_t packet_interval = getPacketInterval(rawPacket->data_type);
        int64_t packetLossThresholdLower = packet_interval + packet_interval / 2;

        timestamp = getStoragePacketTimestamp(&storagePacket, dataSource);

        int64_t packetGap = timestamp - lastTimestamp;

        if ((packetGap > packetLossThresholdLower) && publishedPacket && m_lds->m_lidars[handle].dataIsPublished) {
            DEBUG_LOG("Lidar[%d] packet loss, interval is %ldus", handle, packetGap);
            int64_t packetLossThresholdUpper = packet_interval * packetNum;

            // skip when gap is too large
            if (packetGap > packetLossThresholdUpper) {
                DEBUG_LOG("Packet gap too large\n");
                break;
            }

            typename LivoxPointCloudType::PointType livoxPoint;
            for (uint32_t i = 0; i < storagePacket.pointNum; i++) {
                livoxCloud->m_points.emplace_back(livoxPoint);
            }
            lastTimestamp = lastTimestamp + packet_interval;
            ++publishedPacket;
            continue;
        }

        if (!publishedPacket) {
            livoxCloud->m_header.m_stamp = timestamp / 1000.0;
        }

        livoxCloud->m_width += storagePacket.pointNum;

        uint8_t pointBuffer[2048];
        if (dataSource != SOURCE_LVX_FILE) {
            PointConvertHandler pf_point_convert = getConvertHandler(rawPacket->data_type);

            if (pf_point_convert) {
                pf_point_convert(pointBuffer, rawPacket, m_lds->m_lidars[handle].extrinsicParameter);
            } else {
                // Skip the packet
                DEBUG_LOG("Lidar[%d] unknown packet type[%d]", handle, rawPacket->data_type);
                break;
            }
        } else {
            livoxPointToPXYZRTL(pointBuffer, rawPacket, m_lds->m_lidars[handle].extrinsicParameter);
        }

        LivoxPointXYZRTL* dstPoint = reinterpret_cast<LivoxPointXYZRTL*>(pointBuffer);

        std::copy(dstPoint, dstPoint + storagePacket.pointNum, std::back_inserter(livoxCloud->m_points));

        queue_util::queuePopUpdate(queue);
        lastTimestamp = timestamp;
        ++publishedPacket;
    }

    if (!m_lds->m_lidars[handle].dataIsPublished) {
        m_lds->m_lidars[handle].dataIsPublished = true;
    }

    if (livoxCloud->m_points.size() != livoxCloud->m_width * livoxCloud->m_height) {
        livoxCloud->m_width = livoxCloud->m_points.size();
    }

    return livoxCloud;
}

LivoxImu::Ptr LidarDataDistributeController::getLidarImuData(LidarDataQueue* queue, const uint32_t packetNum,
                                                             const uint8_t handle)
{
    uint64_t timestamp = 0;
    uint32_t publishedPacket = 0;

    LivoxImu::Ptr imuData(new LivoxImu);

    imuData->m_header.m_frameId = this->getLidarSpecificHeaderStr(handle);

    uint8_t dataSource = m_lds->m_lidars[handle].dataSrc;
    StoragePacket storagePacket;
    queue_util::queueProPop(queue, &storagePacket);

    LivoxEthPacket* rawPacket = reinterpret_cast<LivoxEthPacket*>(storagePacket.rawData);
    timestamp = getStoragePacketTimestamp(&storagePacket, dataSource);

    if (timestamp >= 0) {
        // TODO: need to check the timestamp conversion
        imuData->m_header.m_stamp = timestamp / 1000000000.0;
        // ros::Time(timestamp / 1000000000.0); // to ros time stamp
    }

    uint8_t pointBuffer[2048];
    processLivoxImuData(pointBuffer, rawPacket);

    imuData->m_imuPoint = *(reinterpret_cast<LivoxImuPoint*>(pointBuffer));

    queue_util::queuePopUpdate(queue);
    ++publishedPacket;

    return imuData;
}

std::vector<LivoxPointCloud<LivoxPointXYZRTL>::Ptr>
LidarDataDistributeController::pollLidarPointCloudData(const uint8_t handle, LidarDevice* const lidar)
{
    LidarDataQueue* pQueue = &lidar->data;
    if (pQueue == nullptr) {
        return {};
    }

    std::vector<LivoxPointCloud<LivoxPointXYZRTL>::Ptr> result;
    while (!queue_util::queueIsEmpty(pQueue)) {
        uint32_t usedSize = queue_util::queueUsedSize(pQueue);
        uint32_t onetimePublishPackets = getPacketNumPerSec(lidar->info.type) / m_publishFrq;
        if (usedSize < onetimePublishPackets) {
            break;
        }

        result.emplace_back(this->getLidarPointCloudData(pQueue, onetimePublishPackets, handle));
    }

    return result;
}

std::vector<LivoxImu::Ptr> LidarDataDistributeController::pollLidarImuData(const uint8_t handle,
                                                                           LidarDevice* const lidar)
{
    LidarDataQueue* p_queue = &lidar->imuData;
    if (p_queue == nullptr) {
        return {};
    }

    std::vector<LivoxImu::Ptr> result;

    while (!queue_util::queueIsEmpty(p_queue)) {
        result.emplace_back(this->getLidarImuData(p_queue, 1, handle));
    }

    return result;
}

std::string LidarDataDistributeController::getLidarSpecificHeaderStr(const uint8_t handle) const
{
    std::stringstream ss;
    ss << "livox_frame";

    ss << "_" << m_lds->m_lidars[handle].info.broadcast_code;
    ss << "_" << std::to_string(m_lds->m_lidars[handle].info.type);

    return ss.str();
}

bool LidarDataDistributeController::createOutputDir(const std::string& outputPath)
{
    namespace fs = std::experimental::filesystem;

    if (!fs::is_directory(outputPath) && !fs::create_directories(outputPath)) {
        DEBUG_LOG("Failed to create %s", outputPath.c_str());
        return false;
    }

    return true;
}
}  // namespace lvx
