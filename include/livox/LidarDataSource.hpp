/**
 * @file    LidarDataSource.hpp
 *
 * @author  livox sdk, btran
 *
 * Copyright (c) organization
 *
 */

#pragma once

#include <any>
#include <cmath>
#include <string>
#include <vector>

#include "Constants.hpp"
#include "LidarDataQueue.hpp"
#include "Types.hpp"

namespace lvx
{
static constexpr PacketInfoPair PACKET_INFO_PAIR_TABLE[kMaxPointDataType] = {{100, 100000, 10000, 1000000, 1318},  //
                                                                             {100, 100000, 10000, 1000000, 918},   //
                                                                             {96, 240000, 4167, 400000, 1362},     //
                                                                             {96, 240000, 4167, 400000, 978},      //
                                                                             {96, 480000, 4167, 400000, 1362},     //
                                                                             {48, 480000, 4167, 400000, 978},      //
                                                                             {1, 100, 10000000, 10000000, 42}};    //

uint64_t getStoragePacketTimestamp(StoragePacket* packet, const uint8_t dataSrc);
uint32_t calculatePacketQueueSize(uint32_t intervalMs, uint32_t dataType);
void parseCommandlineInputBdCode(const char* cammandlineStr, std::vector<std::string>& bdCodeList);
PointConvertHandler getConvertHandler(uint8_t dataType);
uint8_t* livoxPointToPXYZRTL(uint8_t* pointBuffer, LivoxEthPacket* ethPacket, const ExtrinsicParameter& extrinsic);
uint8_t* fillZeroPointXYZRTL(uint8_t* pointBuffer, const uint32_t num);
uint8_t* processLivoxImuData(uint8_t* pointBuffer, LivoxEthPacket* ethPacket);
void eulerAnglesToRotationMatrix(const EulerAngle& euler, RotationMatrix& matrix);
void pointExtrisincCompensation(PointXYZ* dstPoint, const ExtrinsicParameter& extrinsic);

inline uint32_t getPointInterval(const uint32_t dataType)
{
    return PACKET_INFO_PAIR_TABLE[dataType].pointInterval;
}

inline uint32_t getPacketNumPerSec(const uint32_t dataType)
{
    return PACKET_INFO_PAIR_TABLE[dataType].pointsPerSecond / PACKET_INFO_PAIR_TABLE[dataType].pointsPerPacket;
}

inline uint32_t getPointsPerPacket(const uint32_t dataType)
{
    return PACKET_INFO_PAIR_TABLE[dataType].pointsPerPacket;
}

inline uint32_t getPacketInterval(const uint32_t dataType)
{
    return PACKET_INFO_PAIR_TABLE[dataType].packetInterval;
}

inline uint32_t getEthPacketLen(const uint32_t dataType)
{
    return PACKET_INFO_PAIR_TABLE[dataType].packetLength;
}

inline void convertRawPoint(LivoxPointXYZR* dstPoint, LivoxPoint* rawPoint)
{
    dstPoint->x = rawPoint->x;
    dstPoint->y = rawPoint->y;
    dstPoint->z = rawPoint->z;
    dstPoint->reflectivity = static_cast<float>(rawPoint->reflectivity);
}

inline void convertRawPoint(LivoxPointXYZR* dstPoint, LivoxRawPoint* rawPoint)
{
    dstPoint->x = rawPoint->x / 1000.0f;
    dstPoint->y = rawPoint->y / 1000.0f;
    dstPoint->z = rawPoint->z / 1000.0f;
    dstPoint->reflectivity = static_cast<float>(rawPoint->reflectivity);
}

inline void convertRawPoint(LivoxPointXYZR* dstPoint, LivoxSpherPoint* rawPoint)
{
    double radius = rawPoint->depth / 1000.0;
    double theta = rawPoint->theta / 100.0 / 180 * M_PI;
    double phi = rawPoint->phi / 100.0 / 180 * M_PI;
    dstPoint->x = radius * sin(theta) * cos(phi);
    dstPoint->y = radius * sin(theta) * sin(phi);
    dstPoint->z = radius * cos(theta);
    dstPoint->reflectivity = static_cast<float>(rawPoint->reflectivity);
}

inline void convertRawPoint(LivoxPointXYZR* dstPoint1, LivoxPointXYZR* dstPoint2, LivoxDualExtendSpherPoint* rawPoint)
{
    double radius1 = rawPoint->depth1 / 1000.0;
    double radius2 = rawPoint->depth2 / 1000.0;
    double theta = rawPoint->theta / 100.0 / 180 * M_PI;
    double phi = rawPoint->phi / 100.0 / 180 * M_PI;
    dstPoint1->x = radius1 * sin(theta) * cos(phi);
    dstPoint1->y = radius1 * sin(theta) * sin(phi);
    dstPoint1->z = radius1 * cos(theta);
    dstPoint1->reflectivity = static_cast<float>(rawPoint->reflectivity1);

    (dstPoint2 + 1)->x = radius2 * sin(theta) * cos(phi);
    (dstPoint2 + 1)->y = radius2 * sin(theta) * sin(phi);
    (dstPoint2 + 1)->z = radius2 * cos(theta);
    (dstPoint2 + 1)->reflectivity = static_cast<float>(rawPoint->reflectivity2);
}

/**
 *  @brief Lidar data source abstract.
 */
class LidarDataSource
{
 protected:
    LidarDataSource(const uint32_t bufferTimeMs, const uint8_t dataSrc);
    virtual ~LidarDataSource();

 public:
    virtual int initSource(const std::vector<std::any>& fileSource) = 0;

    uint8_t getDeviceType(const uint8_t handle);

    static void resetLidar(LidarDevice* lidar, const uint8_t dataSrc);

    static void setLidarDataSrc(LidarDevice* lidar, const uint8_t dataSrc);

    void resetLidarDataSource(const uint8_t dataSrc);

    void requestExit()
    {
        m_requestExit = true;
    }

    bool isRequestExit() const
    {
        return m_requestExit;
    }

    void cleanRequestExit()
    {
        m_requestExit = false;
    }

    virtual void prepareExit(void);

    // lidar access handle
    uint8_t m_lidarCount;

    // the index is the handle
    LidarDevice m_lidars[MAX_SOURCE_LIDAR];

 protected:
    // buffer time before data in queue is read
    uint32_t m_bufferTimeMs;

    uint8_t m_dataSrc;

 private:
    volatile bool m_requestExit;
};
}  // namespace lvx
