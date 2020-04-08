/**
 * @file    LDS.cpp
 *
 * @brief   livox lidar data source
 *
 * @author  livox sdk, btran
 *
 * Copyright (c) organization
 *
 */

#include <cstring>
#include <experimental/filesystem>

#include <livox/lvx.hpp>

namespace lvx
{
uint64_t getStoragePacketTimestamp(StoragePacket* packet, const uint8_t dataSrc)
{
    LivoxEthPacket* rawPacket = reinterpret_cast<LivoxEthPacket*>(packet->rawData);
    LidarDataSourceStamp timestamp;
    memcpy(timestamp.stampBytes, rawPacket->timestamp, sizeof(timestamp));

    if (rawPacket->timestamp_type == kTimestampTypePps) {
        if (dataSrc != SOURCE_LVX_FILE) {
            return (timestamp.stamp + packet->timeRcv);
        } else {
            return timestamp.stamp;
        }
    } else if (rawPacket->timestamp_type == kTimestampTypeNoSync) {
        return timestamp.stamp;
    } else if (rawPacket->timestamp_type == kTimestampTypePtp) {
        return timestamp.stamp;
    } else if (rawPacket->timestamp_type == kTimestampTypePpsGps) {
        struct tm timeUtc;
        timeUtc.tm_isdst = 0;
        timeUtc.tm_year = rawPacket->timestamp[0] + 100;  // map 2000 to 1990
        timeUtc.tm_mon = rawPacket->timestamp[1];
        timeUtc.tm_mday = rawPacket->timestamp[2];
        timeUtc.tm_hour = rawPacket->timestamp[3];
        timeUtc.tm_min = 0;
        timeUtc.tm_sec = 0;

        uint64_t timeEpoch = mktime(&timeUtc);
        timeEpoch = timeEpoch * 1000000 + timestamp.stampWord.high;  // to us
        timeEpoch = timeEpoch * 1000;                                // to ns

        return timeEpoch;
    } else {
        DEBUG_LOG("Timestamp type[%d] invalid.\n", rawPacket->timestamp_type);
        return 0;
    }
}

uint32_t calculatePacketQueueSize(uint32_t intervalMs, uint32_t dataType)
{
    uint32_t queueSize = (intervalMs * getPacketNumPerSec(dataType)) / 1000;
    if (queueSize < MIN_ETH_PACKET_QUEUE_SIZE) {
        queueSize = MIN_ETH_PACKET_QUEUE_SIZE;
    } else if (queueSize > MAX_ETH_PACKET_QUEUE_SIZE) {
        queueSize = MAX_ETH_PACKET_QUEUE_SIZE;
    }

    return queueSize;
}

void parseCommandlineInputBdCode(const char* cammandlineStr, std::vector<std::string>& bdCodeList)
{
    char* strs = new char[strlen(cammandlineStr) + 1];
    strcpy(strs, cammandlineStr);

    std::string pattern = "&";
    char* bdStr = strtok(strs, pattern.c_str());
    std::string invalidBd = "000000000";
    while (bdStr != NULL) {
        DEBUG_LOG("Commandline input bd:%s\n", bdStr);
        if ((BD_CODE_SIZE == strlen(bdStr)) && (NULL == strstr(bdStr, invalidBd.c_str()))) {
            bdCodeList.emplace_back(bdStr);
        } else {
            DEBUG_LOG("Invalid bd:%s!\n", bdStr);
        }
        bdStr = strtok(NULL, pattern.c_str());
    }

    delete[] strs;
}

void eulerAnglesToRotationMatrix(const EulerAngle& euler, RotationMatrix& matrix)
{
    double cosRoll = cos(static_cast<double>(euler[0]));
    double cosPitch = cos(static_cast<double>(euler[1]));
    double cosYaw = cos(static_cast<double>(euler[2]));
    double sinRoll = sin(static_cast<double>(euler[0]));
    double sinPitch = sin(static_cast<double>(euler[1]));
    double sinYaw = sin(static_cast<double>(euler[2]));

    matrix[0][0] = cosPitch * cosYaw;
    matrix[0][1] = sinRoll * sinPitch * cosYaw - cosRoll * sinYaw;
    matrix[0][2] = cosRoll * sinPitch * cosYaw + sinRoll * sinYaw;

    matrix[1][0] = cosPitch * sinYaw;
    matrix[1][1] = sinRoll * sinPitch * sinYaw + cosRoll * cosYaw;
    matrix[1][2] = cosRoll * sinPitch * sinYaw - sinRoll * cosYaw;

    matrix[2][0] = -sinPitch;
    matrix[2][1] = sinRoll * cosPitch;
    matrix[2][2] = cosRoll * cosPitch;
}

void pointExtrisincCompensation(PointXYZ* dstPoint, const PointXYZ& srcPoint, const ExtrinsicParameter& extrinsic)
{
    dstPoint->x = srcPoint.x * extrinsic.rotation[0][0] + srcPoint.y * extrinsic.rotation[0][1] +
                  srcPoint.z * extrinsic.rotation[0][2] + extrinsic.trans[0];
    dstPoint->y = srcPoint.x * extrinsic.rotation[1][0] + srcPoint.y * extrinsic.rotation[1][1] +
                  srcPoint.z * extrinsic.rotation[1][2] + extrinsic.trans[1];
    dstPoint->z = srcPoint.x * extrinsic.rotation[2][0] + srcPoint.y * extrinsic.rotation[2][1] +
                  srcPoint.z * extrinsic.rotation[2][2] + extrinsic.trans[2];
}

// Livox point procees for different raw data format
uint8_t* livoxPointToPXYZRTL(uint8_t* pointBuffer, LivoxEthPacket* ethPacket, const ExtrinsicParameter& extrinsic)
{
    LivoxPointXYZRTL* dstPoint = reinterpret_cast<LivoxPointXYZRTL*>(pointBuffer);
    uint32_t pointsPerPacket = getPointsPerPacket(ethPacket->data_type);
    LivoxPoint* rawPoint = reinterpret_cast<LivoxPoint*>(ethPacket->data);

    while (pointsPerPacket) {
        convertRawPoint(reinterpret_cast<LivoxPointXYZR*>(dstPoint), rawPoint);
        if (extrinsic.enable) {
            PointXYZ srcPoint = *(reinterpret_cast<PointXYZ*>(dstPoint));
            pointExtrisincCompensation(reinterpret_cast<PointXYZ*>(dstPoint), srcPoint, extrinsic);
        }
        dstPoint->tag = 0;
        dstPoint->line = 0;
        ++rawPoint;
        ++dstPoint;
        --pointsPerPacket;
    }

    return reinterpret_cast<uint8_t*>(dstPoint);
}

uint8_t* livoxRawPointToPXYZRTL(uint8_t* pointBuffer, LivoxEthPacket* ethPacket, const ExtrinsicParameter& extrinsic)
{
    LivoxPointXYZRTL* dstPoint = reinterpret_cast<LivoxPointXYZRTL*>(pointBuffer);
    uint32_t pointsPerPacket = getPointsPerPacket(ethPacket->data_type);
    LivoxRawPoint* rawPoint = reinterpret_cast<LivoxRawPoint*>(ethPacket->data);

    while (pointsPerPacket) {
        convertRawPoint(reinterpret_cast<LivoxPointXYZR*>(dstPoint), rawPoint);
        if (extrinsic.enable) {
            PointXYZ srcPoint = *(reinterpret_cast<PointXYZ*>(dstPoint));
            pointExtrisincCompensation(reinterpret_cast<PointXYZ*>(dstPoint), srcPoint, extrinsic);
        }
        dstPoint->tag = 0;
        dstPoint->line = 0;
        ++rawPoint;
        ++dstPoint;
        --pointsPerPacket;
    }

    return reinterpret_cast<uint8_t*>(dstPoint);
}

uint8_t* livoxSpherPointToPXYZRTL(uint8_t* pointBuffer, LivoxEthPacket* ethPacket, const ExtrinsicParameter& extrinsic)
{
    LivoxPointXYZRTL* dstPoint = reinterpret_cast<LivoxPointXYZRTL*>(pointBuffer);
    uint32_t pointsPerPacket = getPointsPerPacket(ethPacket->data_type);
    LivoxSpherPoint* rawPoint = reinterpret_cast<LivoxSpherPoint*>(ethPacket->data);

    while (pointsPerPacket) {
        convertRawPoint(reinterpret_cast<LivoxPointXYZR*>(dstPoint), rawPoint);
        if (extrinsic.enable) {
            PointXYZ srcPoint = *(reinterpret_cast<PointXYZ*>(dstPoint));
            pointExtrisincCompensation(reinterpret_cast<PointXYZ*>(dstPoint), srcPoint, extrinsic);
        }
        dstPoint->tag = 0;
        dstPoint->line = 0;
        ++rawPoint;
        ++dstPoint;
        --pointsPerPacket;
    }

    return reinterpret_cast<uint8_t*>(dstPoint);
}

uint8_t* livoxExtendRawPointToPXYZRTL(uint8_t* pointBuffer, LivoxEthPacket* ethPacket,
                                      const ExtrinsicParameter& extrinsic)
{
    LivoxPointXYZRTL* dstPoint = reinterpret_cast<LivoxPointXYZRTL*>(pointBuffer);
    uint32_t pointsPerPacket = getPointsPerPacket(ethPacket->data_type);
    LivoxExtendRawPoint* rawPoint = reinterpret_cast<LivoxExtendRawPoint*>(ethPacket->data);

    uint8_t lineId = 0;
    while (pointsPerPacket) {
        convertRawPoint(reinterpret_cast<LivoxPointXYZR*>(dstPoint), reinterpret_cast<LivoxRawPoint*>(rawPoint));
        if (extrinsic.enable) {
            PointXYZ srcPoint = *(reinterpret_cast<PointXYZ*>(dstPoint));
            pointExtrisincCompensation(reinterpret_cast<PointXYZ*>(dstPoint), srcPoint, extrinsic);
        }
        dstPoint->tag = rawPoint->tag;
        dstPoint->line = lineId;
        dstPoint->line = dstPoint->line % 6;
        ++rawPoint;
        ++dstPoint;
        ++lineId;
        --pointsPerPacket;
    }

    return reinterpret_cast<uint8_t*>(dstPoint);
}

uint8_t* livoxExtendSpherPointToPXYZRTL(uint8_t* pointBuffer, LivoxEthPacket* ethPacket,
                                        const ExtrinsicParameter& extrinsic)
{
    LivoxPointXYZRTL* dstPoint = reinterpret_cast<LivoxPointXYZRTL*>(pointBuffer);
    uint32_t pointsPerPacket = getPointsPerPacket(ethPacket->data_type);
    LivoxExtendSpherPoint* rawPoint = reinterpret_cast<LivoxExtendSpherPoint*>(ethPacket->data);

    uint8_t lineId = 0;
    while (pointsPerPacket) {
        convertRawPoint(reinterpret_cast<LivoxPointXYZR*>(dstPoint), reinterpret_cast<LivoxSpherPoint*>(rawPoint));
        if (extrinsic.enable) {
            PointXYZ srcPoint = *(reinterpret_cast<PointXYZ*>(dstPoint));
            pointExtrisincCompensation(reinterpret_cast<PointXYZ*>(dstPoint), srcPoint, extrinsic);
        }
        dstPoint->tag = rawPoint->tag;
        dstPoint->line = lineId;
        dstPoint->line = dstPoint->line % 6;
        ++rawPoint;
        ++dstPoint;
        ++lineId;
        --pointsPerPacket;
    }

    return reinterpret_cast<uint8_t*>(dstPoint);
}

uint8_t* livoxDualExtendRawPointToPXYZRTL(uint8_t* pointBuffer, LivoxEthPacket* ethPacket,
                                          const ExtrinsicParameter& extrinsic)
{
    LivoxPointXYZRTL* dstPoint = reinterpret_cast<LivoxPointXYZRTL*>(pointBuffer);
    uint32_t pointsPerPacket = getPointsPerPacket(ethPacket->data_type);
    LivoxExtendRawPoint* rawPoint = reinterpret_cast<LivoxExtendRawPoint*>(ethPacket->data);

    uint8_t lineId = 0;
    while (pointsPerPacket) {
        convertRawPoint(reinterpret_cast<LivoxPointXYZR*>(dstPoint), reinterpret_cast<LivoxRawPoint*>(rawPoint));
        if (extrinsic.enable) {
            PointXYZ srcPoint = *(reinterpret_cast<PointXYZ*>(dstPoint));
            pointExtrisincCompensation(reinterpret_cast<PointXYZ*>(dstPoint), srcPoint, extrinsic);
        }
        dstPoint->tag = rawPoint->tag;

        // LivoxDualExtendRawPoint = 2*LivoxExtendRawPoint
        dstPoint->line = lineId / 2;

        dstPoint->line = dstPoint->line % 6;
        ++rawPoint;
        ++dstPoint;
        ++lineId;
        --pointsPerPacket;
    }

    return reinterpret_cast<uint8_t*>(dstPoint);
}

uint8_t* livoxDualExtendSpherPointToPXYZRTL(uint8_t* pointBuffer, LivoxEthPacket* ethPacket,
                                            const ExtrinsicParameter& extrinsic)
{
    LivoxPointXYZRTL* dstPoint = reinterpret_cast<LivoxPointXYZRTL*>(pointBuffer);
    uint32_t pointsPerPacket = getPointsPerPacket(ethPacket->data_type);
    LivoxDualExtendSpherPoint* rawPoint = reinterpret_cast<LivoxDualExtendSpherPoint*>(ethPacket->data);

    uint8_t lineId = 0;
    while (pointsPerPacket) {
        convertRawPoint(reinterpret_cast<LivoxPointXYZR*>(dstPoint), reinterpret_cast<LivoxPointXYZR*>(dstPoint + 1),
                        (LivoxDualExtendSpherPoint*)rawPoint);
        if (extrinsic.enable) {
            PointXYZ srcPoint = *(reinterpret_cast<PointXYZ*>(dstPoint));
            pointExtrisincCompensation(reinterpret_cast<PointXYZ*>(dstPoint), srcPoint, extrinsic);
        }
        dstPoint->tag = rawPoint->tag1;
        dstPoint->line = lineId;
        dstPoint->line = dstPoint->line % 6;
        ++dstPoint;

        if (extrinsic.enable) {
            PointXYZ srcPoint = *(reinterpret_cast<PointXYZ*>(dstPoint));
            pointExtrisincCompensation(reinterpret_cast<PointXYZ*>(dstPoint), srcPoint, extrinsic);
        }
        dstPoint->tag = rawPoint->tag2;
        dstPoint->line = lineId;
        dstPoint->line = dstPoint->line % 6;
        ++dstPoint;

        // only increase one
        ++rawPoint;
        ++lineId;
        --pointsPerPacket;
    }

    return reinterpret_cast<uint8_t*>(dstPoint);
}

uint8_t* processLivoxImuData(uint8_t* pointBuffer, LivoxEthPacket* ethPacket)
{
    memcpy(pointBuffer, ethPacket->data, sizeof(LivoxImuPoint));
    return pointBuffer;
}

static constexpr PointConvertHandler TO_PXYZI_HANDLER_TABLE[kMaxPointDataType] = {livoxRawPointToPXYZRTL,
                                                                                  livoxSpherPointToPXYZRTL,
                                                                                  livoxExtendRawPointToPXYZRTL,
                                                                                  livoxExtendSpherPointToPXYZRTL,
                                                                                  livoxDualExtendRawPointToPXYZRTL,
                                                                                  livoxDualExtendSpherPointToPXYZRTL,
                                                                                  nullptr};

PointConvertHandler getConvertHandler(uint8_t dataType)
{
    if (dataType < kMaxPointDataType)
        return TO_PXYZI_HANDLER_TABLE[dataType];
    else
        return nullptr;
}

uint8_t* fillZeroPointXYZRTL(uint8_t* pointBuffer, const uint32_t num)
{
    LivoxPointXYZRTL* dstPoint = reinterpret_cast<LivoxPointXYZRTL*>(pointBuffer);
    uint32_t pointsPerPacket = num;

    while (pointsPerPacket) {
        dstPoint->x = 0;
        dstPoint->y = 0;
        dstPoint->z = 0;
        dstPoint->reflectivity = 0;
        dstPoint->tag = 0;
        dstPoint->line = 0;
        ++dstPoint;
        --pointsPerPacket;
    }

    return reinterpret_cast<uint8_t*>(dstPoint);
}

LidarDataSource::LidarDataSource(const uint32_t bufferTimeMs, const uint8_t dataSrc)
    : m_lidarCount(MAX_SOURCE_LIDAR), m_bufferTimeMs(bufferTimeMs), m_dataSrc(dataSrc), m_requestExit(false)
{
    this->resetLidarDataSource(dataSrc);
};

LidarDataSource::~LidarDataSource()
{
    m_lidarCount = 0;
    this->resetLidarDataSource(0);
};

void LidarDataSource::resetLidar(LidarDevice* lidar, const uint8_t dataSrc)
{
    queue_util::deInitQueue(&lidar->data);
    queue_util::deInitQueue(&lidar->imuData);

    memset(lidar, 0, sizeof(LidarDevice));

    // unallocated state
    lidar->handle = MAX_SOURCE_LIDAR;
    lidar->dataSrc = dataSrc;
    lidar->dataIsPublished = false;
    lidar->connectState = CONNECT_STATE_OFF;
}

void LidarDataSource::setLidarDataSrc(LidarDevice* lidar, const uint8_t dataSrc)
{
    lidar->dataSrc = dataSrc;
}

void LidarDataSource::resetLidarDataSource(const uint8_t dataSrc)
{
    m_lidarCount = MAX_SOURCE_LIDAR;
    for (uint32_t i = 0; i < MAX_SOURCE_LIDAR; i++) {
        resetLidar(&m_lidars[i], dataSrc);
    }
}

uint8_t LidarDataSource::getDeviceType(uint8_t handle)
{
    if (handle < MAX_SOURCE_LIDAR) {
        return m_lidars[handle].info.type;
    } else {
        return kDeviceTypeHub;
    }
}

void LidarDataSource::prepareExit(void)
{
    DEBUG_LOG("Lidar abstract class exit\n");
}
}  // namespace lvx
