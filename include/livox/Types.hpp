/**
 * @file    Types.hpp
 *
 * @author  livox sdk, btran
 *
 * Copyright (c) organization
 *
 */

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <Eigen/StdVector>

#include <livox_def.h>

#include "Constants.hpp"

namespace lvx
{
#pragma pack(1)
typedef struct {
    uint64_t timeRcv;  // receive time when data arrive
    uint32_t pointNum;
    uint8_t rawData[ETH_PACKET_MAX_LENGTH];
} StoragePacket;

#pragma pack()
typedef struct {
    StoragePacket* storagePacket;
    volatile uint32_t rdIdx;
    volatile uint32_t wrIdx;
    uint32_t mask;
    uint32_t size;  // must be power of 2
} LidarDataQueue;

// Lidar connect state
enum LidarConnectState {
    CONNECT_STATE_OFF = 0,
    CONNECT_STATE_ON = 1,
    CONNECT_STATE_CONFIG = 2,
    CONNECT_STATE_SAMPLING = 3,
};

// Device data source type
enum LidarDataSourceType {
    SOURCE_RAW_LIDAR = 0,  // data from raw lidar
    SOURCE_RAW_HUB = 1,    // data from lidar hub
    SOURCE_LVX_FILE = 2,   // data from parse lvx file
    SOURCE_UNDEF,          // undefined
};

// Lidar Data output type
enum LidarDataOutputType {
    OUTPUT_TO_ROS = 0,
    OUTPUT_TO_ROS_BAG_FILE = 1,
};

enum CoordinateType { COORDINATE_CARTESIAN = 0, COORDINATE_SPHERICAL = 1 };

enum LidarConfigCodeBit {
    CONFIG_FAN = 1,
    CONFIG_RETURN_MODE = 2,
    CONFIG_COORDINATE = 4,
    CONFIG_IMU_RATE = 8,
    CONFIG_GET_EXTRINSIC_PARAMETER = 16,
    CONFIG_UNDEF
};

enum ExtrinsicParameterType {
    NONE_EXTRINSIC_PARAMETER = 0,
    EXTRINSIC_PARAMETER_FROM_LIDAR = 1,
    EXTRINSIC_PARAMETER_FROM_XML = 2
};

typedef struct {
    uint32_t receivePacketCount;
    uint32_t lossPacketCount;
    int64_t lastTimeStamp;
    int64_t timeBase;  // unit:ns
    int64_t lastImuTimestamp;
    int64_t imuTimeBase;  // unit:ns
    uint32_t timeBaseState;
} LidarPacketStatistic;

// 8bytes stamp to uint64_t stamp
typedef union {
    struct {
        uint32_t low;
        uint32_t high;
    } stampWord;

    uint8_t stampBytes[8];
    int64_t stamp;
} LidarDataSourceStamp;

// Configuration in json config file for livox lidar
typedef struct {
    char broadcastCode[16];
    bool enableConnect;
    bool enableFan;
    uint32_t returnMode;
    uint32_t coordinate;
    uint32_t imuRate;
    uint32_t extrinsicParameterSource;
} UserRawConfig;

typedef struct {
    bool enableFan;
    uint32_t returnMode;
    uint32_t coordinate;
    uint32_t imuRate;
    uint32_t extrinsicParameterSource;
    volatile uint32_t setBits;
    volatile uint32_t getBits;
} UserConfig;

using EulerAngle = float[3];         // Roll, Pitch, Yaw, unit:radian.
using TranslationVector = float[3];  // x, y, z translation, unit: m.
using RotationMatrix = float[3][3];

typedef struct {
    EulerAngle euler;
    TranslationVector trans;
    RotationMatrix rotation;
    bool enable;
} ExtrinsicParameter;

// Lidar data source info abstract
typedef struct {
    uint8_t handle;        // Lidar access handle
    uint8_t dataSrc;       // From raw lidar or livox file
    bool dataIsPublished;  // Indicate the data of lidar whether is pubulished
    volatile LidarConnectState connectState;
    DeviceInfo info;
    LidarPacketStatistic statisticInfo;
    LidarDataQueue data;
    LidarDataQueue imuData;
    uint32_t firmwareVer;  // Firmware version of lidar
    UserConfig config;
    ExtrinsicParameter extrinsicParameter;
} LidarDevice;

typedef struct {
    uint32_t pointsPerPacket;
    uint32_t pointsPerSecond;
    uint32_t pointInterval;   // unit:ns
    uint32_t packetInterval;  // unit:ns
    uint32_t packetLength;
} PacketInfoPair;

#pragma pack(1)
typedef struct PointXYZ {
    PointXYZ() : x(0), y(0), z(0)
    {
    }

    float x;  // X axis, Unit:m
    float y;  // Y axis, Unit:m
    float z;  // Z axis, Unit:m
} PointXYZ;

typedef struct LivoxPointXYZR : public PointXYZ {
    LivoxPointXYZR() : PointXYZ(), reflectivity(0)
    {
    }

    float reflectivity;  // Reflectivity
} LivoxPointXYZR;

typedef struct LivoxPointXYZRTL : public LivoxPointXYZR {
    LivoxPointXYZRTL() : LivoxPointXYZR(), tag(0), line(0)
    {
    }

    uint8_t tag;   // Livox point tag
    uint8_t line;  // Laser line id
} LivoxPointXYZRTL;

struct LivoxDataFrameHeader {
    LivoxDataFrameHeader() : m_seq(0), m_stamp(), m_frameId()
    {
    }

    uint32_t m_seq;
    uint64_t m_stamp;
    std::string m_frameId;

    using Ptr = std::shared_ptr<LivoxDataFrameHeader>;
    using ConstPtr = std::shared_ptr<const LivoxDataFrameHeader>;
};

inline std::ostream& operator<<(std::ostream& out, const LivoxDataFrameHeader& h)
{
    out << "seq: " << h.m_seq;
    out << " stamp: " << h.m_stamp;
    out << " frameId: " << h.m_frameId << std::endl;
    return out;
}

class LivoxImu
{
 public:
    LivoxImu() : m_imuPoint()
    {
    }

    LivoxDataFrameHeader m_header;
    LivoxImuPoint m_imuPoint;

    using Ptr = std::shared_ptr<LivoxImu>;
    using ConstPtr = std::shared_ptr<const LivoxImu>;
};

template <typename POINT_TYPE = LivoxPointXYZRTL> class LivoxPointCloud
{
 public:
    LivoxPointCloud()
        : m_width(0), m_height(0), m_isDense(true), m_sensorOrigin({0, 0, 0, 1.0}),
          m_sensorOrientation(Eigen::Quaternionf::Identity())
    {
    }

    std::vector<POINT_TYPE> m_points;
    uint32_t m_width;
    uint32_t m_height;
    bool m_isDense;
    LivoxDataFrameHeader m_header;

    // homogenous coordinate
    Eigen::Vector4f m_sensorOrigin;
    Eigen::Quaternionf m_sensorOrientation;

    using PointType = POINT_TYPE;
    using VectorType = std::vector<POINT_TYPE>;
    using Ptr = std::shared_ptr<LivoxPointCloud<POINT_TYPE>>;
    using ConstPtr = std::shared_ptr<const LivoxPointCloud<POINT_TYPE>>;
};

#pragma pack()
typedef uint8_t* (*PointConvertHandler)(uint8_t* pointBuf, LivoxEthPacket* ethPacket,
                                        const ExtrinsicParameter& extrinsic);

enum ColorCoding { NONE = -1, REFLECTIVITY = 0, DEPTH = 1 };
}  // namespace lvx
