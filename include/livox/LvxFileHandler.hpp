/**
 * @file    LvxFileHandler.hpp
 *
 * @author  livox sdk, btran
 *
 * Copyright (c) organization
 *
 */

#pragma once

#include <fstream>
#include <ios>
#include <list>
#include <memory>
#include <mutex>
#include <vector>

#include <livox_sdk.h>

#include "Constants.hpp"

namespace lvx
{
enum DeviceState {
    DEVICE_STATE_DISCONNECT = 0,
    DEVICE_STATE_CONNECT = 1,
    DEVICE_STATE_SAMPLING = 2,
};

enum LvxFileState {
    LVX_FILE_OK = 0,
    LVX_FILE_NOT_EXIST,
    LVX_FILE_SIZE_FAULT,
    LVX_FILE_HEADER_FAULT,
    LVX_FILE_DEVICE_INFO_FAULT,
    LVX_FILE_DATA_INFO_FAULT,
    LVX_FILE_AT_END,
    LVX_FILE_READ_FAIL,
    LVX_FILE_FRAME_HEADER_ERROR,
    LVX_FILE_UNDEF_FAULT
};

enum LvxFileVersion {
    LVX_FILE_V0 = 0,
    LVX_FILE_V1 = 1,
    LVX_FILE_VERSION_UNDEF = 2,
};

typedef struct {
    uint8_t handle;
    DeviceState deviceState;
    DeviceInfo info;
} DeviceItem;

#pragma pack(1)
typedef struct {
    uint8_t signature[16];
    uint8_t version[4];
    uint32_t magicCode;
} LvxFilePublicHeader;

typedef struct {
    uint32_t frameDuration;
    uint8_t deviceCount;
} LvxFilePrivateHeader;

typedef struct {
    uint8_t lidarBroadcastCode[16];
    uint8_t hubBroadcastCode[16];
    uint8_t deviceIndex;
    uint8_t deviceType;
    uint8_t extrinsicEnable;
    float roll;
    float pitch;
    float yaw;
    float x;
    float y;
    float z;
} LvxFileDeviceInfo;

typedef struct {
    uint8_t deviceIndex;
    uint8_t version;
    uint8_t portId;
    uint8_t lidarIndex;
    uint8_t rsvd;
    uint32_t errorCode;
    uint8_t timestampType;
    uint8_t dataType;
    uint8_t timestamp[8];
    uint8_t rawPoint[MAX_POINT_SIZE];
    uint32_t packSize;
} LvxFilePacket;

typedef struct {
    uint64_t currentOffset;
    uint64_t nextOffset;
    uint64_t frameIndex;
} FrameHeader;

typedef struct {
    FrameHeader header;
    LvxFilePacket* packet;
} LvxFileFrame;

typedef struct {
    uint8_t deviceCount;
} LvxFilePrivateHeaderV0;

typedef struct {
    uint8_t lidarBroadcastCode[16];
    uint8_t hubBroadcastCode[16];
    uint8_t deviceIndex;
    uint8_t deviceType;
    float roll;
    float pitch;
    float yaw;
    float x;
    float y;
    float z;
} LvxFileDeviceInfoV0;

typedef struct {
    uint8_t deviceIndex;
    uint8_t version;
    uint8_t portId;
    uint8_t lidarIndex;
    uint8_t rsvd;
    uint32_t errorCode;
    uint8_t timestampType;
    uint8_t dataType;
    uint8_t timestamp[8];
    LivoxPoint rawPoint[100];
} LvxFilePacketV0;

typedef struct {
    uint64_t currentOffset;
    uint64_t nextOffset;
    uint64_t frameIndex;
    uint64_t packetCount;
} FrameHeaderV0;

typedef struct {
    FrameHeaderV0 header;
    LvxFilePacketV0* packet;
} LvxFileFrameV0;

typedef struct {
    uint32_t bufferCapacity;  // max buffer size
    uint32_t dataSize;        // frame data erea size
    uint8_t* packet;          // packet data erea
} OutPacketBuffer;

#pragma pack()
class LvxFileHandler
{
 public:
    LvxFileHandler();

    ~LvxFileHandler() = default;

    int open(const char* fileName, const std::ios_base::openmode& mode);

    bool eof() const;

    int initLvxFile();

    void initLvxFileHeader();

    void saveFrameToLvxFile(const std::list<LvxFilePacket>& pointPacketList);

    void handleBasePoints(LivoxEthPacket* data, LvxFilePacket& packet);

    void closeLvxFile();

    void addDeviceInfo(const LvxFileDeviceInfo& info)
    {
        m_deviceInfoList.emplace_back(info);
    }

    int getDeviceInfoListSize()
    {
        return m_deviceInfoList.size();
    }

    int deviceCount()
    {
        return m_deviceCount;
    }

    int getDeviceInfo(const uint8_t idx, LvxFileDeviceInfo* info);

    int getFileState(void)
    {
        return m_state;
    };

    int getPacketsOfFrame(OutPacketBuffer* packetsOfFrame);

    int getLvxFileReadProgress();

    int fileVersion() const
    {
        return m_fileVer;
    }

 private:
    uint64_t miniFileSize() const;

    uint64_t privateHeaderOffset() const;

    uint64_t dataStartOffset() const;

    // uint32_t packetNumOfFrame();

    bool readAndCheckHeader();

    bool addAndCheckDeviceInfo();

    bool prepareDataRead();

    uint64_t dataSizeOfFrame(const FrameHeader& frameHeader) const
    {
        return (frameHeader.nextOffset - frameHeader.currentOffset - sizeof(frameHeader));
    }

    uint64_t dataSizeOfFrame(const FrameHeaderV0& frameHeaderV0) const
    {
        return (frameHeaderV0.nextOffset - frameHeaderV0.currentOffset - sizeof(frameHeaderV0));
    }

 private:
    std::fstream m_lvxFile;

    std::vector<LvxFileDeviceInfo> m_deviceInfoList;

    uint8_t m_fileVer;

    uint8_t m_deviceCount;

    LvxFilePublicHeader m_publicHeader;

    LvxFilePrivateHeader m_privateHeader;

    LvxFilePrivateHeaderV0 m_privateHeaderv0;

    uint32_t m_curFrameIndex;

    uint64_t m_curOffset;

    uint32_t m_frameDuration;

    uint64_t m_dataStartOffset;

    uint64_t m_size;

    int m_mode;

    int m_state;
};
}  // namespace lvx
