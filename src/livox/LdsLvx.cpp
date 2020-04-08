/**
 * @file    LdsLvx.cpp
 *
 * @author  livox sdk, btran
 *
 * Copyright (c) organization
 *
 */

#include <cmath>
#include <cstring>
#include <functional>
#include <memory>
#include <thread>

#include <livox/lvx.hpp>

namespace lvx
{
LdsLvx::LdsLvx(uint32_t intervalMs)
    : LidarDataSource(intervalMs, SOURCE_LVX_FILE), m_isInitialized(false),
      m_lvxFile(std::make_shared<LvxFileHandler>()), m_threadReadLvx(nullptr), m_startReadLvx(false)
{
    m_packetsOfFrame.bufferCapacity = MAX_PACKETS_NUM_OF_FRAME * sizeof(LvxFilePacket);
    m_packetsOfFrame.packet = new uint8_t[MAX_PACKETS_NUM_OF_FRAME * sizeof(LvxFilePacket)];
    std::fill(m_remainSize, m_remainSize + MAX_SOURCE_LIDAR, 0);
}

LdsLvx::~LdsLvx()
{
    DEBUG_LOG("Destroy livox file data source\n");
    if (m_packetsOfFrame.packet != nullptr) {
        delete[] m_packetsOfFrame.packet;
    }

    if (m_threadReadLvx && m_threadReadLvx->joinable()) {
        m_threadReadLvx->join();
    }
}

int LdsLvx::initSource(const std::vector<std::any>& fileSource)
{
    if (fileSource.size() != 1) {
        throw std::runtime_error("Please provide livox file path only\n");
    }

    const char* lvxPath = std::any_cast<const char*>(fileSource[0]);
    return this->initLdsLvx(lvxPath);
}

void LdsLvx::prepareExit(void)
{
    m_lvxFile->closeLvxFile();
    DEBUG_LOG("Successfully close lvx file!\n");
}

int LdsLvx::initLdsLvx(const char* lvxPath)
{
    if (m_isInitialized) {
        DEBUG_LOG("Livox file data source is already inited!\n");
        return -1;
    }

    int ret = m_lvxFile->open(lvxPath, std::ios::in);
    if (ret) {
        DEBUG_LOG("Open %s file fail[%d]!\n", lvxPath, ret);
        return ret;
    }

    if (m_lvxFile->fileVersion() == LVX_FILE_V1) {
        resetLidarDataSource(SOURCE_RAW_LIDAR);
    } else {
        resetLidarDataSource(SOURCE_LVX_FILE);
    }

    m_lidarCount = m_lvxFile->deviceCount();
    DEBUG_LOG("There are %d lidar sources", m_lidarCount);

    if (!m_lidarCount || (m_lidarCount >= MAX_SOURCE_LIDAR)) {
        m_lvxFile->closeLvxFile();
        DEBUG_LOG("Lidar count error in %s : %d\n", lvxPath, m_lidarCount);
        return -1;
    }
    DEBUG_LOG("LvxFile[%s] have %d lidars\n", lvxPath, m_lidarCount);

    for (int i = 0; i < m_lidarCount; i++) {
        LvxFileDeviceInfo lvxDevInfo;
        m_lvxFile->getDeviceInfo(i, &lvxDevInfo);
        m_lidars[i].handle = i;
        m_lidars[i].connectState = CONNECT_STATE_SAMPLING;
        m_lidars[i].info.handle = i;
        m_lidars[i].info.type = lvxDevInfo.deviceType;
        memcpy(m_lidars[i].info.broadcast_code, lvxDevInfo.lidarBroadcastCode, sizeof(m_lidars[i].info.broadcast_code));

        if (m_lvxFile->fileVersion() == LVX_FILE_V1) {
            m_lidars[i].dataSrc = SOURCE_RAW_LIDAR;
        } else {
            m_lidars[i].dataSrc = SOURCE_LVX_FILE;
        }

        ExtrinsicParameter* pExtrinsic = &m_lidars[i].extrinsicParameter;
        pExtrinsic->euler[0] = lvxDevInfo.roll * M_PI / 180.0;
        pExtrinsic->euler[1] = lvxDevInfo.pitch * M_PI / 180.0;
        pExtrinsic->euler[2] = lvxDevInfo.yaw * M_PI / 180.0;
        pExtrinsic->trans[0] = lvxDevInfo.x;
        pExtrinsic->trans[1] = lvxDevInfo.y;
        pExtrinsic->trans[2] = lvxDevInfo.z;
        eulerAnglesToRotationMatrix(pExtrinsic->euler, pExtrinsic->rotation);
        pExtrinsic->enable = lvxDevInfo.extrinsicEnable;

        uint32_t queueSize = MAX_ETH_PACKET_QUEUE_SIZE * 16;
        queue_util::initQueue(&m_lidars[i].data, queueSize);
        queueSize = MAX_ETH_PACKET_QUEUE_SIZE;
        queue_util::initQueue(&m_lidars[i].imuData, queueSize);
    }

    m_threadReadLvx = std::make_shared<std::thread>(&LdsLvx::readLvxFile, this);
    m_isInitialized = true;

    this->startRead();

    return ret;
}

void LdsLvx::readLvxFile()
{
    while (!m_startReadLvx) {
    }

    DEBUG_LOG("Start to read lvx file.\n");

    int fileState = LVX_FILE_OK;
    int progress = 0;
    while (m_startReadLvx) {
        fileState = m_lvxFile->getPacketsOfFrame(&m_packetsOfFrame);
        if (!fileState) {
            uint32_t dataSize = m_packetsOfFrame.dataSize;
            uint8_t* packetBase = m_packetsOfFrame.packet;
            uint32_t dataOffset = 0;
            while (dataOffset < dataSize) {
                LivoxEthPacket* ethPacket;
                int32_t handle;
                uint8_t dataType;
                if (m_lvxFile->fileVersion()) {
                    LvxFilePacket* detailPacket = reinterpret_cast<LvxFilePacket*>(&packetBase[dataOffset]);
                    ethPacket = reinterpret_cast<LivoxEthPacket*>(&detailPacket->version);
                    handle = detailPacket->deviceIndex;
                } else {
                    LvxFilePacketV0* detailPacket = reinterpret_cast<LvxFilePacketV0*>(&packetBase[dataOffset]);
                    ethPacket = reinterpret_cast<LivoxEthPacket*>(&detailPacket->version);
                    handle = detailPacket->deviceIndex;
                }
                dataType = ethPacket->data_type;

                // packet length + device index
                dataOffset += (getEthPacketLen(dataType) + 1);
                if (dataType != kImu) {
                    LidarDataQueue* lidarDataQueue = &m_lidars[handle].data;
                    if ((lidarDataQueue != nullptr) && (handle < m_lidarCount)) {
                        while (queue_util::queueIsFull(lidarDataQueue)) {
                            std::this_thread::sleep_for(std::chrono::milliseconds(1));
                        }
                        queue_util::queuePushAny(lidarDataQueue, reinterpret_cast<uint8_t*>(ethPacket),
                                                 getEthPacketLen(dataType), 0, getPointsPerPacket(dataType));
                    }
                } else {
                    LidarDataQueue* lidarDataQueue = &m_lidars[handle].imuData;
                    if ((lidarDataQueue != nullptr) && (handle < m_lidarCount)) {
                        while (queue_util::queueIsFull(lidarDataQueue)) {
                            std::this_thread::sleep_for(std::chrono::milliseconds(1));
                        }
                        queue_util::queuePushAny(lidarDataQueue, reinterpret_cast<uint8_t*>(ethPacket),
                                                 getEthPacketLen(dataType), 0, getPointsPerPacket(dataType));
                    }
                }
            }
        } else {
            if (fileState != LVX_FILE_AT_END) {
                DEBUG_LOG("Exit read the lvx file, read file state[%d]!\n", fileState);
            } else {
                DEBUG_LOG("Read the lvx file complete!\n");
            }
            break;
        }

        if (progress != m_lvxFile->getLvxFileReadProgress()) {
            progress = m_lvxFile->getLvxFileReadProgress();
            DEBUG_LOG("Read progress : %d \n", progress);
        }
    }

    int32_t wait_cnt = 10;
    while (!isAllQueueEmpty()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        if (isAllQueueReadStop()) {
            --wait_cnt;
            if (wait_cnt <= 0) {
                break;
            }
        }
    }
    this->requestExit();
}

bool LdsLvx::isAllQueueEmpty()
{
    for (int i = 0; i < m_lidarCount; i++) {
        LidarDevice* p_lidar = &m_lidars[i];
        if (!queue_util::queueIsEmpty(&p_lidar->data)) {
            return false;
        }
    }

    return true;
}

bool LdsLvx::isAllQueueReadStop()
{
    bool isAllQReadStop = true;

    for (int i = 0; i < m_lidarCount; i++) {
        LidarDevice* p_lidar = &m_lidars[i];
        if (m_remainSize[i] != queue_util::queueIsEmpty(&p_lidar->data)) {
            m_remainSize[i] = queue_util::queueIsEmpty(&p_lidar->data);
            isAllQReadStop = false;
        }
    }

    return isAllQReadStop;
}
}  // namespace lvx
