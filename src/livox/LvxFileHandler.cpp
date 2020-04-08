/**
 * @file    LvxFileHandler.cpp
 *
 * @author  livox sdk, btran
 *
 * Copyright (c) organization
 *
 */

#include <cmath>
#include <cstring>
#include <memory>
#include <string>

#include <livox/lvx.hpp>

#include <rapidxml/rapidxml.hpp>
#include <rapidxml/rapidxml_utils.hpp>

namespace lvx
{
LvxFileHandler::LvxFileHandler()
    : m_fileVer(LVX_FILE_V1), m_deviceCount(0), m_curFrameIndex(0), m_curOffset(0), m_dataStartOffset(0), m_size(0),
      m_mode(0), m_state(0)
{
    memset(reinterpret_cast<void*>(&m_publicHeader), 0, sizeof(m_publicHeader));
    memset(reinterpret_cast<void*>(&m_privateHeader), 0, sizeof(m_privateHeader));
    memset(reinterpret_cast<void*>(&m_privateHeaderv0), 0, sizeof(m_privateHeaderv0));
    m_deviceInfoList.reserve(MAX_SOURCE_LIDAR);
}

bool LvxFileHandler::readAndCheckHeader()
{
    m_lvxFile.seekg(0, std::ios::beg);
    m_lvxFile.read(reinterpret_cast<char*>(&m_publicHeader), sizeof(m_publicHeader));

    if (strcmp((const char*)m_publicHeader.signature, LVX_HEADER_SIG_STR)) {
        return false;
    }

    if (m_publicHeader.version[1] > LVX_FILE_V1) {
        DEBUG_LOG("Unkown lvx file version[%d.%d.%d.%d]\n", m_publicHeader.version[0], m_publicHeader.version[1],
                  m_publicHeader.version[2], m_publicHeader.version[3]);
        return false;
    }

    m_fileVer = m_publicHeader.version[1];
    DEBUG_LOG("Livox file version[%d]\n", m_fileVer);

    return true;
}

uint64_t LvxFileHandler::miniFileSize() const
{
    if (m_fileVer == LVX_FILE_V1) {
        return (sizeof(LvxFilePublicHeader) + sizeof(LvxFilePrivateHeader) + sizeof(LvxFileDeviceInfo) +
                sizeof(FrameHeader) + sizeof(LvxFilePacket));
    } else {
        return (sizeof(LvxFilePublicHeader) + sizeof(LvxFilePrivateHeaderV0) + sizeof(LvxFileDeviceInfoV0) +
                sizeof(FrameHeaderV0) + sizeof(LvxFilePacketV0));
    }
}

uint64_t LvxFileHandler::privateHeaderOffset() const
{
    return sizeof(LvxFilePublicHeader);
}

uint64_t LvxFileHandler::dataStartOffset() const
{
    if (m_fileVer == LVX_FILE_V1) {
        return (sizeof(LvxFilePublicHeader) + sizeof(LvxFilePrivateHeader) +
                sizeof(LvxFileDeviceInfo) * m_privateHeader.deviceCount);
    } else {
        return (sizeof(LvxFilePublicHeader) + sizeof(LvxFilePrivateHeaderV0) +
                sizeof(LvxFileDeviceInfoV0) * m_privateHeaderv0.deviceCount);
    }
}

bool LvxFileHandler::addAndCheckDeviceInfo()
{
    m_lvxFile.seekg(privateHeaderOffset(), std::ios::beg);

    if (m_fileVer == LVX_FILE_V1) {
        m_lvxFile.read(reinterpret_cast<char*>(&m_privateHeader), sizeof(m_privateHeader));
        m_deviceCount = m_privateHeader.deviceCount;
    } else {
        m_lvxFile.read(reinterpret_cast<char*>(&m_privateHeaderv0), sizeof(m_privateHeaderv0));
        m_deviceCount = m_privateHeaderv0.deviceCount;
    }

    if (!m_deviceCount) {
        return false;
    }

    for (int i = 0; i < m_deviceCount; i++) {
        LvxFileDeviceInfo device_info;
        if (m_fileVer == LVX_FILE_V1) {
            m_lvxFile.read(reinterpret_cast<char*>(&device_info), sizeof(LvxFileDeviceInfo));
        } else {  // device info v0 to v1
            LvxFileDeviceInfoV0 device_info_v0;
            m_lvxFile.read(reinterpret_cast<char*>(&device_info_v0), sizeof(LvxFileDeviceInfoV0));
            memcpy(reinterpret_cast<void*>(&device_info), reinterpret_cast<void*>(&device_info_v0),
                   &device_info.extrinsicEnable - device_info.lidarBroadcastCode);
            memcpy(reinterpret_cast<void*>(&device_info.roll), reinterpret_cast<void*>(&device_info_v0.roll),
                   sizeof(float) * 6);
            device_info.extrinsicEnable = 0;
        }
        addDeviceInfo(device_info);
    }

    return true;
}

bool LvxFileHandler::prepareDataRead()
{
    m_lvxFile.seekg(dataStartOffset(), std::ios::beg);

    FrameHeader frameHeader;  // v0 & v1 compatible
    m_lvxFile.read(reinterpret_cast<char*>(&frameHeader), sizeof(frameHeader));

    if ((frameHeader.currentOffset != dataStartOffset()) || (frameHeader.frameIndex != 0)) {
        return false;
    }

    // reset the read position to the start offset of data erea
    m_lvxFile.seekg(dataStartOffset(), std::ios::beg);

    return true;
}

int LvxFileHandler::open(const char* fileName, const std::ios_base::openmode& mode)
{
    if ((mode & std::ios::in) == std::ios::in) {
        m_state = LVX_FILE_OK;
        m_lvxFile.open(fileName, mode | std::ios_base::binary | std::ios_base::ate);

        if (!m_lvxFile.is_open()) {
            m_state = LVX_FILE_NOT_EXIST;
            return m_state;
        }

        m_size = m_lvxFile.tellg();
        m_lvxFile.seekg(0, std::ios::beg);
        DEBUG_LOG("Filesize %lu\n", m_size);

        if (m_size < this->miniFileSize()) {
            m_state = LVX_FILE_SIZE_FAULT;

            return m_state;
        }

        if (!this->readAndCheckHeader()) {
            m_state = LVX_FILE_HEADER_FAULT;
            return m_state;
        }

        if (!this->addAndCheckDeviceInfo()) {
            m_state = LVX_FILE_DEVICE_INFO_FAULT;
            return m_state;
        }

        if (!this->prepareDataRead()) {
            m_state = LVX_FILE_DATA_INFO_FAULT;
            return m_state;
        }
    } else {
        m_lvxFile.open(fileName, mode | std::ios_base::binary);

        if (!m_lvxFile.is_open()) {
            m_state = LVX_FILE_NOT_EXIST;
            return m_state;
        }
    }

    return m_state;
}

bool LvxFileHandler::eof() const
{
    return m_lvxFile.eof();
}

int LvxFileHandler::initLvxFile()
{
    time_t curTime = time(nullptr);
    char fileName[30] = {0};

    struct tm localTime;
    localtime_r(&curTime, &localTime);

    strftime(fileName, sizeof(fileName), "%Y%m%d%H%M%S", &localTime);

    return open(fileName, std::ios::out | std::ios::binary);
}

void LvxFileHandler::initLvxFileHeader()
{
    char write_buffer[MAX_LVX_FILE_HEADER_LENGTH];
    m_curOffset = 0;

    std::string signature = LVX_HEADER_SIG_STR;
    memcpy(m_publicHeader.signature, signature.c_str(), signature.size());
    m_publicHeader.version[0] = 1;
    m_publicHeader.version[1] = m_fileVer;  // default version 1
    m_publicHeader.version[2] = 0;
    m_publicHeader.version[3] = 0;
    m_publicHeader.magicCode = LVX_HEADER_MAGIC_CODE;
    memcpy(&write_buffer[m_curOffset], reinterpret_cast<void*>(&m_publicHeader), sizeof(m_publicHeader));
    m_curOffset += sizeof(m_publicHeader);

    if (m_fileVer == LVX_FILE_V1) {
        m_privateHeader.deviceCount = static_cast<uint8_t>(m_deviceInfoList.size());
        m_privateHeader.frameDuration = m_frameDuration;
        m_deviceCount = m_privateHeader.deviceCount;
        memcpy(&write_buffer[m_curOffset], reinterpret_cast<void*>(&m_privateHeader), sizeof(m_privateHeader));
        m_curOffset += sizeof(m_privateHeader);
    } else {
        m_privateHeaderv0.deviceCount = static_cast<uint8_t>(m_deviceInfoList.size());
        m_deviceCount = m_privateHeaderv0.deviceCount;
        memcpy(&write_buffer[m_curOffset], reinterpret_cast<void*>(&m_privateHeaderv0), sizeof(m_privateHeaderv0));
        m_curOffset += sizeof(m_privateHeaderv0);
    }

    for (int i = 0; i < m_deviceCount; i++) {
        if (m_fileVer == LVX_FILE_V1) {
            memcpy(&write_buffer[m_curOffset], reinterpret_cast<void*>(&m_deviceInfoList[i]),
                   sizeof(LvxFileDeviceInfo));
            m_curOffset += sizeof(LvxFileDeviceInfo);
        } else {
            LvxFileDeviceInfoV0 device_info_v0;
            memcpy(reinterpret_cast<void*>(&device_info_v0), reinterpret_cast<void*>(&m_deviceInfoList[i]),
                   &m_deviceInfoList[i].extrinsicEnable - m_deviceInfoList[i].lidarBroadcastCode);
            memcpy(reinterpret_cast<void*>(&device_info_v0.roll), reinterpret_cast<void*>(&m_deviceInfoList[i].roll),
                   sizeof(float) * 6);
            memcpy(reinterpret_cast<void*>(&write_buffer[m_curOffset]), reinterpret_cast<void*>(&device_info_v0),
                   sizeof(device_info_v0));
            m_curOffset += sizeof(device_info_v0);
        }
    }

    m_lvxFile.write(&write_buffer[m_curOffset], m_curOffset);
}

void LvxFileHandler::saveFrameToLvxFile(const std::list<LvxFilePacket>& pointPacketList)
{
    uint64_t cur_pos = 0;
    FrameHeader frameHeader = {0};
    std::unique_ptr<char[]> write_buffer(new char[MAX_FRAME_SIZE]);

    frameHeader.currentOffset = m_curOffset;
    frameHeader.nextOffset = m_curOffset + sizeof(FrameHeader);
    for (auto iter : pointPacketList) {
        frameHeader.nextOffset += iter.packSize;
    }

    frameHeader.frameIndex = m_curFrameIndex;

    memcpy(write_buffer.get() + cur_pos, reinterpret_cast<void*>(&frameHeader), sizeof(FrameHeader));
    cur_pos += sizeof(FrameHeader);

    for (auto iter : pointPacketList) {
        if (cur_pos + iter.packSize >= MAX_FRAME_SIZE) {
            m_lvxFile.write(reinterpret_cast<char*>(write_buffer.get()), cur_pos);
            cur_pos = 0;
            memcpy(write_buffer.get() + cur_pos, reinterpret_cast<void*>(&iter), iter.packSize);
            cur_pos += iter.packSize;
        } else {
            memcpy(write_buffer.get() + cur_pos, reinterpret_cast<void*>(&iter), iter.packSize);
            cur_pos += iter.packSize;
        }
    }
    m_lvxFile.write(reinterpret_cast<char*>(write_buffer.get()), cur_pos);

    m_curOffset = frameHeader.nextOffset;
    m_curFrameIndex++;
}

void LvxFileHandler::closeLvxFile()
{
    if (m_lvxFile && m_lvxFile.is_open()) {
        m_lvxFile.close();
    }
}

void LvxFileHandler::handleBasePoints(LivoxEthPacket* data, LvxFilePacket& packet)
{
    memcpy(reinterpret_cast<void*>(&packet), reinterpret_cast<void*>(data), getEthPacketLen(data->data_type));
}

int LvxFileHandler::getDeviceInfo(const uint8_t idx, LvxFileDeviceInfo* info)
{
    if (idx < m_deviceInfoList.size()) {
        *info = m_deviceInfoList[idx];
        return 0;
    }

    return -1;
}

int LvxFileHandler::getPacketsOfFrame(OutPacketBuffer* packetsOfFrame)
{
    if (!m_lvxFile || m_lvxFile.eof()) {
        m_state = LVX_FILE_AT_END;
        return LVX_FILE_AT_END;
    }

    uint64_t tmpSize = m_lvxFile.tellg();
    if (tmpSize >= m_size) {
        DEBUG_LOG("At the file end %lu\n", tmpSize);
        m_state = LVX_FILE_AT_END;
        return LVX_FILE_AT_END;
    }

    FrameHeader frameHeader;
    FrameHeaderV0 frameHeaderV0;
    uint64_t readLength;
    if (m_fileVer == LVX_FILE_V1) {
        m_lvxFile.read(reinterpret_cast<char*>(&frameHeader), sizeof(frameHeader));
        if (!m_lvxFile) {
            return LVX_FILE_READ_FAIL;
        }
        if ((m_size < frameHeader.currentOffset) || (frameHeader.nextOffset < frameHeader.currentOffset)) {
            return LVX_FILE_FRAME_HEADER_ERROR;
        }
        packetsOfFrame->dataSize = dataSizeOfFrame(frameHeader);
        readLength = packetsOfFrame->dataSize;
    } else {
        m_lvxFile.read(reinterpret_cast<char*>(&frameHeaderV0), sizeof(frameHeaderV0));
        if (!m_lvxFile) {
            return LVX_FILE_READ_FAIL;
        }
        if ((m_size < frameHeaderV0.currentOffset) || (frameHeaderV0.nextOffset < frameHeaderV0.currentOffset)) {
            return LVX_FILE_FRAME_HEADER_ERROR;
        }
        packetsOfFrame->dataSize = dataSizeOfFrame(frameHeaderV0);
        readLength = packetsOfFrame->dataSize;
    }
    m_lvxFile.read(reinterpret_cast<char*>(packetsOfFrame->packet), readLength);
    if (m_lvxFile) {
        return LVX_FILE_OK;
    } else {
        return LVX_FILE_READ_FAIL;
    }
}

int LvxFileHandler::getLvxFileReadProgress()
{
    if (!m_size) {
        return 0;
    }

    if (!m_lvxFile.eof()) {
        return (m_lvxFile.tellg() * 100ULL) / m_size;
    } else {
        return 100;
    }
}

void parseExtrinsicXml(const DeviceItem& item, LvxFileDeviceInfo& info)
{
    rapidxml::file<> extrinsic_param("extrinsic.xml");
    rapidxml::xml_document<> doc;
    doc.parse<0>(extrinsic_param.data());
    rapidxml::xml_node<>* root = doc.first_node();
    if ("Livox" == (std::string)root->name()) {
        for (rapidxml::xml_node<>* device = root->first_node(); device; device = device->next_sibling()) {
            if ("Device" == (std::string)device->name() &&
                (strncmp(item.info.broadcast_code, device->value(), kBroadcastCodeSize) == 0)) {
                memcpy(info.lidarBroadcastCode, device->value(), kBroadcastCodeSize);
                memset(info.hubBroadcastCode, 0, kBroadcastCodeSize);
                info.deviceType = item.info.type;
                info.deviceIndex = item.handle;
                for (rapidxml::xml_attribute<>* param = device->first_attribute(); param;
                     param = param->next_attribute()) {
                    if ("roll" == (std::string)param->name()) {
                        info.roll = static_cast<float>(atof(param->value()));
                    }
                    if ("pitch" == (std::string)param->name()) {
                        info.pitch = static_cast<float>(atof(param->value()));
                    }
                    if ("yaw" == (std::string)param->name()) {
                        info.yaw = static_cast<float>(atof(param->value()));
                    }
                    if ("x" == (std::string)param->name()) {
                        info.x = static_cast<float>(atof(param->value()));
                    }
                    if ("y" == (std::string)param->name()) {
                        info.y = static_cast<float>(atof(param->value()));
                    }
                    if ("z" == (std::string)param->name()) {
                        info.z = static_cast<float>(atof(param->value()));
                    }
                }
            }
        }
    }
}
}  // namespace lvx
