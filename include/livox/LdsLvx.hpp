/**
 * @file    LdsLvx.hpp
 *
 * @brief   livox file data source
 *
 * @author  livox sdk, btran
 *
 * Copyright (c) organization
 *
 */

#pragma once

#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "LidarDataSource.hpp"
#include "LvxFileHandler.hpp"

namespace lvx
{
class LdsLvx : public LidarDataSource
{
 public:
    static LdsLvx* getInstance(const uint32_t intervalMs)
    {
        static LdsLvx ldsLvx(intervalMs);
        return &ldsLvx;
    }

    int initSource(const std::vector<std::any>& fileSource) override;

    void prepareExit(void) override;

 private:
    explicit LdsLvx(const uint32_t intervalMs);

    LdsLvx(const LdsLvx&) = delete;

    ~LdsLvx();

    LdsLvx& operator=(const LdsLvx&) = delete;

    int initLdsLvx(const char* lvxPath);

    void startRead()
    {
        m_startReadLvx = true;
    }

    void stopRead()
    {
        m_startReadLvx = false;
    }

    bool isStarted()
    {
        return m_startReadLvx;
    }

    void readLvxFile();

    bool isAllQueueEmpty();

    bool isAllQueueReadStop();

 private:
    volatile bool m_isInitialized;

    OutPacketBuffer m_packetsOfFrame;

    std::shared_ptr<LvxFileHandler> m_lvxFile;

    std::shared_ptr<std::thread> m_threadReadLvx;

    volatile bool m_startReadLvx;

    uint32_t m_remainSize[MAX_SOURCE_LIDAR];
};
}  // namespace lvx
