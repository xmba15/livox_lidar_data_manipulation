/**
 * @file    lds_lvx.hpp
 *
 * @brief   livox file data source
 *
 * @author  btran
 *
 * @date    2020-04-03
 *
 * Copyright (c) organization
 *
 */

#include <memory>
#include <thread>

#include "lds.hpp"
#include "lvx_file.hpp"

namespace lvx
{
/**
 * Lidar data source abstract.
 */
class LdsLvx : public Lds
{
 public:
    static LdsLvx* GetInstance(uint32_t interval_ms)
    {
        static LdsLvx lds_lvx(interval_ms);
        return &lds_lvx;
    }

    int InitLdsLvx(const char* lvx_path);
    int DeInitLdsLvx(void);
    void PrepareExit(void);

 private:
    LdsLvx(uint32_t interval_ms);
    LdsLvx(const LdsLvx&) = delete;
    ~LdsLvx();
    LdsLvx& operator=(const LdsLvx&) = delete;

    void StartRead()
    {
        start_read_lvx_ = true;
    }
    void StopRead()
    {
        start_read_lvx_ = false;
    }
    bool IsStarted()
    {
        return start_read_lvx_;
    }

    void ReadLvxFile();
    bool IsAllQueueEmpty();

    volatile bool is_initialized_;
    OutPacketBuffer packets_of_frame_;
    std::shared_ptr<LvxFileHandle> lvx_file_;
    std::shared_ptr<std::thread> t_read_lvx_;
    volatile bool start_read_lvx_;
};

}  // namespace lvx
