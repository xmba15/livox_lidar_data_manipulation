/**
 * @file    ldq.hpp
 *
 * @brief   livox lidar data queue
 *
 * @author  btran
 *
 * @date    2020-04-03
 *
 * Copyright (c) organization
 *
 */

#pragma once
#ifndef LDQ_HPP_
#define LDQ_HPP_

#include <cstdint>

namespace lvx
{
static constexpr uint32_t KEthPacketMaxLength = 1500;

#pragma pack(1)

typedef struct {
    uint64_t time_rcv; /**< receive time when data arrive */
    uint32_t point_num;
    uint8_t raw_data[KEthPacketMaxLength];
} StoragePacket;

#pragma pack()

typedef struct {
    StoragePacket* storage_packet;
    volatile uint32_t rd_idx;
    volatile uint32_t wr_idx;
    uint32_t mask;
    uint32_t size; /**< must be power of 2. */
} LidarDataQueue;

inline static bool IsPowerOf2(uint32_t size)
{
    return (size != 0) && ((size & (size - 1)) == 0);
}

inline static uint32_t RoundupPowerOf2(uint32_t size)
{
    uint32_t power2_val = 0;
    for (int i = 0; i < 32; i++) {
        power2_val = ((uint32_t)1) << i;
        if (size <= power2_val) {
            break;
        }
    }

    return power2_val;
}

/** queue operate function */
int InitQueue(LidarDataQueue* queue, uint32_t queue_size);
int DeInitQueue(LidarDataQueue* queue);
void ResetQueue(LidarDataQueue* queue);
void QueueProPop(LidarDataQueue* queue, StoragePacket* storage_packet);
void QueuePopUpdate(LidarDataQueue* queue);
uint32_t QueuePop(LidarDataQueue* queue, StoragePacket* storage_packet);
uint32_t QueueUsedSize(LidarDataQueue* queue);
uint32_t QueueUnusedSize(LidarDataQueue* queue);
uint32_t QueueIsFull(LidarDataQueue* queue);
uint32_t QueueIsEmpty(LidarDataQueue* queue);
uint32_t QueuePush(LidarDataQueue* queue, StoragePacket* storage_packet);
uint32_t QueuePushAny(LidarDataQueue* queue, uint8_t* data, uint32_t length, uint64_t time_rcv, uint32_t point_num);

}  // namespace lvx
#endif  // LDQ_HPP_
