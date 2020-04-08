/**
 * @file    LidarDataQueue.hpp
 *
 * @brief   livox lidar data queue
 *
 * @author  livox sdk, btran
 *
 * Copyright (c) organization
 *
 */

#pragma once

#include <cstdint>

#include "Constants.hpp"
#include "Types.hpp"

namespace lvx
{
namespace queue_util
{
int initQueue(LidarDataQueue* queue, uint32_t queueSize);

int deInitQueue(LidarDataQueue* queue);

void resetQueue(LidarDataQueue* queue);

void queueProPop(LidarDataQueue* const queue, StoragePacket* storagePacket);

void queuePopUpdate(LidarDataQueue* queue);

uint32_t queuePop(LidarDataQueue* queue, StoragePacket* storagePacket);

uint32_t queueUsedSize(LidarDataQueue* const queue);

uint32_t queueUnusedSize(LidarDataQueue* const queue);

uint32_t queueIsFull(LidarDataQueue* const queue);

uint32_t queueIsEmpty(LidarDataQueue* const queue);

uint32_t queuePush(LidarDataQueue* queue, StoragePacket* storagePacket);

uint32_t queuePushAny(LidarDataQueue* queue, uint8_t* data, const uint32_t length, const uint64_t timeRcv,
                      const uint32_t pointNum);
}  // namespace queue_util
}  // namespace lvx
