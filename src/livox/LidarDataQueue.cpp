/**
 * @file    LidarDataQueue.cpp
 *
 * @brief   livox lidar data queue
 *
 * @author  livox sdk, btran
 *
 * Copyright (c) organization
 *
 */

#include <livox/lvx.hpp>

#include <cstring>

namespace lvx
{
namespace queue_util
{
int initQueue(LidarDataQueue* queue, uint32_t queueSize)
{
    if (queue == nullptr) {
        return 1;
    }

    if (isPowerOf2(queueSize) != true) {
        queueSize = roundupPowerOf2(queueSize);
    }

    queue->storagePacket = new StoragePacket[queueSize];
    if (queue->storagePacket == nullptr) {
        return 1;
    }

    queue->rdIdx = 0;
    queue->wrIdx = 0;
    queue->size = queueSize;
    queue->mask = queueSize - 1;

    return 0;
}

int deInitQueue(LidarDataQueue* queue)
{
    if (queue == nullptr) {
        return 1;
    }

    if (queue->storagePacket) {
        delete[] queue->storagePacket;
    }

    queue->rdIdx = 0;
    queue->wrIdx = 0;
    queue->size = 0;
    queue->mask = 0;

    return 0;
}

void resetQueue(LidarDataQueue* queue)
{
    queue->rdIdx = 0;
    queue->wrIdx = 0;
}

void queueProPop(LidarDataQueue* const queue, StoragePacket* storagePacket)
{
    uint32_t rdIdx = queue->rdIdx & queue->mask;

    memcpy(storagePacket, &(queue->storagePacket[rdIdx]), sizeof(StoragePacket));
}

void queuePopUpdate(LidarDataQueue* queue)
{
    queue->rdIdx++;
}

uint32_t queuePop(LidarDataQueue* queue, StoragePacket* storagePacket)
{
    queueProPop(queue, storagePacket);
    queuePopUpdate(queue);

    return 1;
}

uint32_t queueUsedSize(LidarDataQueue* const queue)
{
    return queue->wrIdx - queue->rdIdx;
}

uint32_t queueUnusedSize(const LidarDataQueue* queue)
{
    return (queue->size - (queue->wrIdx - queue->rdIdx));
}

uint32_t queueIsFull(LidarDataQueue* const queue)
{
    return ((queue->wrIdx - queue->rdIdx) > queue->mask);
}

uint32_t queueIsEmpty(LidarDataQueue* const queue)
{
    return (queue->rdIdx == queue->wrIdx);
}

uint32_t queuePush(LidarDataQueue* queue, StoragePacket* storagePacket)
{
    uint32_t wrIdx = queue->wrIdx & queue->mask;

    memcpy(reinterpret_cast<void*>(&queue->storagePacket[wrIdx]), reinterpret_cast<void*>(storagePacket),
           sizeof(StoragePacket));

    queue->wrIdx++;

    return 1;
}

uint32_t queuePushAny(LidarDataQueue* queue, uint8_t* data, const uint32_t length, const uint64_t timeRcv,
                      const uint32_t pointNum)
{
    uint32_t wrIdx = queue->wrIdx & queue->mask;

    queue->storagePacket[wrIdx].timeRcv = timeRcv;
    queue->storagePacket[wrIdx].pointNum = pointNum;
    memcpy(queue->storagePacket[wrIdx].rawData, data, length);

    queue->wrIdx++;

    return 1;
}
}  // namespace queue_util
}  // namespace lvx
