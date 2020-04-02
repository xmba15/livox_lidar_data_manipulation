/**
 * @file    ldq.cpp
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

#include <cstring>
#include <lvx_to_pcd/ldq.hpp>

namespace lvx
{
int InitQueue(LidarDataQueue* queue, uint32_t queue_size)
{
    if (queue == nullptr) {
        return 1;
    }

    if (IsPowerOf2(queue_size) != true) {
        queue_size = RoundupPowerOf2(queue_size);
    }

    queue->storage_packet = new StoragePacket[queue_size];
    if (queue->storage_packet == nullptr) {
        return 1;
    }

    queue->rd_idx = 0;
    queue->wr_idx = 0;
    queue->size = queue_size;
    queue->mask = queue_size - 1;

    return 0;
}

int DeInitQueue(LidarDataQueue* queue)
{
    if (queue == nullptr) {
        return 1;
    }

    if (queue->storage_packet) {
        delete[] queue->storage_packet;
    }

    queue->rd_idx = 0;
    queue->wr_idx = 0;
    queue->size = 0;
    queue->mask = 0;

    return 0;
}

void ResetQueue(LidarDataQueue* queue)
{
    queue->rd_idx = 0;
    queue->wr_idx = 0;
}

void QueueProPop(LidarDataQueue* queue, StoragePacket* storage_packet)
{
    uint32_t rd_idx = queue->rd_idx & queue->mask;

    memcpy(storage_packet, &(queue->storage_packet[rd_idx]), sizeof(StoragePacket));
}

void QueuePopUpdate(LidarDataQueue* queue)
{
    queue->rd_idx++;
}

uint32_t QueuePop(LidarDataQueue* queue, StoragePacket* storage_packet)
{
    QueueProPop(queue, storage_packet);
    QueuePopUpdate(queue);

    return 1;
}

uint32_t QueueUsedSize(LidarDataQueue* queue)
{
    return queue->wr_idx - queue->rd_idx;
}

uint32_t QueueUnusedSize(LidarDataQueue* queue)
{
    return (queue->size - (queue->wr_idx - queue->rd_idx));
}

uint32_t QueueIsFull(LidarDataQueue* queue)
{
    return ((queue->wr_idx - queue->rd_idx) > queue->mask);
}

uint32_t QueueIsEmpty(LidarDataQueue* queue)
{
    return (queue->rd_idx == queue->wr_idx);
}

uint32_t QueuePush(LidarDataQueue* queue, StoragePacket* storage_packet)
{
    uint32_t wr_idx = queue->wr_idx & queue->mask;

    memcpy((void*)(&(queue->storage_packet[wr_idx])), (void*)(storage_packet), sizeof(StoragePacket));

    queue->wr_idx++;

    return 1;
}

uint32_t QueuePushAny(LidarDataQueue* queue, uint8_t* data, uint32_t length, uint64_t time_rcv, uint32_t point_num)
{
    uint32_t wr_idx = queue->wr_idx & queue->mask;

    queue->storage_packet[wr_idx].time_rcv = time_rcv;
    queue->storage_packet[wr_idx].point_num = point_num;
    memcpy(queue->storage_packet[wr_idx].raw_data, data, length);

    queue->wr_idx++;

    return 1;
}
}  // namespace lvx
