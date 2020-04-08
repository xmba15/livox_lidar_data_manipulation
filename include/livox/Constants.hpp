/**
 * @file    Constants.hpp
 *
 * @author  livox sdk, btran
 *
 * Copyright (c) organization
 *
 */

#pragma once

#include <cstdint>

namespace lvx
{
static constexpr char LVX_HEADER_SIG_STR[] = "livox_tech";

static constexpr uint32_t MAX_LVX_FILE_HEADER_LENGTH = 16 * 1024;

static constexpr uint32_t LVX_HEADER_MAGIC_CODE = 0xac0ea767;

static constexpr uint32_t ETH_PACKET_MAX_LENGTH = 1500;

// Max lidar data source num
static constexpr uint32_t MAX_SOURCE_LIDAR = 32;

// Eth packet relative info parama
static constexpr uint32_t MAX_POINT_PER_ETH_PACKET = 100;

// must be 2^n
static constexpr uint32_t MIN_ETH_PACKET_QUEUE_SIZE = 32;

// must be 2^n
static constexpr uint32_t MAX_ETH_PACKET_QUEUE_SIZE = 8192;

static constexpr uint32_t IMU_ETH_PACKET_QUEUE_SIZE = 256;

// (sizeof(LivoxEthPacket) - 1)
static constexpr uint32_t ETH_PACKET_HEADER_LENGTH = 18;

static constexpr uint32_t CARTESIAN_POINT_SIZE = 13;

static constexpr uint32_t SPHERICAL_POINT_SZIE = 9;

// 1ms = 1000000ns
static constexpr int64_t PACKET_TIME_GAP = 1000000;

// the threshold of packet continuous
static constexpr int64_t MAX_PACKET_TIME_GAP = 1700000;

// the threshold of device disconect
static constexpr int64_t DEVICE_DISCONNECT_THRESHOLD = 1000000000;

// 1s  = 1000000000ns
static constexpr int64_t NS_PER_SECOND = 1000000000;

// Must more than 4 char
static constexpr int PATH_STR_MIN_SIZE = 4;

// Must less than 256 char
static constexpr int PATH_STR_MAX_SIZE = 256;

static constexpr int BD_CODE_SIZE = 15;

static constexpr uint32_t POINT_XYZR_SIZE = 16;

static constexpr uint32_t POINT_XYZRTR_SIZE = 18;

// LvXFile
static constexpr uint32_t MAX_POINT_SIZE = 1500;

static constexpr uint32_t DEFAULT_FRAME_DURATION_TIME = 50;

static constexpr uint32_t MAX_FRAME_SIZE = 2048 * 1024;

static constexpr uint32_t MAX_PACKETS_NUM_OF_FRAME = 8192;
}  // namespace lvx
