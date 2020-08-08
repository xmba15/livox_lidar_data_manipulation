/**
 * @file    Utility.hpp
 *
 * @author  livox sdk, btran
 *
 * Copyright (c) organization
 *
 */

#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

#include "Constants.hpp"

#ifdef DEBUG
#define ENABLE_DEBUG 1
#include <iostream>
#else
#define ENABLE_DEBUG 0
#endif

namespace lvx
{
#if ENABLE_DEBUG
#define DEBUG_LOG(...)                                                                                                 \
    {                                                                                                                  \
        char str[100];                                                                                                 \
        snprintf(str, sizeof(str), __VA_ARGS__);                                                                       \
        std::cout << "[" << __FILE__ << "][" << __FUNCTION__ << "][Line " << __LINE__ << "] >>> " << str << std::endl; \
    }
#else
#define DEBUG_LOG(...)
#endif

inline static bool isPowerOf2(const uint32_t val)
{
    return (val != 0) && ((val & (val - 1)) == 0);
}

inline static uint32_t roundupPowerOf2(const uint32_t val)
{
    uint32_t power2Val = 0;
    for (int i = 0; i < 32; i++) {
        power2Val = ((uint32_t)1) << i;
        if (val <= power2Val) {
            break;
        }
    }

    return power2Val;
}

inline bool isFilePathValid(const char* pathStr)
{
    int str_len = strlen(pathStr);

    if ((str_len > PATH_STR_MIN_SIZE) && (str_len < PATH_STR_MAX_SIZE)) {
        return true;
    } else {
        return false;
    }
}

/**
 *  @brief color coding skeme
 *
 *  Ref: https://github.com/Livox-SDK/Livox-SDK/wiki/Livox-Viewer#1-color-coding-strategy
 */
inline std::array<uint8_t, 3> colorCodingReflectivityRGB(const float reflectivity)
{
    uint8_t r, g, b;

    if (reflectivity < 30) {
        r = 0;
        g = static_cast<uint8_t>(reflectivity * 255 / 30) & 0xff;
        b = 0xff;
    } else if (reflectivity < 90) {
        r = 0;
        g = 0xff;
        b = static_cast<uint8_t>((90 - reflectivity) * 255 / 60) & 0xff;
    } else if (reflectivity < 150) {
        r = static_cast<uint8_t>((reflectivity - 90) * 255 / 60) & 0xff;
        g = 0xff;
        b = 0;
    } else {
        r = 0xff;
        g = static_cast<uint8_t>((255 - reflectivity) * 255 / (256 - 150)) & 0xff;
        b = 0;
    }

    return std::array<uint8_t, 3>{r, g, b};
}

inline std::array<uint8_t, 3> colorCodingDepthRGB(const float depth)
{
    uint8_t r, g, b;

    if (depth < 0) {
        r = 0;
        g = 0;
        b = 0;
    } else if (depth < 50) {  //  0~100m
        r = 0;
        g = static_cast<uint8_t>((depth - 0) / 50 * 255) & 0xff;
        b = 0xff;
    } else if (depth < 120) {  //  100~200m
        r = 0;
        g = 0xff;
        b = 0xff - (static_cast<uint8_t>((depth - 50) / 70 * 255) & 0xff);
    } else if (depth < 200) {  //  200~300m
        r = static_cast<uint8_t>((depth - 120) / 80 * 255) & 0xff;
        g = 0xff;
        b = 0;
    } else if (depth < 350) {  //  300~400m
        r = 0xff;
        g = 0xff - (static_cast<uint8_t>((depth - 200) / 150 * 255) & 0xff);
        b = 0;
    } else if (depth < 500) {  //  400~500m
        r = 0xff;
        g = 0x00;
        b = static_cast<uint8_t>((depth - 350) / 150 * 255) & 0xff;
    } else {  //  500m+
        r = 0xff;
        g = 0;
        b = 0xff;
    }

    return std::array<uint8_t, 3>{r, g, b};
}

template <typename T>
bool almostEquals(const T val, const T correctVal, const T epsilon = std::numeric_limits<T>::epsilon())
{
    const T maxXYOne = std::max({static_cast<T>(1.0f), std::fabs(val), std::fabs(correctVal)});
    return std::fabs(val - correctVal) <= epsilon * maxXYOne;
}

inline double deg2Rad(const double deg)
{
    return deg * M_PI / 180.;
}

inline std::vector<std::string> split(const std::string& s, const char delimiter)
{
    std::stringstream ss(s);
    std::string token;
    std::vector<std::string> tokens;
    while (std::getline(ss, token, delimiter)) {
        tokens.emplace_back(token);
    }
    return tokens;
}
}  // namespace lvx
