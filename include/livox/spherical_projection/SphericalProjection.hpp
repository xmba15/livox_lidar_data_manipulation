/**
 * @file    SphericalProjection.hpp
 *
 * @author  btran
 *
 * Copyright (c) organization
 *
 */

#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "../Utility.hpp"

namespace perception
{
class SphericalProjectionParam
{
 public:
    double fovUpDeg = 25.1 / 2;

    double fovDownDeg = -25.1 / 2;

    double fovRightDeg = 81.7 / 2;

    double fovLeftDeg = -81.7 / 2;

    double horizontalAngularResolutionDeg = 0.2;

    // number of firing laser, equal to imgHeight
    std::size_t numLaser = 128;
};

template <typename POINT_CLOUD_TYPE, typename VEC_POINT_TYPE> class SphericalProjection
{
 public:
    using PointCloudType = POINT_CLOUD_TYPE;
    using VecPointType = VEC_POINT_TYPE;

    struct SphericalImage;

    // ProjectionInfo->(3d distance, x, y in image coordinate)
    using ProjectionInfo = std::array<double, 3>;

    using Ptr = std::shared_ptr<SphericalProjection>;

    struct SphericalImage {
        SphericalImage(const std::size_t height, const std::size_t width);

        std::size_t height;
        std::size_t width;
        std::vector<std::vector<std::vector<double>>> data;
    };

 public:
    explicit SphericalProjection(const SphericalProjectionParam& param);
    virtual ~SphericalProjection() = default;

    ProjectionInfo project2D(const PointCloudType& point) const;
    SphericalImage projectSphericalImage(const VecPointType& pointVec) const;

    std::size_t imgHeight() const;
    std::size_t imgWidth() const;

 private:
    void updateInternalParams();

 private:
    SphericalProjectionParam m_param;
    double m_fovVerticalRad;
    double m_fovHorizontalRad;
    double m_fovUpRad;
    double m_fovRightRad;
    std::size_t m_imgHeight;
    std::size_t m_imgWidth;
};
}  // namespace perception
#include "impl/SphericalProjection.ipp"
