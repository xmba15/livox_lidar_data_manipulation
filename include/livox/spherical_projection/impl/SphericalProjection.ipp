/**
 * @file    SphericalProjection.ipp
 *
 * @author  btran
 *
 */

namespace perception
{
template <typename POINT_CLOUD_TYPE, typename VEC_POINT_TYPE>
SphericalProjection<POINT_CLOUD_TYPE, VEC_POINT_TYPE>::SphericalImage::SphericalImage(const std::size_t height,
                                                                                      const std::size_t width)
    : height(height), width(width),
      data(std::vector(height, std::vector<std::vector<double>>(width, std::vector<double>(5, 0.0))))
{
}

template <typename POINT_CLOUD_TYPE, typename VEC_POINT_TYPE>
SphericalProjection<POINT_CLOUD_TYPE, VEC_POINT_TYPE>::SphericalProjection(const SphericalProjectionParam& param)
    : m_param(param)
{
    this->updateInternalParams();
}

template <typename POINT_CLOUD_TYPE, typename VEC_POINT_TYPE>
typename SphericalProjection<POINT_CLOUD_TYPE, VEC_POINT_TYPE>::ProjectionInfo
SphericalProjection<POINT_CLOUD_TYPE, VEC_POINT_TYPE>::project2D(const POINT_CLOUD_TYPE& point) const
{
    double distanceToPoint = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
    if (lvx::almostEquals<double>(distanceToPoint, 0.0) || lvx::almostEquals<double>(point.x, 0.0)) {
        return {0, 0, 0};
    }

    double yaw = std::atan2(point.y, point.x);
    double pitch = std::asin(point.z / distanceToPoint);

    // camera coordinate with origin in the top left
    double camCoordYaw = m_fovRightRad + yaw;
    double camCoordPitch = m_fovUpRad - pitch;

    // normalize coordinates
    double normalizedYaw = camCoordYaw / m_fovHorizontalRad;
    double normalizedPitch = camCoordPitch / m_fovVerticalRad;

    // scaling coordinates
    double xCoord = static_cast<int>(m_imgWidth * normalizedYaw);
    double yCoord = static_cast<int>(m_imgHeight * normalizedPitch);

    auto correctCoordinate = [](double& coord, const double maxVal) {
        coord = std::max(coord, 0.0);
        coord = std::min(coord, maxVal);
    };
    correctCoordinate(xCoord, m_imgWidth - 1);
    correctCoordinate(yCoord, m_imgHeight - 1);

    return {distanceToPoint, xCoord, yCoord};
}

template <typename POINT_CLOUD_TYPE, typename VEC_POINT_TYPE>
typename SphericalProjection<POINT_CLOUD_TYPE, VEC_POINT_TYPE>::SphericalImage
SphericalProjection<POINT_CLOUD_TYPE, VEC_POINT_TYPE>::projectSphericalImage(const VEC_POINT_TYPE& pointVec) const
{
    SphericalImage sphericalImage(m_imgHeight, m_imgWidth);

    for (const auto& curPoint : pointVec) {
        ProjectionInfo curProjection = this->project2D(curPoint);
        int curY = static_cast<int>(curProjection[2]);
        int curX = static_cast<int>(curProjection[1]);

        sphericalImage.data[curY][curX] =
            std::vector<double>{curPoint.x, curPoint.y, curPoint.z, curProjection[0], curPoint.intensity};
    }

    return sphericalImage;
}

template <typename POINT_CLOUD_TYPE, typename VEC_POINT_TYPE>
void SphericalProjection<POINT_CLOUD_TYPE, VEC_POINT_TYPE>::updateInternalParams()
{
    m_fovUpRad = lvx::deg2Rad(m_param.fovUpDeg);
    m_fovRightRad = lvx::deg2Rad(m_param.fovRightDeg);
    m_fovVerticalRad = lvx::deg2Rad(m_param.fovUpDeg - m_param.fovDownDeg);
    double fovHorizontalDeg = m_param.fovRightDeg - m_param.fovLeftDeg;
    m_fovHorizontalRad = lvx::deg2Rad(fovHorizontalDeg);
    m_imgHeight = m_param.numLaser;
    m_imgWidth = static_cast<std::size_t>(fovHorizontalDeg / m_param.horizontalAngularResolutionDeg / 32) * 32;
}
}  // namespace perception
