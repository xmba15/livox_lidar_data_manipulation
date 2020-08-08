/**
 * @file    main.cpp
 *
 * @author  btran
 *
 * Copyright (c) organization
 *
 */

#include <opencv2/opencv.hpp>

#include <pcl/io/pcd_io.h>

#include <livox/lvx.hpp>

namespace
{
using PointCloudType = pcl::PointXYZI;
using PointCloud = pcl::PointCloud<PointCloudType>;
using PointCloudPtr = typename PointCloud::Ptr;
using VecPointType = std::vector<PointCloudType, Eigen::aligned_allocator<PointCloudType>>;
using SphericalProjection = perception::SphericalProjection<PointCloudType, VecPointType>;

cv::Mat toVisualCvMat(const SphericalProjection::SphericalImage& sphericalImage);
}  // namespace

int main(int argc, char* argv[])
{
    if (argc != 2) {
        std::cerr << "Usage: [app] [path/to/pcl/file]\n";
        return EXIT_FAILURE;
    }

    const std::string pclFilePath = argv[1];

    perception::SphericalProjectionParam sphericalProjectionParam;
    SphericalProjection::Ptr sphericalProjector = std::make_shared<SphericalProjection>(sphericalProjectionParam);

    PointCloudPtr inputPcl(new PointCloud);
    if (pcl::io::loadPCDFile(pclFilePath, *inputPcl) == -1) {
        std::cerr << "Failed to load pcl file\n";
        return EXIT_FAILURE;
    }

    SphericalProjection::SphericalImage sphericalImage = sphericalProjector->projectSphericalImage(inputPcl->points);

    cv::Mat ouputImage = toVisualCvMat(sphericalImage);
    cv::imwrite("output.png", ouputImage);
    cv::imshow("Intensity Image", ouputImage);
    cv::waitKey(0);
    cv::destroyAllWindows();

    return EXIT_SUCCESS;
}

namespace
{
cv::Mat toVisualCvMat(const SphericalProjection::SphericalImage& sphericalImage)
{
    cv::Mat output(cv::Size(sphericalImage.width, sphericalImage.height), CV_8UC3, cv::Scalar(0, 0, 0));
    for (int i = 0; i < output.rows; ++i) {
        for (int j = 0; j < output.cols; ++j) {
            // if depth is 0 then colorize the point as (0,0,0)
            std::array<uint8_t, 3> curColor = static_cast<int>(sphericalImage.data[i][j][3]) == 0
                                                  ? std::array<uint8_t, 3>{0, 0, 0}
                                                  : lvx::colorCodingReflectivityRGB(sphericalImage.data[i][j][0]);

            output.at<cv::Vec3b>(i, j)[0] = curColor[0];
            output.at<cv::Vec3b>(i, j)[1] = curColor[1];
            output.at<cv::Vec3b>(i, j)[2] = curColor[2];
        }
    }

    return output;
}
}  // namespace
