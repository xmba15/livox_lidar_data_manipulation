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

inline cv::Mat toVisualCvMat(const SphericalProjection::SphericalImage& sphericalImage);
inline std::vector<std::string> parseMetaDataFile(const std::string& metaDataFilePath);
}  // namespace

int main(int argc, char* argv[])
{
    if (argc != 3) {
        std::cerr << "Usage: [app] [path/to/pcl/file/list] [path/to/pcd/data]\n";
        return EXIT_FAILURE;
    }

    const std::string pclListFile = argv[1];
    const std::string dataPath = argv[2];

    std::vector<std::string> pcdPaths = ::parseMetaDataFile(pclListFile);
    std::for_each(pcdPaths.begin(), pcdPaths.end(), [&dataPath](std::string& path) { path = dataPath + "/" + path; });

    perception::SphericalProjectionParam sphericalProjectionParam;
    SphericalProjection::Ptr sphericalProjector = std::make_shared<SphericalProjection>(sphericalProjectionParam);

    cv::VideoWriter video("output.avi", cv::VideoWriter::fourcc('X', 'V', 'I', 'D'), 10 /* fps */,
                          cv::Size(sphericalProjector->imgWidth(), sphericalProjector->imgHeight()));

    for (const std::string& pclFilePath : pcdPaths) {
        PointCloudPtr inputPcl(new PointCloud);
        if (pcl::io::loadPCDFile(pclFilePath, *inputPcl) == -1) {
            std::cerr << "Failed to load pcl file\n";
            return EXIT_FAILURE;
        }

        SphericalProjection::SphericalImage sphericalImage =
            sphericalProjector->projectSphericalImage(inputPcl->points);

        cv::Mat outputImage = ::toVisualCvMat(sphericalImage);
        cv::imshow("Intensity Image", outputImage);
        video.write(outputImage);

        if (static_cast<char>(cv::waitKey(25)) == 27) {
            break;
        }
    }

    cv::destroyAllWindows();
    video.release();

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

std::vector<std::string> parseMetaDataFile(const std::string& metaDataFilePath)
{
    std::ifstream inFile;
    inFile.open(metaDataFilePath);
    if (!inFile) {
        throw std::runtime_error("Unable to open " + metaDataFilePath + "\n");
    }

    std::stringstream buffer;
    buffer << inFile.rdbuf();

    return lvx::split(buffer.str(), '\n');
}
}  // namespace
