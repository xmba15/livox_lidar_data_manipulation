/**
 * @file    main.cpp
 *
 * @author  btran
 *
 * Copyright (c) organization
 *
 */

#include <livox/lvx.hpp>

#include <pcl/common/common_headers.h>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/range_image/range_image.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/range_image_visualizer.h>

#include "../shared_utilities/Utility.hpp"

namespace
{
void viewerOneOff(const pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    pcl::PointXYZ o;
    o.x = 0.1;
    o.y = 0;
    o.z = 0;
    viewer->addSphere(o, 0.1, "sphere", 0);
    viewer->addCoordinateSystem(0.5);
    viewer->setCameraPosition(-26, 0, 3, 10, -1, 0.5, 0, 0, 1);
}
}  // namespace

int main(int argc, const char* argv[])
{
    if (argc != 3) {
        std::cout << "Usage: [apps] [path/to/lvx/file] [publish/frequency]" << std::endl;
        return EXIT_FAILURE;
    }

    const std::string pathToLvx = argv[1];
    const double publishFreq = std::atof(argv[2]);
    const int dataSrc = lvx::SOURCE_LVX_FILE;

    std::shared_ptr<lvx::LidarDataDistributeController> lddcPtr =
        std::make_shared<lvx::LidarDataDistributeController>(lvx::LidarDataDistributeController(dataSrc, publishFreq));

    const int intervalMs = 1000 / publishFreq;
    lvx::LidarDataSource* lvxInstance = lvx::LdsLvx::getInstance(intervalMs);

    lddcPtr->registerLds(lvxInstance);
    lddcPtr->m_lds->initSource({pathToLvx.c_str()});

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    ::viewerOneOff(viewer);

    while (lvx::visualizePointcloud(lddcPtr, *viewer)) {
    }

    return EXIT_SUCCESS;
}
