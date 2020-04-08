/**
 * @file    main.cpp
 *
 * @brief   simple app to convert lvx file to pcd frames
 *
 * @author  btran
 *
 * Copyright (c) organization
 *
 */
#include <iostream>
#include <string>

#include <livox/lvx.hpp>

#include "../shared_utilities/Utility.hpp"

int main(int argc, const char* argv[])
{
    if (argc != 4) {
        std::cout << "Usage: [apps] [path/to/lvx/file] [output/path] [publish/frequency]" << std::endl;
        return EXIT_FAILURE;
    }

    const std::string pathToLvx = argv[1];
    const std::string outputPath = argv[2];
    const double publishFreq = std::atof(argv[3]);
    int dataSrc = lvx::SOURCE_LVX_FILE;

    std::shared_ptr<lvx::LidarDataDistributeController> lddcPtr =
        std::make_shared<lvx::LidarDataDistributeController>(lvx::LidarDataDistributeController(dataSrc, publishFreq));

    const int intervalMs = 1000 / publishFreq;
    lvx::LidarDataSource* lvxInstance = lvx::LdsLvx::getInstance(intervalMs);

    lddcPtr->registerLds(lvxInstance);

    if (!lvx::LidarDataDistributeController::createOutputDir(outputPath)) {
        std::cerr << "Failed to create " << outputPath << "\n";
        return EXIT_FAILURE;
    }

    lddcPtr->m_lds->initSource({pathToLvx.c_str()});

    while (lvx::writeToPcd(lddcPtr, outputPath)) {
    }

    return EXIT_SUCCESS;
}
