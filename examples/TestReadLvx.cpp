/**
 * @file    TestReadLvx.cpp
 *
 * @author  btran
 *
 * @date    2020-04-03
 *
 * Copyright (c) organization
 *
 */

#include <lvx_to_pcd/lvx_file.hpp>

#include <livox_def.h>
#include <livox_sdk.h>

#include <iostream>
#include <string>

int main(int argc, const char* argv[])
{
    if (argc < 2) {
        std::cout << "Usage: [apps] [path/to/lvx/file]" << std::endl;
        return EXIT_FAILURE;
    }

    const std::string pathToLvx = argv[1];

    // lvx::lvx_file_handler.InitLvxFileHeader();

    // const int intervalMs = 1000 / 100;
    // auto lvxInstance = lvx::LdsLvx::GetInstance(intervalMs);
    // int ret = lvxInstance->InitLdsLvx(pathToLvx.c_str());

    return 0;
}
