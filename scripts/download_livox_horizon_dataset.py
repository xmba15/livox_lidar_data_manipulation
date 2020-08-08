#!/usr/bin/env python
from download_utility import download_file_from_google_drive
import os
import sys


_CURRENT_DIR = os.path.dirname(os.path.realpath(__file__))


def main():
    data_path = os.path.join(_CURRENT_DIR, "../data")
    file_id = "16EFbJ3oA3musTM0kPMxSo120yT58t6DP"
    destination = os.path.join(data_path, "livox_horizon_data.zip")
    if not os.path.isfile(destination) and not os.path.isdir(os.path.join(data_path, "livox_horizon_data")):
        download_file_from_google_drive(file_id, destination)
        os.system("cd {} && unzip livox_horizon_data".format(data_path))


if __name__ == "__main__":
    main()
