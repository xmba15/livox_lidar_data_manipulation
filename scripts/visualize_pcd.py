#!/usr/bin/env python
# -*- coding: utf-8 -*-
import argparse
import open3d as o3d


def visualize_pcd(data_path):
    pointcloud = o3d.io.read_point_cloud(data_path)
    o3d.visualization.draw_geometries([pointcloud])


def main(args):
    visualize_pcd(args.data_path)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        "simple script to visualize pointcloud data using open3d"
    )
    parser.add_argument(
        "--data_path", type=str, required=True, help="path to pointcloud data"
    )
    args = parser.parse_args()
    main(args)
