#!/usr/bin/env bash

# run_print_pointcloud_segment_example.sh:
#   runs example application of how to print
#   a pointcloud using the toolkit
#
# author: Everett
# created: 2021-06-22 10:13
# Github: https://github.com/antiqueeverett/

PROJECT_DIR=$(dirname $(dirname $(readlink -f "$0")))

# -- BIN directory
cd "$PROJECT_DIR" || return

mkdir -p ./output
rm -rf ./output/print-pointcloud-segment
./build/bin/print-pointcloud-segment --logtostderr=1

cloudcompare.CloudCompare ./output/context.ply >/dev/null 2>&1 &
