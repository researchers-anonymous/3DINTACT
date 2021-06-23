#!/usr/bin/env bash

# run_print_pointcloud_example.sh:
#   runs example application of how to print
#   a pointcloud using the toolkit
#
# author: Everett
# created: 2021-06-22 09:32
# Github: https://github.com/antiqueeverett/

PROJECT_DIR=$(dirname $(dirname $(readlink -f "$0")))

# -- BIN directory
cd "$PROJECT_DIR" || return

mkdir -p ./output
rm -rf ./output/print-pointcloud
./build/bin/print-pointcloud --logtostderr=1

cloudcompare.CloudCompare ./output/context.ply >/dev/null 2>&1 &
