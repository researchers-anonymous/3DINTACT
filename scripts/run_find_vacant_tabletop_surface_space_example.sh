#!/usr/bin/env bash

# run_find_vacant_tabletop_surface_space_example.sh:
#   runs example of how to find vacant tabletop surface.
#
# author: Everett
# created: 2021-06-22 10:13
# Github: https://github.com/antiqueeverett/

PROJECT_DIR=$(dirname $(dirname $(readlink -f "$0")))

# -- BIN directory
cd "$PROJECT_DIR" || return

mkdir -p ./output
rm -rf ./output/context.ply
./build/bin/find-vacant-tabletop-surface-space --logtostderr=1

cloudcompare.CloudCompare ./output/context.ply >/dev/null 2>&1 &
