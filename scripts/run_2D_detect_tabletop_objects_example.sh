#!/usr/bin/env bash

# run_2D_detect_tabletop_objects_example.sh:
#   example application of how detect objects using
#   the toolkit
#
# author: Everett
# created: 2021-06-23 13:09
# Github: https://github.com/antiqueeverett/

PROJECT_DIR=$(dirname $(dirname $(readlink -f "$0")))

# -- BIN directory
cd "$PROJECT_DIR" || return

./build/bin/2D-detect-tabletop-objects --logtostderr=1
