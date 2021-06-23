#!/usr/bin/env bash

# run_find_epsilon_helper.sh:
#   runs application for evaluating the 4th nearest
#   neighbours: necessary for finding epsilon proxy
#
# author: Everett
# created: 2021-06-23 09:56
# Github: https://github.com/antiqueeverett/

PROJECT_DIR=$(dirname $(dirname $(readlink -f "$0")))

# -- BIN directory
cd "$PROJECT_DIR" || return

mkdir -p ./output
rm -rf ./output/knn.csv
./build/bin/find-epsilon-helper --logtostderr=1
