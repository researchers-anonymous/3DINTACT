#!/usr/bin/env bash

# libtorch.sh
#   Iff no libtorch directory exists in external,
#   downloads the most up-to-date version.
#
# author: Everett
# created: 2021-05-10 16:52
# Github: https://github.com/antiqueeverett/

PROJECT_DIR=$(dirname $(dirname $(readlink -f "$0")))
echo "-- executing project"

# -- BIN directory
cd "$PROJECT_DIR/external" || return

if [ ! -d "libtorch" ]
then
    wget https://download.pytorch.org/libtorch/nightly/cpu/libtorch-shared-with-deps-latest.zip
    unzip libtorch-shared-with-deps-latest.zip
fi
