#!/usr/bin/env bash

# run_3D_render_tabletop_environment_example.sh:
#   example application of how render using
#   the toolkit
#
# author: Everett
# created: 2021-04-29 13:09
# Github: https://github.com/antiqueeverett/

PROJECT_DIR=$(dirname $(dirname $(readlink -f "$0")))

# -- BIN directory
cd "$PROJECT_DIR" || return

./build/bin/3D-render-tabletop-environment --logtostderr=1
