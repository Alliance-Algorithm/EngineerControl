#!/bin/bash

SRC_DIR=/workspaces/EngineerControl/install
DST_DIR=ssh://remote//rmcs_install

unison -auto -batch -repeat watch -times ${SRC_DIR} ${DST_DIR} -force ${SRC_DIR}