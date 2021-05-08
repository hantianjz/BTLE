#!/bin/bash
gn gen out --export-compile-commands
cd out
ninja -v -C .
