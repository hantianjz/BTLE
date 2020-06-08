#!/bin/bash
mkdir -p out
cd out
cmake ../host -DUSE_BLADERF=1 -G Ninja
ninja -C .
