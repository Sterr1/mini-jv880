#!/bin/bash

# Build MiniJV880
cd src
make clean || true
make -j
ls *.img
cd ..
