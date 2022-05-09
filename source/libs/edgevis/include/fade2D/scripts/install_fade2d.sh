#!/bin/bash
DIR="/usr/local/lib"
# create directory in user lib
sudo mkdir -p "$DIR/FADE2D"
# copy library
sudo cp ../lib/ubuntu16.10_x86_64/libfade2d.so $DIR/FADE2D/
