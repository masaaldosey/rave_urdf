#!/bin/bash

files=$(find . -name '*.stl')

for file in $files; do
    ctmconv $file "${file%.*}"_y.stl --upaxis Y
done
