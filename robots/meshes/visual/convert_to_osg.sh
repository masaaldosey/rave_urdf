#!/bin/bash

files=$(find . -name '*_y.stl')

for file in $files; do
    osgconv $file "${file%_*}".osg
done
