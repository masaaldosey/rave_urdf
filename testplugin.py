#!/usr/bin/env 
from openravepy import *
RaveInitialize()
RaveLoadPlugin('build/raveurdf')
try:
    env=Environment()
    env.Load('scenes/myscene.env.xml')

finally:
    RaveDestroy()
