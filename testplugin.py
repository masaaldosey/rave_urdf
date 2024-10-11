#!/usr/bin/env python

import os

# Set up the plugin path
spike_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'build/src')
openrave_plugin_path = os.getenv('OPENRAVE_PLUGINS', '')
os.environ['OPENRAVE_PLUGINS'] = os.pathsep.join([spike_path, openrave_plugin_path])

print("OPENRAVE_PLUGINS:", os.environ['OPENRAVE_PLUGINS'])


from openravepy import *

try:
    RaveInitialize()

    if not RaveLoadPlugin('raveurdf'):
        raveLogError("Plugin not correctly loaded")

    env=Environment()
    env.SetViewer('qtcoin')
    env.Load('scenes/myscene.env.xml')

    MyModule = RaveCreateModule(env, 'raveurdf')
    
    arg = 'load {:s}'.format("/home/prabhat.kondamadugula/github/my-repos/rave_urdf/robots/panda.urdf")
    MyModule.SendCommand(arg)
    robot = env.GetRobots()[0]
    print(f"Robot name: {robot.GetName()}")
    print(f"Robot DOF: {robot.GetDOF()}")
    from IPython import embed; embed()


except Exception as e:
    print(f"Failed to load module: {e}")



finally:
    RaveDestroy()
