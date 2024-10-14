#!/usr/bin/env python3

import os
from pathlib import Path
from openravepy import *

# set up the plugin path
plugin_library_path = Path(__file__).resolve().parent / "build/src"
openrave_plugin_path = os.getenv("OPENRAVE_PLUGINS", "")
os.environ["OPENRAVE_PLUGINS"] = os.pathsep.join(
    [str(plugin_library_path), openrave_plugin_path]
)

print("OPENRAVE_PLUGINS:", os.environ["OPENRAVE_PLUGINS"])


try:
    RaveInitialize()

    if not RaveLoadPlugin("raveurdf"):
        raveLogError("Plugin not correctly loaded")

    env = Environment()
    env.SetViewer("qtcoin")
    env.Load("scenes/myscene.env.xml")

    MyModule = RaveCreateModule(env, "raveurdf")

    arg = "load {:s}".format("./robots/panda.urdf")
    MyModule.SendCommand(arg)
    robot = env.GetRobots()[0]
    print(f"Robot name: {robot.GetName()}")
    print(f"Robot DOF: {robot.GetDOF()}")
    print(f"Robot joints: {robot.GetJoints()}")
    print(f"Robot links: {robot.GetLinks()}")
    from IPython import embed

    embed()

except Exception as e:
    print(f"Failed to load module: {e}")

finally:
    RaveDestroy()
