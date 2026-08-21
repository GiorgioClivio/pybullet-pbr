"""The locally built pybullet loads as `pybullet`, and physics is unchanged."""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import pybullet_pbr

assert not pybullet_pbr.is_active()
pybullet_pbr.activate()

import pybullet as p
import pybullet_data

assert p.__file__ == pybullet_pbr.binary_path(), p.__file__
assert pybullet_pbr.is_active()
pybullet_pbr.activate()  # idempotent

p.connect(p.DIRECT)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.loadURDF("plane.urdf")
body = p.loadURDF("r2d2.urdf", [0, 0, 1])
for _ in range(240):
    p.stepSimulation()
z = p.getBasePositionAndOrientation(body)[0][2]
assert 0.1 < z < 0.9, "r2d2 should settle on the plane, got z=%r" % z

w, h, rgb, dep, seg = p.getCameraImage(64, 48)
assert (w, h) == (64, 48) and __import__("numpy").asarray(dep).size == 64 * 48

p.disconnect()
print("OK: local build active, physics + getCameraImage fine (z=%.3f)" % z)
