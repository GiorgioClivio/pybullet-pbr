"""Temporal anti-aliasing resolves edges on still *and* moving geometry, and a moving
object leaves no ghost trail behind it."""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import pybullet_pbr

pybullet_pbr.activate()

import numpy as np
import pybullet as p
import pybullet_data

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
# TAA accumulates across frames of one viewport. GUI frames render at the window's
# size and getCameraImage at its own, and alternating the two throws the history
# away every frame - so drive the offscreen render alone here.
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
p.setGravity(0, 0, 0)

plane = p.loadURDF("plane.urdf")
pybullet_pbr.set_material(plane, base_color=[0.75, 0.75, 0.75, 1], roughness=0.9)
box = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.35, 0.35, 0.35])
body = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=box, basePosition=[0, 0, 0.4])
pybullet_pbr.set_material(body, base_color=[0.85, 0.2, 0.2, 1], metallic=0, roughness=0.5)

view = p.computeViewMatrixFromYawPitchRoll([0, 0, 0.4], 2.4, 35, -15, 0, 2)
proj = p.computeProjectionMatrixFOV(50, 4 / 3, 0.1, 100)
W, H = 400, 300


def run(mode, moving, frames=40):
    pybullet_pbr.set_antialiasing(mode)
    for i in range(frames):
        x = 0.5 * np.sin(i * 0.12) if moving else 0.0
        yaw = i * 0.05 if moving else 0.0
        p.resetBasePositionAndOrientation(body, [x, 0, 0.4],
                                          p.getQuaternionFromEuler([0, 0, yaw]))
        p.stepSimulation()
        w, h, rgb, _, _ = p.getCameraImage(W, H, view, proj,
                                           renderer=p.ER_BULLET_HARDWARE_OPENGL)
    img = np.asarray(rgb, dtype=np.uint8).reshape(h, w, 4)[:, :, :3].astype(np.float64)
    return img, img @ [0.2126, 0.7152, 0.0722]


def steepest(lum):
    return np.abs(np.diff(lum, axis=1)).max()


_, still_off = run("off", False)
_, still_fxaa = run("fxaa", False)
_, still_taa = run("taa", False)
moving_off_img, moving_off = run("off", True)
_, moving_fxaa = run("fxaa", True)
moving_taa_img, moving_taa = run("taa", True)

assert steepest(still_taa) < steepest(still_off) * 0.9, \
    "TAA does not resolve static edges (%.0f vs %.0f)" % (steepest(still_taa),
                                                          steepest(still_off))
assert steepest(moving_taa) < steepest(moving_off) * 0.75, \
    "TAA does not resolve moving edges (%.0f vs %.0f)" % (steepest(moving_taa),
                                                          steepest(moving_off))
assert steepest(moving_taa) < steepest(moving_fxaa), \
    "TAA is not better than FXAA on motion (%.0f vs %.0f)" % (steepest(moving_taa),
                                                              steepest(moving_fxaa))

# No ghost trail: where the object used to be, TAA must agree with the un-antialiased
# frame. A trail would show up as the object's colour lingering on empty ground.
trail = np.abs(moving_taa_img - moving_off_img).max(axis=2)
red_ghost = ((moving_taa_img[:, :, 0] - moving_taa_img[:, :, 2]) > 40) & \
            ((moving_off_img[:, :, 0] - moving_off_img[:, :, 2]) < 15)
assert red_ghost.mean() < 0.002, \
    "the moving object leaves a ghost trail over %.2f%% of the frame" % (
        red_ghost.mean() * 100)
assert trail.mean() < 12, "TAA frame differs wholesale from the reference (%.1f)" % trail.mean()

# Switching back is clean.
pybullet_pbr.set_antialiasing("fxaa")
assert pybullet_pbr.get_antialiasing() == "fxaa"
p.disconnect()
print("OK: steepest edge still %.0f->%.0f, moving %.0f->%.0f (fxaa %.0f), ghost %.3f%%"
      % (steepest(still_off), steepest(still_taa), steepest(moving_off),
         steepest(moving_taa), steepest(moving_fxaa), red_ghost.mean() * 100))
