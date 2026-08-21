"""Exposure behaves, and the filmic curve rolls off instead of clipping."""
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
p.loadURDF("plane.urdf")
p.loadURDF("r2d2.urdf", [0, 0, 0.6])
p.setGravity(0, 0, -9.81)
for _ in range(60):
    p.stepSimulation()

view = p.computeViewMatrixFromYawPitchRoll([0, 0, 0.3], 3.0, 45, -30, 0, 2)
proj = p.computeProjectionMatrixFOV(60, 4 / 3, 0.1, 100)


def grab():
    w, h, rgb, _, _ = p.getCameraImage(320, 240, view, proj,
                                       renderer=p.ER_BULLET_HARDWARE_OPENGL)
    return np.asarray(rgb, dtype=np.uint8).reshape(h, w, 4)[:, :, :3].astype(np.float64)


means = []
for exposure in (0.25, 0.5, 1.0, 2.0, 4.0):
    pybullet_pbr.configure(exposure=exposure, tonemapper="aces")
    means.append(grab().mean())
assert all(b < a for a, b in zip(means[1:], means[:-1])), "exposure is not monotonic: %s" % means
assert means[-1] - means[0] > 40, "exposure barely changes the image: %s" % means

# Same very bright input, two curves: clamping blows out, ACES keeps detail.
pybullet_pbr.configure(exposure=6.0, tonemapper="none")
clamped = grab()
pybullet_pbr.configure(exposure=6.0, tonemapper="aces")
filmic = grab()
blown_clamped = (clamped >= 254).mean()
blown_filmic = (filmic >= 254).mean()
assert blown_clamped > 0.5, "test scene is not bright enough to clip: %.2f" % blown_clamped
assert blown_filmic < blown_clamped / 2, \
    "aces does not roll off highlights (clamped %.2f vs aces %.2f)" % (blown_clamped, blown_filmic)

# Diagnostic toggle still available for A/B comparison.
pybullet_pbr.configure(exposure=1.0, diagnostic=True)
diag = grab()
pybullet_pbr.configure(diagnostic=False)
lit = grab()
assert np.abs(diag - lit).mean() > 10, "diagnostic mode does not change the image"

p.disconnect()
print("OK: exposure means %s; blown pixels clamp %.2f -> aces %.2f"
      % ([round(m, 1) for m in means], blown_clamped, blown_filmic))
