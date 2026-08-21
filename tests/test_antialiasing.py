"""FXAA softens silhouette staircases without blurring the frame."""
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
p.setGravity(0, 0, 0)
p.configureDebugVisualizer(rgbBackground=[0, 0, 0])
# No environment: this test needs a black background to measure a silhouette against,
# and the sky would be drawn over it.
pybullet_pbr.set_environment(None)

# A bright sphere on a black background: its silhouette is pure aliasing.
sphere = p.createCollisionShape(p.GEOM_SPHERE, radius=0.5)
body = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=sphere, basePosition=[0, 0, 0])
pybullet_pbr.set_material(body, base_color=[0.9, 0.9, 0.9, 1], metallic=0, roughness=0.5)
p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0)

view = p.computeViewMatrixFromYawPitchRoll([0, 0, 0], 2.0, 30, -20, 0, 2)
proj = p.computeProjectionMatrixFOV(50, 4 / 3, 0.1, 100)


def render():
    p.stepSimulation()
    w, h, rgb, _, _ = p.getCameraImage(400, 300, view, proj,
                                       renderer=p.ER_BULLET_HARDWARE_OPENGL)
    img = np.asarray(rgb, dtype=np.uint8).reshape(h, w, 4)[:, :, :3].astype(np.float64)
    return img @ [0.2126, 0.7152, 0.0722]


def _spread(mask):
    out = np.zeros_like(mask)
    out[1:, :] |= mask[:-1, :]
    out[:-1, :] |= mask[1:, :]
    out[:, 1:] |= mask[:, :-1]
    out[:, :-1] |= mask[:, 1:]
    return out


def edge_pixels(lum):
    """Silhouette pixels that are part-way between black background and lit sphere.
    An aliased edge is a binary step and has almost none; AA fills them in."""
    bg = lum < 3
    band = _spread(bg) & _spread(~bg)
    return (band & (lum > 3) & (lum < 60)).sum()


pybullet_pbr.set_antialiasing("off")
plain = render()
pybullet_pbr.set_antialiasing("fxaa")
aa = render()

assert pybullet_pbr.get_antialiasing() == "fxaa"
soft_plain, soft_aa = edge_pixels(plain), edge_pixels(aa)
assert soft_aa > soft_plain * 1.3, \
    "FXAA barely softens the silhouette (%d -> %d intermediate pixels)" % (soft_plain, soft_aa)

# The hard step across the silhouette must come down too.
grad_plain = np.abs(np.diff(plain, axis=1)).max()
grad_aa = np.abs(np.diff(aa, axis=1)).max()
assert grad_aa < grad_plain * 0.85, \
    "steepest edge barely changed (%.0f -> %.0f)" % (grad_plain, grad_aa)

# It must be an edge filter, not a blur: the interior and background stay put.
changed = (np.abs(aa - plain) > 2)
assert changed.mean() < 0.10, "FXAA touches %.0f%% of the frame - that is a blur" % (
    changed.mean() * 100)
# Flat areas (little local contrast) must be left alone; FXAA is allowed to soften
# genuinely high-contrast interior features such as a specular edge.
neighbours = np.stack([np.roll(plain, s, axis=a) for a in (0, 1) for s in (-1, 1)])
local_range = neighbours.max(0) - neighbours.min(0)
flat = local_range < 5
interior = np.abs(aa - plain)[flat]
assert interior.mean() < 0.5, "FXAA is blurring flat areas (%.2f)" % interior.mean()

# Toggling back must reproduce the un-antialiased image exactly.
pybullet_pbr.set_antialiasing("off")
again = render()
assert np.abs(again - plain).mean() < 0.01, "turning FXAA off did not restore the image"

# TAA is a separate mode with its own test; here just check the name is accepted.
pybullet_pbr.set_antialiasing("taa")
assert pybullet_pbr.get_antialiasing() == "taa"
try:
    pybullet_pbr.set_antialiasing("msaa")
    raise AssertionError("an unknown antialiasing mode was accepted")
except ValueError:
    pass

pybullet_pbr.set_antialiasing("fxaa")
pybullet_pbr.set_environment()
p.disconnect()
print("OK: silhouette pixels %d -> %d (+%.0f%%), steepest edge %.0f -> %.0f, "
      "%.1f%% of frame touched, flat-area delta %.2f"
      % (soft_plain, soft_aa, (soft_aa / soft_plain - 1) * 100, grad_plain, grad_aa,
         changed.mean() * 100, interior.mean()))
