"""Contact shading where surfaces meet, nothing where they do not, and no halos."""
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
plane = p.loadURDF("plane.urdf")
pybullet_pbr.set_material(plane, base_color=[0.8, 0.8, 0.8, 1], metallic=0, roughness=0.8)
box = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.5, 0.5, 0.5])
p.createMultiBody(baseMass=0, baseCollisionShapeIndex=box, basePosition=[0, 0, 0.5])

view = p.computeViewMatrixFromYawPitchRoll([0, 0, 0.3], 3.0, 45, -25, 0, 2)
proj = p.computeProjectionMatrixFOV(60, 4 / 3, 0.1, 100)
W, H = 320, 240


def render():
    p.stepSimulation()
    w, h, rgb, dep, _ = p.getCameraImage(W, H, view, proj,
                                         renderer=p.ER_BULLET_HARDWARE_OPENGL)
    img = np.asarray(rgb, dtype=np.uint8).reshape(h, w, 4)[:, :, :3].astype(np.float64)
    return img, np.asarray(dep).reshape(h, w)


pybullet_pbr.ssao(enabled=True, radius=0.3, strength=1.0)
# Read the AO buffer unfiltered: FXAA would soften it and blur object AO one pixel
# onto the background, which is not the halo this test is looking for.
aa_mode = pybullet_pbr.get_antialiasing()
pybullet_pbr.set_antialiasing("off")
pybullet_pbr.set_diagnostic("ao")
ao_img, depth = render()
ao = ao_img[:, :, 0] / 255.0
pybullet_pbr.set_diagnostic(False)
pybullet_pbr.set_antialiasing(aa_mode)

# 1. Somewhere the geometry meets itself, occlusion must be substantial. A right-angle
#    corner blocks half the hemisphere, so ~0.5 is the physically expected floor.
assert ao.min() < 0.7, "no contact occlusion anywhere (min AO %.2f)" % ao.min()
assert ao.min() > 0.3, "occlusion far past a right-angle corner, kernel is wrong (%.2f)" % ao.min()

# 2. Open, flat, unoccluded ground must stay unoccluded - no acne, no false darkening.
open_ground = ao[H - 20:, :60]
assert open_ground.mean() > 0.97, \
    "flat open ground is being occluded (%.3f)" % open_ground.mean()

# 3. Background (nothing in front of it) must be untouched: this is the halo test.
background = ao[depth >= 1.0]
assert background.size > 100 and background.min() > 0.99, \
    "halo bleeding onto the background (min %.3f)" % background.min()

# 4. In the shaded image the effect must be visible but restrained, not dirty corners.
lit, _ = render()
pybullet_pbr.ssao(enabled=False)
plain, _ = render()
pybullet_pbr.ssao(enabled=True)
again, _ = render()

lum_lit = lit @ [0.2126, 0.7152, 0.0722]
lum_plain = plain @ [0.2126, 0.7152, 0.0722]
darkened = lum_plain - lum_lit
touched = (darkened > 2).mean()
assert touched > 0.005, "SSAO changes almost nothing (%.3f%% of pixels)" % (touched * 100)
# A pixel may never lose more light than its occlusion factor allows: AO applies to
# the ambient share only, so (1 - min AO) is the hard ceiling on the darkening. This
# is what separates honest occlusion from the "dirty corners" look.
worst = (darkened / np.maximum(lum_plain, 1e-6)).max()
ceiling = 1.0 - ao.min()
assert worst <= ceiling + 0.02, \
    "SSAO darkens a pixel by %.0f%%, past the %.0f%% its occlusion allows" % (
        worst * 100, ceiling * 100)
assert np.abs(again - lit).mean() < 1.0, "SSAO did not come back after being toggled off"

p.disconnect()
print("OK: min AO %.2f, open ground %.3f, background %.3f, %.1f%% of pixels darkened, "
      "worst %.0f%% of a %.0f%% ceiling"
      % (ao.min(), open_ground.mean(), background.min(), touched * 100, worst * 100,
         ceiling * 100))
