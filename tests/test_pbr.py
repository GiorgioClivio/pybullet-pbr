"""Metal, glossy plastic and rough plastic respond differently to identical lighting.

This one isolates the direct-lighting BRDF, so image-based lighting is switched off:
under an environment a metal is a mirror of the sky rather than a dark surface, which
is correct but says nothing about the Cook-Torrance terms being tested here.
"""
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

pybullet_pbr.set_environment(None)
sphere = p.createCollisionShape(p.GEOM_SPHERE, radius=0.5)
body = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=sphere, basePosition=[0, 0, 0])
BASE_COLOR = [0.8, 0.8, 0.8, 1]

view = p.computeViewMatrixFromYawPitchRoll([0, 0, 0], 2.2, 40, -20, 0, 2)
proj = p.computeProjectionMatrixFOV(50, 1.0, 0.1, 100)


def render(base_color=None, **material):
    pybullet_pbr.set_material(body, base_color=base_color or BASE_COLOR, **material)
    p.stepSimulation()
    w, h, rgb, dep, _ = p.getCameraImage(256, 256, view, proj,
                                         renderer=p.ER_BULLET_HARDWARE_OPENGL)
    img = np.asarray(rgb, dtype=np.uint8).reshape(h, w, 4)[:, :, :3].astype(np.float64)
    # the sphere is the only geometry in the scene, so anything with depth is it
    mask = np.asarray(dep).reshape(h, w) < 0.999
    assert mask.sum() > 500, "sphere not visible in the render"
    return img, mask


def luminance(img):
    return img @ [0.2126, 0.7152, 0.0722]


rough, mask = render(metallic=0.0, roughness=0.9)
glossy, _ = render(metallic=0.0, roughness=0.08)
metal, _ = render(metallic=1.0, roughness=0.2)

lr, lg, lm = (luminance(i)[mask] for i in (rough, glossy, metal))

# A glossy dielectric concentrates the same energy into a smaller, brighter highlight.
assert lg.max() > lr.max() + 15, \
    "glossy highlight is not brighter than the rough one (%.0f vs %.0f)" % (lg.max(), lr.max())
bright_rough = (lr > lr.max() * 0.8).mean()
bright_glossy = (lg > lg.max() * 0.8).mean()
assert bright_glossy < bright_rough / 2, \
    "glossy highlight is not more concentrated (%.3f vs %.3f)" % (bright_glossy, bright_rough)

# Metal has no diffuse lobe: away from the highlight it goes much darker.
assert np.median(lm) < np.median(lg) * 0.7, \
    "metal is not darker off-highlight than the dielectric (%.0f vs %.0f)" \
    % (np.median(lm), np.median(lg))

# ... and its specular is tinted by the base colour, unlike a dielectric's white one.
# Dial the exposure down first so the highlight is measured, not clipped.
pybullet_pbr.configure(exposure=0.01, tonemapper="reinhard")
gold, gmask = render(base_color=[0.95, 0.75, 0.3, 1], metallic=1.0, roughness=0.25)
hot = (luminance(gold) > luminance(gold)[gmask].max() * 0.8) & gmask
assert gold[:, :, 0][hot].max() < 250, "highlight is still clipped, cannot judge its colour"
r, g, b = (gold[:, :, c][hot].mean() for c in range(3))
assert r > g > b, "metal highlight is not tinted by the base colour (%.0f %.0f %.0f)" % (r, g, b)

pybullet_pbr.configure(exposure=0.8, tonemapper="aces")
pybullet_pbr.set_environment()
p.disconnect()
print("OK: rough max %.0f, glossy max %.0f (%.1f%% vs %.1f%% bright), metal median %.0f vs %.0f;"
      " gold highlight rgb %.0f/%.0f/%.0f"
      % (lr.max(), lg.max(), bright_rough * 100, bright_glossy * 100,
         np.median(lm), np.median(lg), r, g, b))
