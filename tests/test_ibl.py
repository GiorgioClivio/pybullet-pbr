"""The environment lights the scene and is what you see behind it, a rough dielectric
and a mirror metal respond to it differently, and it can be switched off for scenes
that should have almost no indirect light."""
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
pybullet_pbr.set_material(plane, base_color=[0.7, 0.7, 0.7, 1], metallic=0, roughness=0.9)
shape = p.createCollisionShape(p.GEOM_SPHERE, radius=0.5)
ball = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=shape, basePosition=[0, 0, 0.5])
pybullet_pbr.set_antialiasing("off")
pybullet_pbr.ssao(enabled=False)

view = p.computeViewMatrixFromYawPitchRoll([0, 0, 0.5], 2.5, 40, 8, 0, 2)
proj = p.computeProjectionMatrixFOV(80, 4 / 3, 0.1, 100)


def render():
    p.stepSimulation()
    w, h, rgb, dep, _ = p.getCameraImage(320, 240, view, proj,
                                         renderer=p.ER_BULLET_HARDWARE_OPENGL)
    img = np.asarray(rgb, dtype=np.uint8).reshape(h, w, 4)[:, :, :3].astype(np.float64)
    return img, np.asarray(dep).reshape(h, w) >= 1.0


# 1. The default procedural sky is drawn behind the scene, and it is a sky: the top
#    of the frame differs from the horizon, and it is blue-ish rather than grey.
pybullet_pbr.set_environment(strength=1.0)
lit, sky_mask = render()
assert sky_mask.sum() > 5000, "no background visible in the test framing"
sky_rows = np.nonzero(sky_mask.all(1))[0]
assert sky_rows.size > 40, "not enough full-width sky rows to judge a gradient"
band = max(len(sky_rows) // 4, 5)
top = lit[sky_rows[:band]].mean()
bottom = lit[sky_rows[-band:]].mean()
assert bottom - top > 4, \
    "the background is flat, not a sky gradient (top %.0f, horizon %.0f)" % (top, bottom)
sky = lit[sky_mask]
assert sky.mean(0)[2] > sky.mean(0)[0] + 8, "sky is not blue-ish (%s)" % sky.mean(0).round(0)

# 2. It actually lights the scene: turning it off must darken the shaded geometry.
pybullet_pbr.set_material(ball, base_color=[0.8, 0.8, 0.8, 1], metallic=0, roughness=0.85)
with_ibl, _ = render()
pybullet_pbr.set_environment(None)
without_ibl, _ = render()
geom = ~sky_mask
lit_mean = (with_ibl @ [0.2126, 0.7152, 0.0722])[geom].mean()
dark_mean = (without_ibl @ [0.2126, 0.7152, 0.0722])[geom].mean()
assert lit_mean > dark_mean + 8, \
    "environment lighting barely changes the scene (%.0f vs %.0f)" % (lit_mean, dark_mean)

# 3. Strength scales the environment's contribution. (Note that switching the
#    environment off entirely is not the bottom of this scale: it hands ambient back
#    to the flat term, which can be brighter than a very weak environment.)
pybullet_pbr.set_environment(strength=1.0)
full, _ = render()
pybullet_pbr.set_environment(strength=0.3)
weak, _ = render()


def lum(img):
    return (img @ [0.2126, 0.7152, 0.0722])[geom].mean()


assert lum(full) - lum(weak) > 10, \
    "strength does not scale the environment (1.0 -> %.0f, 0.3 -> %.0f)" % (
        lum(full), lum(weak))

# 4. A mirror metal reflects the environment; a rough dielectric does not mirror it.
#    Compare the top and bottom of the ball: the sky is above, dark ground below, so a
#    mirror shows a much bigger vertical difference than a diffuse surface does.
pybullet_pbr.set_environment(strength=1.0)


# Drop the ground out of shot so every non-sky pixel is the ball itself.
p.resetBasePositionAndOrientation(plane, [0, 0, -60], [0, 0, 0, 1])


def vertical_contrast():
    """Top-of-ball minus bottom-of-ball brightness. The environment is bright above
    and dark below, so a mirror shows a far bigger split than a diffuse surface."""
    img, sky_m = render()
    lum_img = img @ [0.2126, 0.7152, 0.0722]
    ys, xs = np.nonzero(~sky_m)
    assert ys.size > 500, "the ball is not visible"
    top = ys.min() + (ys.max() - ys.min()) // 5
    bottom = ys.max() - (ys.max() - ys.min()) // 5
    rows = np.arange(img.shape[0])[:, None]
    ball = ~sky_m
    return lum_img[ball & (rows < top)].mean() - lum_img[ball & (rows > bottom)].mean()


pybullet_pbr.set_material(ball, base_color=[0.8, 0.8, 0.8, 1], metallic=0.0, roughness=0.9)
diffuse_contrast = vertical_contrast()
pybullet_pbr.set_material(ball, base_color=[0.9, 0.9, 0.9, 1], metallic=1.0, roughness=0.05)
mirror_contrast = vertical_contrast()
assert mirror_contrast > diffuse_contrast + 15, \
    "a mirror metal does not reflect the environment more than a diffuse ball " \
    "(%.0f vs %.0f)" % (mirror_contrast, diffuse_contrast)

pybullet_pbr.set_antialiasing("fxaa")
pybullet_pbr.ssao(enabled=True)
p.disconnect()
print("OK: sky zenith %.0f vs horizon %.0f, scene %.0f lit / %.0f unlit, strength 1.0 %.0f -> 0.3 %.0f, "
      "sky reflection contrast diffuse %.0f vs mirror %.0f"
      % (top, bottom, lit_mean, dark_mean, lum(full), lum(weak),
         diffuse_contrast, mirror_contrast))
