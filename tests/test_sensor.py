"""Camera sensor model: auto-exposure, bloom, vignette, noise, motion blur."""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import pybullet_pbr

pybullet_pbr.activate()

import numpy as np
import pybullet as p
import pybullet_data

# Headless, so every frame renders at one size. Interleaving GUI frames with
# getCameraImage alternates the viewport, which reallocates the render targets each
# frame and makes exposure metering jump around.
p.connect(p.DIRECT)
pybullet_pbr.enable_headless()
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, 0)

plane = p.loadURDF("plane.urdf")
pybullet_pbr.set_material(plane, preset="concrete")
sphere = p.createCollisionShape(p.GEOM_SPHERE, radius=0.4)
lamp = p.createMultiBody(0, sphere, basePosition=[0, 0, 0.4])
pybullet_pbr.set_antialiasing("off")
pybullet_pbr.set_sensor(bloom=0, vignette=0, noise=0, motion_blur=0)

view = p.computeViewMatrixFromYawPitchRoll([0, 0, 0.4], 3.0, 40, -15, 0, 2)
proj = p.computeProjectionMatrixFOV(55, 4 / 3, 0.1, 100)


def render():
    p.stepSimulation()
    w, h, rgb, _, _ = p.getCameraImage(400, 300, view, proj,
                                       renderer=p.ER_BULLET_HARDWARE_OPENGL)
    return np.asarray(rgb, dtype=np.uint8).reshape(h, w, 4)[:, :, :3].astype(np.float64)


# --- auto-exposure ---------------------------------------------------------
# The same camera, pointed at scenes 100000x apart in brightness, should bring both
# near mid grey - that is the whole job of a light meter.
pybullet_pbr.set_material(lamp, base_color=[0.7, 0.7, 0.7, 1], metallic=0, roughness=0.6)
pybullet_pbr.set_auto_exposure(False)
pybullet_pbr.set_scene("earth_noon")
manual_bright = render().mean()
pybullet_pbr.set_scene("earth_night")
manual_dark = render().mean()

pybullet_pbr.set_auto_exposure(True, key=0.18)
pybullet_pbr.set_scene("earth_noon")
bright = render().mean()
pybullet_pbr.set_scene("earth_night")
dark = render().mean()

manual_gap = abs(manual_bright - manual_dark)
auto_gap = abs(bright - dark)
assert auto_gap < manual_gap * 0.6, \
    "auto-exposure does not close the gap between day and night (%.0f -> %.0f)" % (
        manual_gap, auto_gap)
assert dark > manual_dark * 1.5, \
    "auto-exposure does not open up for a dark scene (%.0f vs %.0f)" % (dark, manual_dark)

# It is clamped, which is how a sensor that simply cannot see is modelled.
pybullet_pbr.set_auto_exposure(True, key=0.18, min_ev=14.0, max_ev=20.0)
clamped = render().mean()
assert clamped < dark * 0.5, \
    "clamping the meter's range has no effect (%.0f vs %.0f)" % (clamped, dark)
pybullet_pbr.set_auto_exposure(False)

# --- bloom -----------------------------------------------------------------
pybullet_pbr.set_scene("earth_night")
pybullet_pbr.set_material(lamp, base_color=[0.2, 1.0, 0.4, 1], emissive=3000)
plain = render()
pybullet_pbr.set_sensor(bloom=1.0, bloom_threshold=1.0)
bloomed = render()
# The halo has to appear *around* the source, not just brighten the source itself.
source = plain.mean(axis=2) > 200
halo = ~source & (np.abs(bloomed - plain).mean(axis=2) > 4)
assert halo.sum() > 400, "bloom produces no halo around a bright source (%d px)" % halo.sum()
# and it must be roughly round, not the rectangle a naive mip blur would give
ys, xs = np.nonzero(halo)
aspect = (ys.max() - ys.min() + 1) / float(xs.max() - xs.min() + 1)
assert 0.5 < aspect < 2.0, "the bloom halo is not round (aspect %.2f)" % aspect
pybullet_pbr.set_sensor(bloom=0.0)

# --- vignette --------------------------------------------------------------
pybullet_pbr.set_material(lamp, emissive=0.0)
pybullet_pbr.set_scene("earth_day")
flat = render()
pybullet_pbr.set_sensor(vignette=1.0)
vignetted = render()
corner_before = flat[:40, :40].mean()
corner_after = vignetted[:40, :40].mean()
centre_before = flat[130:170, 180:220].mean()
centre_after = vignetted[130:170, 180:220].mean()
assert corner_after < corner_before * 0.85, \
    "the vignette does not darken the corners (%.0f vs %.0f)" % (corner_after, corner_before)
assert abs(centre_after - centre_before) < 3, \
    "the vignette is darkening the centre too (%.0f vs %.0f)" % (centre_after, centre_before)
pybullet_pbr.set_sensor(vignette=0.0)

# --- sensor noise ----------------------------------------------------------
clean = render()
pybullet_pbr.set_sensor(noise=0.3)
noisy = render()
delta = np.abs(noisy - clean).mean()
assert delta > 0.5, "noise changes nothing (%.2f)" % delta
pybullet_pbr.set_sensor(noise=0.0)
restored = render()
assert np.abs(restored - clean).mean() < 0.2, "turning noise off does not restore the image"

# --- motion blur -----------------------------------------------------------
# A moving object's edges soften; a still one is untouched.
def edge_sharpness(moving):
    pybullet_pbr.set_material(lamp, base_color=[0.9, 0.9, 0.9, 1], emissive=0.0)
    for i in range(6):
        x = (0.06 * i) if moving else 0.0
        p.resetBasePositionAndOrientation(lamp, [x, 0, 0.4], [0, 0, 0, 1])
        image = render()
    return float(np.abs(np.diff(image.mean(axis=2), axis=1)).max())


pybullet_pbr.set_sensor(motion_blur=0.0)
still_sharp = edge_sharpness(False)
moving_sharp = edge_sharpness(True)
pybullet_pbr.set_sensor(motion_blur=1.0)
blurred_moving = edge_sharpness(True)
blurred_still = edge_sharpness(False)
assert blurred_moving < moving_sharp * 0.9, \
    "motion blur does not soften a moving edge (%.0f vs %.0f)" % (blurred_moving,
                                                                  moving_sharp)
assert blurred_still > still_sharp * 0.9, \
    "motion blur is smearing something that is not moving (%.0f vs %.0f)" % (
        blurred_still, still_sharp)

pybullet_pbr.set_sensor(motion_blur=0.0)
pybullet_pbr.set_antialiasing("fxaa")
p.disconnect()
print("OK: exposure gap %.0f -> %.0f auto (night %.0f -> %.0f, clamped %.0f), "
      "bloom halo %d px "
      "aspect %.2f, vignette corner %.0f->%.0f, noise delta %.2f, moving edge %.0f->%.0f"
      % (manual_gap, auto_gap, manual_dark, dark, clamped, halo.sum(), aspect,
         corner_before, corner_after, delta, moving_sharp, blurred_moving))
