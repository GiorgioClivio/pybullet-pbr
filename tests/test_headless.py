"""Headless rendering: p.DIRECT + the EGL plugin must produce the same PBR image as
the GUI, with no display at all."""
import os
import sys

# Proves this is genuinely headless rather than quietly leaning on an X server.
os.environ.pop("DISPLAY", None)
os.environ.pop("WAYLAND_DISPLAY", None)

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import pybullet_pbr

pybullet_pbr.activate()

import numpy as np
import pybullet as p
import pybullet_data

p.connect(p.DIRECT)
plugin = pybullet_pbr.enable_headless()
assert plugin >= 0 and pybullet_pbr.is_headless_enabled()

p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, 0)
plane = p.loadURDF("plane.urdf")
sphere = p.createCollisionShape(p.GEOM_SPHERE, radius=0.4)
left = p.createMultiBody(0, sphere, basePosition=[-0.7, 0, 0.4])
right = p.createMultiBody(0, sphere, basePosition=[0.7, 0, 0.4])

view = p.computeViewMatrixFromYawPitchRoll([0, 0, 0.4], 3.2, 40, 5, 0, 2)
proj = p.computeProjectionMatrixFOV(70, 4 / 3, 0.1, 100)


def render(renderer=None):
    p.stepSimulation()
    kwargs = {} if renderer is None else {"renderer": renderer}
    w, h, rgb, dep, _ = p.getCameraImage(320, 240, view, proj, **kwargs)
    img = np.asarray(rgb, dtype=np.uint8).reshape(h, w, 4)[:, :, :3].astype(np.float64)
    return img, np.asarray(dep).reshape(h, w) >= 1.0


# 1. Scene presets reach the headless renderer: the sky is drawn and it is the right
#    colour for the body. (This is also the check that the sun direction arrives -
#    it travels through configureDebugVisualizer in the GUI, which does nothing here.)
pybullet_pbr.set_scene("mars_day")
mars, sky = render(p.ER_BULLET_HARDWARE_OPENGL)
assert sky.sum() > 500, "no background visible in the test framing"
m = mars[sky].mean(0)
assert m[0] > m[2] + 10, "the Martian sky did not reach the headless renderer (%s)" % m.round(0)

pybullet_pbr.set_scene("earth_day")
earth, _ = render(p.ER_BULLET_HARDWARE_OPENGL)
e = earth[sky].mean(0)
assert e[2] > e[0] + 8, "Earth's sky did not reach the headless renderer (%s)" % e.round(0)

# 2. It is genuinely this renderer and not PyBullet's software fallback. Once the EGL
#    plugin is loaded it serves every getCameraImage regardless of the renderer flag,
#    so the check is for something TinyRenderer cannot draw: a sky *gradient*, rather
#    than the flat background colour it clears to.
sky_rows = np.nonzero(sky.all(1))[0]
assert sky_rows.size > 20, "not enough full-width sky rows to judge a gradient"
band = max(len(sky_rows) // 4, 4)
top = earth[sky_rows[:band]].mean()
bottom = earth[sky_rows[-band:]].mean()
assert abs(bottom - top) > 4, \
    "the background is flat, so this is not the PBR renderer (%.0f vs %.0f)" % (top, bottom)

# 3. Materials apply, including through the by-body route the headless path needs.
pybullet_pbr.set_material(left, base_color=[0.85, 0.85, 0.85, 1], metallic=0.0,
                          roughness=0.45)
pybullet_pbr.set_material(right, base_color=[0.85, 0.85, 0.85, 1], metallic=1.0,
                          roughness=0.45)
img, _ = render(p.ER_BULLET_HARDWARE_OPENGL)
lum = img @ [0.2126, 0.7152, 0.0722]
dielectric = lum[110:190, 60:150].mean()
metal = lum[110:190, 175:265].mean()
assert abs(metal - dielectric) > 8, \
    "metal and dielectric render the same headless (%.0f vs %.0f)" % (metal, dielectric)

# 4. Emissive surfaces glow with no light in the scene at all.
pybullet_pbr.set_scene("moon_psr")
pybullet_pbr.set_ev(6.0)
dark, _ = render(p.ER_BULLET_HARDWARE_OPENGL)
assert dark.max() < 8, "a permanently shadowed region is not dark headless (%d)" % dark.max()
pybullet_pbr.set_material(right, base_color=[0.2, 1.0, 0.3, 1], emissive=4000)
glow, _ = render(p.ER_BULLET_HARDWARE_OPENGL)
assert glow.max() > 200, "an emissive surface does not glow headless (%d)" % glow.max()

# 5. Lights work headless too.
pybullet_pbr.set_material(right, emissive=0.0)
pybullet_pbr.clear_lights()
before, _ = render(p.ER_BULLET_HARDWARE_OPENGL)
pybullet_pbr.add_spot_light(position=[0, -1.5, 2.6], yaw=90, pitch=-55, lumens=12000,
                            cone_angle=110)
after, _ = render(p.ER_BULLET_HARDWARE_OPENGL)
assert after.mean() > before.mean() + 5, \
    "a spot light does not light the headless scene (%.1f vs %.1f)" % (after.mean(),
                                                                       before.mean())
pybullet_pbr.clear_lights()
p.disconnect()
print("OK: no DISPLAY; mars sky %s, earth sky %s, sky gradient %.0f->%.0f, "
      "metal %.0f vs dielectric %.0f, emissive %d, spot +%.1f"
      % (m.round(0), e.round(0), top, bottom, metal, dielectric, glow.max(),
         after.mean() - before.mean()))
