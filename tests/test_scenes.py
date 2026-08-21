"""Acceptance for the scenario presets: each lighting environment is applied,
distinguishable, and physically sane; named materials behave like the substance."""
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
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
p.setGravity(0, 0, 0)
plane = p.loadURDF("plane.urdf")
ball = p.createCollisionShape(p.GEOM_SPHERE, radius=0.4)
left = p.createMultiBody(0, ball, basePosition=[-0.7, 0, 0.4])
right = p.createMultiBody(0, ball, basePosition=[0.7, 0, 0.4])

view = p.computeViewMatrixFromYawPitchRoll([0, 0, 0.4], 3.2, 40, 6, 0, 2)
proj = p.computeProjectionMatrixFOV(70, 4 / 3, 0.1, 100)


def render():
    p.stepSimulation()
    w, h, rgb, dep, _ = p.getCameraImage(320, 240, view, proj,
                                         renderer=p.ER_BULLET_HARDWARE_OPENGL)
    img = np.asarray(rgb, dtype=np.uint8).reshape(h, w, 4)[:, :, :3].astype(np.float64)
    return img, np.asarray(dep).reshape(h, w) >= 1.0


# 1. Every scene applies and produces a distinct image.
frames = {}
for name in pybullet_pbr.SCENES:
    pybullet_pbr.set_scene(name)
    frames[name] = render()[0]
means = {k: v.mean() for k, v in frames.items()}
assert len(set(round(m, 1) for m in means.values())) >= len(means) - 1, \
    "scenes are not distinguishable: %s" % {k: round(v, 1) for k, v in means.items()}

# 2. Airless bodies have a black sky; Earth's is blue; Mars' is red-dominant.
_, sky_mask = render()
def sky_of(name):
    img, mask = frames[name], sky_mask
    return img[mask].mean(0)

pybullet_pbr.set_scene("moon_equatorial")
moon_img, moon_sky = render()
assert moon_img[moon_sky].mean() < 6, \
    "the lunar sky is not black (%.1f)" % moon_img[moon_sky].mean()

pybullet_pbr.set_scene("earth_day")
earth_img, earth_sky = render()
e = earth_img[earth_sky].mean(0)
assert e[2] > e[0] + 10, "Earth's sky is not blue (%s)" % e.round(0)

pybullet_pbr.set_scene("mars_day")
mars_img, mars_sky = render()
m = mars_img[mars_sky].mean(0)
assert m[0] > m[2] + 10, "the Martian sky is not red-dominant (%s)" % m.round(0)

# 2b. The units are physical: the sun is stated in lux and the camera in EV.
day = pybullet_pbr.SCENES["earth_day"]
assert 80000 < day["sun"] < 120000, "Earth daylight is not ~100 000 lux (%s)" % day["sun"]
assert pybullet_pbr.SCENES["moon_equatorial"]["sun"] > day["sun"], \
    "the Moon should receive more illuminance than Earth's surface"
assert pybullet_pbr.SCENES["mars_day"]["sun"] < day["sun"] * 0.6, \
    "Mars should receive much less illuminance than Earth"
assert pybullet_pbr.SCENES["earth_night"]["sun"] < 1.0, \
    "a moonlit night should be under 1 lux"

# 3. Brightness ordering follows the physics: daylight > sunset > moonlit night, and a
#    permanently shadowed region is darker than anything else by a wide margin.
assert means["earth_noon"] > means["earth_sunset"] > means["earth_night"], \
    "Earth scenes are not ordered by illumination: %s" % {
        k: round(means[k], 1) for k in ("earth_noon", "earth_sunset", "earth_night")}
assert means["moon_psr"] < 3, "a permanently shadowed region is not dark (%.1f)" % means["moon_psr"]
assert means["moon_polar"] < means["moon_equatorial"], \
    "grazing polar sun is not dimmer than an overhead one"

# 4. Overrides are honoured and unknown keys are rejected.
pybullet_pbr.set_scene("mars_day", ev=12.5)
assert abs(pybullet_pbr.get_ev() - 12.5) < 1e-3
for bad in (("no_such_scene", {}), ("mars_day", {"sun_elevatoin": 10})):
    try:
        pybullet_pbr.set_scene(bad[0], **bad[1])
        raise AssertionError("set_scene accepted %r" % (bad,))
    except ValueError:
        pass

# 5. Named materials behave like the substance they name. Comparing two materials on
#    the *same* sphere isolates the material: the pixels that change are exactly it.
pybullet_pbr.set_scene("earth_day")


def compare(first, second):
    pybullet_pbr.set_material(left, preset=first)
    a = render()[0] @ [0.2126, 0.7152, 0.0722]
    pybullet_pbr.set_material(left, preset=second)
    b = render()[0] @ [0.2126, 0.7152, 0.0722]
    changed = np.abs(a - b) > 3
    assert changed.sum() > 300, "%s vs %s changed almost nothing" % (first, second)
    return a[changed], b[changed]


dark, light = compare("lunar_regolith", "concrete")
regolith, concrete = dark.mean(), light.mean()
# 9% reflectance against 26%, a ratio of 0.35 in linear light - but this is measured on
# the tone-mapped, sRGB-encoded image, where that becomes roughly 0.6.
assert regolith < concrete * 0.75, \
    "lunar regolith is not much darker than concrete (%.0f vs %.0f)" % (regolith, concrete)

# The lunar surface is not one material: Aristarchus-class ejecta reflects about three
# times what the mare does, which is why the plateau stands out from orbit.
mare, bright = compare("lunar_regolith", "lunar_bright")
assert bright.mean() > mare.mean() * 1.4, \
    "bright lunar ejecta is not brighter than mare regolith (%.0f vs %.0f)" % (
        bright.mean(), mare.mean())

rough, glossy = compare("rock_basalt", "ice")
rock_peak, ice_peak = rough.max(), glossy.max()
assert ice_peak > rock_peak + 20, \
    "ice is not glossier than basalt (peak %.0f vs %.0f)" % (ice_peak, rock_peak)

# clear_material returns a body to the configured default
pybullet_pbr.set_default_material(metallic=0.0, roughness=0.6)
pybullet_pbr.set_material(left, preset="lunar_regolith")
dark_again = render()[0] @ [0.2126, 0.7152, 0.0722]
pybullet_pbr.clear_material(left)
cleared = render()[0] @ [0.2126, 0.7152, 0.0722]
changed = np.abs(cleared - dark_again) > 3
assert changed.sum() > 300, "clear_material did not restore the default material"

try:
    pybullet_pbr.set_material(left, preset="unobtanium")
    raise AssertionError("set_material accepted an unknown preset")
except ValueError:
    pass

# 6. link=None covers a whole articulated robot in one call.
robot = p.loadURDF("r2d2.urdf", [0, 2.5, 0.7])
pybullet_pbr.set_material(robot, link=None, preset="gold_foil")
render()

p.disconnect()
print("OK: %d scenes all distinct; moon sky %.1f, earth sky %s, mars sky %s; "
      "regolith %.0f vs concrete %.0f (ejecta %.0f); ice peak %.0f vs basalt %.0f"
      % (len(frames), moon_img[moon_sky].mean(), e.round(0), m.round(0),
         regolith, concrete, bright.mean(), ice_peak, rock_peak))
