"""Acceptance for movable point and spot lights: they light the scene, they cast
shadows, they can be moved, and they can ride on a body."""
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
pybullet_pbr.set_material(plane, base_color=[0.8, 0.8, 0.8, 1], metallic=0, roughness=0.9)
pillar = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.25, 0.25, 0.8])
blocker = p.createMultiBody(0, pillar, basePosition=[0, 0, 0.8])
pybullet_pbr.set_material(blocker, base_color=[0.8, 0.8, 0.8, 1], metallic=0, roughness=0.8)

# A scene with no sun and no sky at all, so anything we measure comes from our lights.
pybullet_pbr.set_scene("moon_psr")
# A PSR is metered at EV -2; expose for the lamps instead, and pick a lumen count
# that reads sensibly there.
pybullet_pbr.set_ev(4.0)
LUMENS = 1200.0
pybullet_pbr.set_antialiasing("off")
pybullet_pbr.clear_lights()

# Straight down, so image x/y map onto world x/y.
view = p.computeViewMatrixFromYawPitchRoll([0, 0, 0], 7.0, 0, -89.9, 0, 2)
proj = p.computeProjectionMatrixFOV(60, 1.0, 0.1, 100)
N = 300


def render():
    for _ in range(2):
        p.stepSimulation()
        w, h, rgb, _, _ = p.getCameraImage(N, N, view, proj,
                                           renderer=p.ER_BULLET_HARDWARE_OPENGL)
    img = np.asarray(rgb, dtype=np.uint8).reshape(h, w, 4)[:, :, :3].astype(np.float64)
    return img @ [0.2126, 0.7152, 0.0722]


def patch(lum, cx, cy, half=22):
    return lum[cy - half:cy + half, cx - half:cx + half].mean()


dark = render()
assert dark.mean() < 3, "a permanently shadowed region with no lights is not dark (%.1f)" % dark.mean()

def corners(lum):
    return max(patch(lum, 30, 30), patch(lum, N - 30, N - 30), patch(lum, N - 30, 30),
               patch(lum, 30, N - 30))


# --- spot light: the cone is a cone ----------------------------------------
# Straight down, so the pool on the ground is a circle of radius 3*tan(15deg) ~ 0.8 m
# while the frame is 8 m across: the corners are far outside it. (Aim a spot at a
# shallow angle instead and its cone projects into a long ellipse that really does
# reach the corners - which is correct, just useless for measuring containment.)
down = pybullet_pbr.add_spot_light(position=[0, 0, 3.0], yaw=0, pitch=-90,
                                   lumens=LUMENS, kelvin=5000, cone_angle=30, range=14)
pool = render()
assert patch(pool, 150, 150) > 40, \
    "the spot light does not light the ground under it (%.1f)" % patch(pool, 150, 150)
corner_delta = corners(pool) - corners(dark)
assert corner_delta < 4, \
    "light is leaking outside the spot cone (corner %.1f, was %.1f unlit)" % (
        corners(pool), corners(dark))
pybullet_pbr.remove_light(down)

# --- spot light: shadows ---------------------------------------------------
# Straight down and off to one side, with a beam wide enough that the pillar sits well
# inside the pool and throws its shadow away from the light.
spot = pybullet_pbr.add_spot_light(position=[-1.2, 0, 3.2], yaw=0, pitch=-90,
                                   lumens=6000, kelvin=5000, cone_angle=110, range=14)
lit = render()
# Behind the pillar is much darker than the same distance to the side of it.
behind = patch(lit, 205, 150)
beside = patch(lit, 150, 205)
assert beside > dark.mean() + 10, \
    "the spot light does not light the ground beside the pillar (%.1f)" % beside
assert behind < beside * 0.4, \
    "the spot light casts no shadow (behind %.1f vs beside %.1f)" % (behind, beside)

# Moving it moves the light. Rotating it 90 degrees about the origin lights a
# different part of the scene while keeping the total roughly the same.
pybullet_pbr.move_light(spot, position=[0, -1.2, 3.2])
moved = render()
assert tuple(round(c, 2) for c in pybullet_pbr.get_light(spot)["position"]) == (0.0, -1.2, 3.2)
assert np.abs(moved - lit).mean() > 10, \
    "moving the spot light barely changed the image (%.1f)" % np.abs(moved - lit).mean()

pybullet_pbr.remove_light(spot)
assert render().mean() < 3, "removing the light left it lit"

# --- point light -----------------------------------------------------------
point = pybullet_pbr.add_point_light(position=[-1.2, 0, 2.6], lumens=5000, range=14)
omni = render()
# Unlike a spot, it throws light in every direction: both flanks are lit.
assert patch(omni, 150, 60) > 4 and patch(omni, 150, N - 60) > 4, \
    "the point light is not omnidirectional (%.1f / %.1f)" % (patch(omni, 150, 60),
                                                              patch(omni, 150, N - 60))
behind_point = patch(omni, 205, 150)
beside_point = patch(omni, 150, 205)
assert behind_point < beside_point * 0.5, \
    "the point light casts no shadow (behind %.1f vs beside %.1f)" % (behind_point,
                                                                      beside_point)

# Shadows can be turned off per light.
pybullet_pbr.update_light(point, shadows=False)
no_shadow = render()
assert patch(no_shadow, 205, 150) > behind_point + 5, \
    "shadows=False still shadows (%.1f vs %.1f)" % (patch(no_shadow, 205, 150), behind_point)
pybullet_pbr.remove_light(point)

# --- the cone angle is the angle it says it is -----------------------------
# A spot aimed straight down from height h with a full cone angle a lights a circle of
# radius h*tan(a/2). Comparing against that catches the classic half-angle mistake,
# which makes every cone twice as wide as documented.
import math

pybullet_pbr.clear_lights()
height = 3.0
frame_metres = 2.0 * 7.0 * math.tan(math.radians(30.0))   # camera distance 7, 60 deg fov
for full_angle in (30.0, 60.0):
    pybullet_pbr.clear_lights()
    # off to one side, so the pillar is nowhere near the pool and cannot eat into it
    pybullet_pbr.add_spot_light(position=[-2.4, 0, height], yaw=0, pitch=-90, lumens=8000,
                                cone_angle=full_angle, cone_softness=0.02, range=14)
    lum = render()
    lit = lum > lum.max() * 0.2
    radius = math.sqrt(lit.sum() / math.pi) * frame_metres / N
    expected = height * math.tan(math.radians(full_angle / 2.0))
    assert abs(radius - expected) < expected * 0.15, \
        "a %.0f degree cone lights a radius of %.2f m, expected %.2f" % (
            full_angle, radius, expected)
pybullet_pbr.clear_lights()

# --- attaching to a body ---------------------------------------------------
rover = p.createMultiBody(0, p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.2, 0.2, 0.2]),
                          basePosition=[-2.0, 0, 0.2])
lamp = pybullet_pbr.add_spot_light(lumens=6000, cone_angle=40, range=14)
pybullet_pbr.attach_light(lamp, rover, link=-1, offset=[0.3, 0, 0.4],
                          direction=[1, 0, -0.35])
pybullet_pbr.sync_lights()
before = render()

p.resetBasePositionAndOrientation(rover, [0, -2.0, 0.2],
                                  p.getQuaternionFromEuler([0, 0, np.pi / 2]))
pybullet_pbr.sync_lights()
after = render()
assert np.abs(after - before).mean() > 0.5, "the attached light did not follow its body"

# The exact check: the light sits at the body's pose with the offset rotated into it.
# Rover at (0, -2, 0.2) yawed 90 degrees, offset (0.3, 0, 0.4) -> (0, -1.7, 0.6).
light_now = pybullet_pbr.get_light(lamp)
expected = (0.0, -1.7, 0.6)
assert all(abs(light_now["position"][i] - expected[i]) < 0.05 for i in range(3)), \
    "the attached light is not where the body put it (%s, expected %s)" % (
        tuple(round(c, 2) for c in light_now["position"]), expected)
# ... and its aim rotated with the body too: local +X became world +Y.
assert light_now["direction"][1] > 0.9 * abs(light_now["direction"][0] + 1e-9) and \
    light_now["direction"][1] > 0.8, \
    "the attached spot did not turn with the body (%s)" % (
        tuple(round(c, 2) for c in light_now["direction"]),)

pybullet_pbr.clear_lights()
assert pybullet_pbr.light_count() == 0
pybullet_pbr.set_antialiasing("fxaa")
p.disconnect()
print("OK: dark %.1f, spot %.1f (corner gain %.1f, shadow %.1f vs %.1f), point flanks lit, "
      "point shadow %.1f vs %.1f, attached light follows"
      % (dark.mean(), lit.mean(), corner_delta, behind, beside, behind_point, beside_point))
