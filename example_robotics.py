"""What integrating pybullet_pbr into an existing robotics script looks like.

    python3 example_robotics.py                  # interactive, earth_day
    python3 example_robotics.py moon_psr         # any scene name
    python3 example_robotics.py mars_day --shot out.png
    python3 example_robotics.py moon_psr --no-lights

In dark scenarios (moon_psr, moon_polar, earth_night) the robot carries its own
lighting: a spot headlamp aimed forward and a point beacon above it, both attached to
the base so they move and turn with it, and both casting real shadows. The robot drives
a slow circle so you can watch the beam sweep across the terrain. Force it on or off
anywhere with --lights / --no-lights.

The two integration lines are marked ADDED; everything else is ordinary PyBullet.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import pybullet_pbr                                             # ADDED
pybullet_pbr.activate()                                         # ADDED

import time

import pybullet as p
import pybullet_data

import math

scene = "earth_day"
shot = None
args = sys.argv[1:]
if args and not args[0].startswith("--"):
    scene = args[0]
if "--shot" in args:
    shot = args[args.index("--shot") + 1]

DARK = ("moon_psr", "moon_polar", "earth_night")
lights_on = scene in DARK
if "--lights" in args:
    lights_on = True
if "--no-lights" in args:
    lights_on = False

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)

terrain = p.loadURDF("plane.urdf")
robot = p.loadURDF("r2d2.urdf", [0, 0, 0.7])
rocks = [p.createMultiBody(0, p.createCollisionShape(p.GEOM_SPHERE, radius=r),
                           basePosition=[x, y, r])
         for x, y, r in ((1.6, 0.8, 0.30), (-1.4, 1.1, 0.22), (0.6, -1.5, 0.35))]

# --- the only rendering-specific lines you need ---------------------------
pybullet_pbr.set_scene(scene)
ground = {"moon": "lunar_regolith", "mars": "mars_regolith"}.get(scene.split("_")[0],
                                                                 "concrete")
pybullet_pbr.set_material(terrain, preset=ground,
                          base_color_texture=os.path.join(
                              os.path.dirname(os.path.abspath(__file__)),
                              "assets", "textures", "scale_grid.png"))
pybullet_pbr.set_material(robot, link=None, preset="aluminium")
for rock in rocks:
    pybullet_pbr.set_material(rock, preset="rock_basalt")

# Lights that ride on the robot. attach_light() places them in the base's local
# frame, so they follow it as it drives; sync_lights() in the loop applies that.
if lights_on:
    # A real rover headlamp: ~4000 lm of neutral-white LED in a 34-degree beam.
    headlamp = pybullet_pbr.add_spot_light(lumens=4000, kelvin=6000, cone_angle=48,
                                           cone_softness=0.35, radius=0.04, range=25)
    pybullet_pbr.attach_light(headlamp, robot, link=-1,
                              offset=[0.45, 0.0, 0.35], direction=[1.0, 0.0, -0.22])
    beacon = pybullet_pbr.add_point_light(lumens=120, kelvin=8000, radius=0.05, range=8)

    # Meter for the headlamp rather than for the ambient. A scene preset's EV is set
    # for whatever natural light it has, and a permanently shadowed region has almost
    # none - a camera driving there would expose for its own lamp, as here.
    lamp_ev = pybullet_pbr.photometry.ev_for_light(lumens=4000, distance=5.0, albedo=0.15)
    pybullet_pbr.set_ev(max(pybullet_pbr.get_ev(), lamp_ev))
    pybullet_pbr.attach_light(beacon, robot, link=-1, offset=[-0.2, 0.0, 0.9])
# --------------------------------------------------------------------------

p.resetDebugVisualizerCamera(4.0, 55, -20, [0, 0, 0.4])

def drive(step):
    """Kinematic circle, so the attached lights visibly sweep the scene."""
    angle = step * 0.006
    radius = 2.2
    position = [radius * math.cos(angle), radius * math.sin(angle), 0.7]
    heading = angle + math.pi / 2
    p.resetBasePositionAndOrientation(robot, position,
                                      p.getQuaternionFromEuler([0, 0, heading]))


if shot:
    import numpy as np
    from PIL import Image
    for step in range(60):
        drive(step * 12)
        p.stepSimulation()
        pybullet_pbr.sync_lights()
    # Chase camera, so a lamp mounted on the robot points away from the viewer and
    # you see what it lights rather than the back of the robot.
    position, orientation = p.getBasePositionAndOrientation(robot)
    heading = p.getEulerFromQuaternion(orientation)[2]
    view = p.computeViewMatrixFromYawPitchRoll(
        [position[0], position[1], 0.4], 3.4, math.degrees(heading) - 90, -18, 0, 2)
    proj = p.computeProjectionMatrixFOV(60, 4 / 3, 0.1, 100)
    w, h, rgb, _, _ = p.getCameraImage(800, 600, view, proj,
                                       renderer=p.ER_BULLET_HARDWARE_OPENGL)
    Image.fromarray(np.asarray(rgb, dtype=np.uint8).reshape(h, w, 4)).save(shot)
    print("wrote", shot)
else:
    print("scene: %s - %s" % (scene, pybullet_pbr.SCENES[scene]["description"]))
    if lights_on:
        print("robot lights: spot headlamp + point beacon, both casting shadows")
    step = 0
    while True:
        drive(step)
        p.stepSimulation()
        pybullet_pbr.sync_lights()      # keeps attached lights on the robot
        step += 1
        time.sleep(1 / 240.0)
p.disconnect()
