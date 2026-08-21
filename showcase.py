"""Everything pybullet_pbr does, in one scene you can drive from the keyboard.

    python3 showcase.py                  # interactive tour
    python3 showcase.py moon_psr         # start in a particular scene
    python3 showcase.py --shots          # render the whole tour to build/showcase/
    python3 showcase.py --list           # scenes and materials

Interactive keys are printed at startup and with H.
"""
import argparse
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import pybullet_pbr

pybullet_pbr.activate()

import pybullet as p
import pybullet_data

ROOT = os.path.dirname(os.path.abspath(__file__))
GRID = os.path.join(ROOT, "assets", "textures", "scale_grid.png")
# Assigning a plain white texture is how "no pattern" is expressed: *removing* a texture
# goes through PyBullet's own channel, which the headless converter ignores, so it would
# silently leave the checkerboard in place there.
WHITE = os.path.join(ROOT, "assets", "textures", "white.png")
NORMAL_MAP = os.path.join(ROOT, "assets", "textures", "demo_normal.png")
ROUGHNESS_MAP = os.path.join(ROOT, "assets", "textures", "demo_roughness.png")
QUAD = os.path.join(ROOT, "assets", "meshes", "uv_quad.obj")

SCENE_ORDER = ["earth_day", "earth_noon", "earth_sunset", "earth_night", "earth_indoor",
               "moon_equatorial", "moon_aristarchus", "moon_polar", "moon_psr",
               "mars_day", "mars_dusty", "mars_sunset"]
GROUND = {"moon": "lunar_regolith", "mars": "mars_regolith", "earth": "concrete"}
# The plateau is a bright unit sitting in dark mare, which is the point of it.
GROUND_BY_SCENE = {"moon_aristarchus": "lunar_highland"}
DARK = ("moon_psr", "moon_polar", "earth_night")

HELP = """
  N / P    next / previous scene            L    robot lights
  [ / ]    exposure down / up               E    auto-exposure
  A        anti-aliasing off/fxaa/taa       O    ambient occlusion
  B        bloom          V  vignette       X    sensor noise
  M        motion blur    T  material maps  D    diagnostic view
  C        camera orbit   R  reset          H    this help      Q  quit
"""


class Showcase(object):
    def __init__(self, scene="earth_day", ground_texture=None):
        # Any texture works here; the shipped grid is only a default, chosen because it
        # keeps object size readable without darkening the material under it.
        self.ground_texture = ground_texture if ground_texture is not None \
            else (GRID if os.path.exists(GRID) else None)
        self.scene_index = SCENE_ORDER.index(scene) if scene in SCENE_ORDER else 0
        self.lights = []
        self.state = dict(lights=False, auto_exposure=False, aa="fxaa", ssao=True,
                          bloom=False, vignette=False, noise=False, motion_blur=False,
                          textures=True, diagnostic="off", orbit=True, ev_offset=0.0)
        self.build()
        self.apply_scene()

    # -- scene -------------------------------------------------------------
    def build(self):
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)

        self.terrain = p.loadURDF("plane.urdf")

        # A row of materials, so one light shows what each substance does with it.
        ball = p.createCollisionShape(p.GEOM_SPHERE, radius=0.32)
        self.samples = []
        row = ["metal_polished", "gold_foil", "plastic_white", "ice", "rubber",
               "carbon_fiber"]
        for i, name in enumerate(row):
            body = p.createMultiBody(0, ball, basePosition=[(i - 2.5) * 0.9, 0.0, 0.32])
            pybullet_pbr.set_material(body, preset=name)
            self.samples.append((body, name))

        # Rocks in the local regolith, and a textured slab that shows the material maps.
        rock = p.createCollisionShape(p.GEOM_SPHERE, radius=0.45)
        self.rocks = [p.createMultiBody(0, rock, basePosition=pos)
                      # kept clear of the robot's circle: it is centred on (0, 2.8) with
                      # a radius of 1.6, and a rock at (0.2, 4.4) sat exactly on it
                      for pos in ([3.6, 2.6, 0.45], [-3.6, 2.4, 0.45], [1.9, 5.0, 0.45])]

        self.slab = None
        if os.path.exists(QUAD):
            visual = p.createVisualShape(p.GEOM_MESH, fileName=QUAD)
            self.slab = p.createMultiBody(0, baseVisualShapeIndex=visual,
                                          basePosition=[0.0, -2.0, 0.02])

        # Emissive markers: they glow with every light switched off.
        marker = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.08, 0.08, 0.3])
        self.markers = []
        for x, colour in ((-2.6, [1.0, 0.15, 0.1, 1]), (2.6, [0.15, 1.0, 0.35, 1])):
            body = p.createMultiBody(0, marker, basePosition=[x, -0.9, 0.3])
            pybullet_pbr.set_material(body, base_color=colour, emissive=2500)
            self.markers.append(body)

        self.robot = p.loadURDF("r2d2.urdf", [0, 2.8, 0.7])
        pybullet_pbr.set_material(self.robot, link=None, preset="metal_brushed")

    def apply_scene(self):
        name = self.scene_name()
        pybullet_pbr.set_scene(name)
        body = name.split("_")[0]
        # A near-white grid instead of PyBullet's checkerboard. The pattern is worth
        # keeping - it is how you read distance and object size at a glance - but the
        # stock one averages about 0.6 and multiplies the base colour, so every ground
        # preset rendered at roughly half the albedo it names. This one averages 0.95.
        preset = GROUND_BY_SCENE.get(name, GROUND.get(body, "concrete"))
        pybullet_pbr.set_material(self.terrain, preset=preset,
                                  base_color_texture=self.ground_texture)
        for rock in self.rocks:
            pybullet_pbr.set_material(rock, preset="rock_basalt")
        self.apply_textures()
        self.state["ev_offset"] = 0.0
        self.state["lights"] = name in DARK
        self.apply_lights()
        self.apply_sensor()

    def scene_name(self):
        return SCENE_ORDER[self.scene_index]

    # -- toggles -----------------------------------------------------------
    def apply_textures(self):
        if self.slab is None:
            return
        maps = dict(normal_texture=NORMAL_MAP, roughness_texture=ROUGHNESS_MAP) \
            if self.state["textures"] and os.path.exists(NORMAL_MAP) else {}
        pybullet_pbr.set_material(self.slab, base_color=[0.72, 0.70, 0.66, 1],
                                  metallic=0.0, roughness=0.45, **maps)

    def apply_lights(self):
        pybullet_pbr.clear_lights()
        self.lights = []
        if not self.state["lights"]:
            return
        # 6000 K: a neutral-white LED work light. Under the camera's daylight balance
        # this reads as white light rather than tungsten.
        lamp = pybullet_pbr.add_spot_light(lumens=4000, kelvin=6000, cone_angle=50,
                                           cone_softness=0.35, radius=0.04, range=25)
        pybullet_pbr.attach_light(lamp, self.robot, link=-1, offset=[0.45, 0, 0.35],
                                  direction=[1, 0, -0.22])
        beacon = pybullet_pbr.add_point_light(lumens=150, kelvin=6500, radius=0.05,
                                              range=8)
        pybullet_pbr.attach_light(beacon, self.robot, link=-1, offset=[-0.2, 0, 0.9])
        self.lights = [lamp, beacon]
        # A camera carrying its own lamp meters for whichever is brighter, the lamp or
        # what little ambient there is. Metering for the lamp unconditionally would blow
        # out a scene like moon_polar, which is dim but still has direct sun.
        lamp_ev = pybullet_pbr.photometry.ev_for_light(4000, distance=5.0, albedo=0.15)
        pybullet_pbr.set_ev(max(pybullet_pbr.get_ev(), lamp_ev))

    def apply_sensor(self):
        s = self.state
        pybullet_pbr.set_sensor(bloom=1.0 if s["bloom"] else 0.0, bloom_threshold=1.0,
                                vignette=0.5 if s["vignette"] else 0.0,
                                noise=0.2 if s["noise"] else 0.0,
                                motion_blur=0.6 if s["motion_blur"] else 0.0)
        pybullet_pbr.set_auto_exposure(s["auto_exposure"], key=0.18)
        pybullet_pbr.set_antialiasing(s["aa"])
        pybullet_pbr.ssao(enabled=s["ssao"])
        pybullet_pbr.set_diagnostic(s["diagnostic"])

    def bump_exposure(self, stops):
        self.state["ev_offset"] += stops
        pybullet_pbr.set_ev(pybullet_pbr.get_ev() + stops)

    def status(self):
        s = self.state
        return ("%-16s ev %+5.1f  aa=%-4s ssao=%-3s lights=%-3s auto=%-3s bloom=%-3s "
                "vig=%-3s noise=%-3s blur=%-3s maps=%-3s view=%s"
                % (self.scene_name(), pybullet_pbr.get_ev(), s["aa"],
                   "on" if s["ssao"] else "off", "on" if s["lights"] else "off",
                   "on" if s["auto_exposure"] else "off", "on" if s["bloom"] else "off",
                   "on" if s["vignette"] else "off", "on" if s["noise"] else "off",
                   "on" if s["motion_blur"] else "off",
                   "on" if s["textures"] else "off", s["diagnostic"]))


def drive(robot, step):
    """Kinematic circle, so attached lights sweep and motion blur has something to blur."""
    angle = step * 0.005
    radius = 1.6
    centre = (0.0, 2.8)   # behind the material row, so it never drives through it
    p.resetBasePositionAndOrientation(
        robot, [centre[0] + radius * math.cos(angle),
                centre[1] + radius * math.sin(angle), 0.7],
        p.getQuaternionFromEuler([0, 0, angle + math.pi / 2]))


def interactive(show):
    print(HELP)
    print(show.status())
    p.resetDebugVisualizerCamera(5.4, 0, -17, [0, 0.4, 0.5])
    step = 0
    keymap = {
        ord("n"): ("scene", 1), ord("p"): ("scene", -1),
        ord("["): ("ev", -0.5), ord("]"): ("ev", 0.5),
        ord("l"): ("toggle", "lights"), ord("e"): ("toggle", "auto_exposure"),
        ord("o"): ("toggle", "ssao"), ord("b"): ("toggle", "bloom"),
        ord("v"): ("toggle", "vignette"), ord("x"): ("toggle", "noise"),
        ord("m"): ("toggle", "motion_blur"), ord("t"): ("toggle", "textures"),
        ord("c"): ("toggle", "orbit"),
        ord("a"): ("cycle", ("aa", ["off", "fxaa", "taa"])),
        ord("d"): ("cycle", ("diagnostic", ["off", "normals", "ao"])),
        ord("h"): ("help", None), ord("r"): ("reset", None),
    }

    while True:
        events = p.getKeyboardEvents()
        changed = False
        for key, (action, value) in keymap.items():
            if not (events.get(key, 0) & p.KEY_WAS_TRIGGERED):
                continue
            if action == "scene":
                show.scene_index = (show.scene_index + value) % len(SCENE_ORDER)
                show.apply_scene()
            elif action == "ev":
                show.bump_exposure(value)
            elif action == "toggle":
                show.state[value] = not show.state[value]
                if value == "lights":
                    show.apply_lights()
                elif value == "textures":
                    show.apply_textures()
                else:
                    show.apply_sensor()
            elif action == "cycle":
                field, options = value
                show.state[field] = options[(options.index(show.state[field]) + 1)
                                            % len(options)]
                show.apply_sensor()
            elif action == "reset":
                show.apply_scene()
            elif action == "help":
                print(HELP)
            changed = True
        if events.get(ord("q"), 0) & p.KEY_WAS_TRIGGERED:
            break
        if changed:
            print(show.status())

        drive(show.robot, step)
        p.stepSimulation()
        pybullet_pbr.sync_lights()
        if show.state["orbit"]:
            p.resetDebugVisualizerCamera(5.4, 0 + 16 * math.sin(step * 0.002), -17,
                                         [0, 0.4, 0.5])
        step += 1
        time.sleep(1 / 240.0)


def shots(show, out, scenes_only=False):
    """Renders the tour: every scene, then every effect on and off."""
    import numpy as np
    from PIL import Image

    os.makedirs(out, exist_ok=True)
    view = p.computeViewMatrixFromYawPitchRoll([0, 0.4, 0.5], 5.4, 0, -17, 0, 2)
    proj = p.computeProjectionMatrixFOV(55, 4 / 3, 0.1, 100)

    def grab(name, steps=4):
        for _ in range(steps):
            drive(show.robot, 200)
            p.stepSimulation()
            pybullet_pbr.sync_lights()
            w, h, rgb, _, _ = p.getCameraImage(800, 600, view, proj,
                                               renderer=p.ER_BULLET_HARDWARE_OPENGL)
        img = np.asarray(rgb, dtype=np.uint8).reshape(h, w, 4)[:, :, :3]
        Image.fromarray(img).save(os.path.join(out, name + ".png"))
        print("  %-28s mean %5.1f" % (name, img.mean()))

    print("scenes ->", out)
    for index, name in enumerate(SCENE_ORDER):
        show.scene_index = index
        show.apply_scene()
        grab("scene_%02d_%s" % (index, name))

    if scenes_only:
        return

    print("features ->", out)
    show.scene_index = SCENE_ORDER.index("moon_psr")
    show.apply_scene()
    grab("feature_psr_headlights")
    show.state["bloom"] = True
    show.apply_sensor()
    grab("feature_psr_bloom")

    show.scene_index = SCENE_ORDER.index("earth_day")
    show.apply_scene()
    for field, name in (("bloom", "bloom"), ("vignette", "vignette"), ("noise", "noise"),
                        ("motion_blur", "motion_blur")):
        show.state = dict(show.state, bloom=False, vignette=False, noise=False,
                          motion_blur=False)
        show.state[field] = True
        show.apply_sensor()
        grab("feature_%s" % name)

    show.state = dict(show.state, bloom=False, vignette=False, noise=False,
                      motion_blur=False)
    for value in ("off", "fxaa", "taa"):
        show.state["aa"] = value
        show.apply_sensor()
        grab("feature_aa_%s" % value)
    show.state["aa"] = "fxaa"

    for value in (True, False):
        show.state["ssao"] = value
        show.apply_sensor()
        grab("feature_ssao_%s" % ("on" if value else "off"))
    show.state["ssao"] = True

    for value in (True, False):
        show.state["textures"] = value
        show.apply_textures()
        grab("feature_maps_%s" % ("on" if value else "off"))
    show.state["textures"] = True

    for value in ("normals", "ao", "off"):
        show.state["diagnostic"] = value
        show.apply_sensor()
        grab("feature_view_%s" % value)

    # Auto-exposure, metering the natural light: no lamps in frame, so the meter is
    # not being asked to expose for a light source pointed at it.
    for index in (SCENE_ORDER.index("earth_night"), SCENE_ORDER.index("earth_noon")):
        show.scene_index = index
        show.apply_scene()
        show.state["lights"] = False
        show.apply_lights()
        # The meter averages the whole frame, so a small very bright source in shot
        # dominates it - correct behaviour, but it is not what this comparison is
        # showing, so the glowing markers come off for it.
        for marker in show.markers:
            pybullet_pbr.set_material(marker, emissive=0.0)
        for auto in (False, True):
            show.state["auto_exposure"] = auto
            show.apply_sensor()
            grab("feature_auto_exposure_%s_%s" % (SCENE_ORDER[index],
                                                  "on" if auto else "off"))
    show.state["auto_exposure"] = False
    for marker in show.markers:
        pybullet_pbr.set_material(marker, emissive=2500)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("scene", nargs="?", default="earth_day")
    parser.add_argument("--shots", nargs="?", const=os.path.join(ROOT, "build", "showcase"),
                        help="render the tour to this directory instead of running it")
    parser.add_argument("--ground-texture", dest="ground_texture",
                        help="image to lay over the ground material; any texture works, "
                             "the shipped scale grid is only the default")
    parser.add_argument("--no-ground-texture", action="store_true")
    parser.add_argument("--scenes-only", action="store_true",
                        help="with --shots, render just the scene tour")
    parser.add_argument("--list", action="store_true")
    args = parser.parse_args()

    if args.list:
        print("scenes:")
        for name, description in pybullet_pbr.scene_names().items():
            print("  %-16s %s" % (name, description))
        print("materials:", ", ".join(sorted(pybullet_pbr.MATERIALS)))
        return

    if args.shots:
        p.connect(p.DIRECT)
        pybullet_pbr.enable_headless()
    else:
        p.connect(p.GUI)

    ground_texture = WHITE if args.no_ground_texture else args.ground_texture
    show = Showcase(args.scene, ground_texture=ground_texture)
    if args.shots:
        shots(show, args.shots, args.scenes_only)
    else:
        interactive(show)
    p.disconnect()


if __name__ == "__main__":
    main()
