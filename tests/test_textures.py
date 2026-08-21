"""Material maps: normal, roughness, metallic and AO textures."""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import pybullet_pbr

pybullet_pbr.activate()

import numpy as np
import pybullet as p
import pybullet_data

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
NORMAL = os.path.join(ROOT, "assets", "textures", "demo_normal.png")
ROUGHNESS = os.path.join(ROOT, "assets", "textures", "demo_roughness.png")
QUAD = os.path.join(ROOT, "assets", "meshes", "uv_quad.obj")

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
p.setGravity(0, 0, 0)

# A mesh with real UVs. Bullet's built-in primitives have poor ones, and a normal map
# is only as good as the UVs under it.
visual = p.createVisualShape(p.GEOM_MESH, fileName=QUAD)
quad = p.createMultiBody(baseMass=0, baseVisualShapeIndex=visual, basePosition=[0, 0, 0.01])
pybullet_pbr.set_scene("earth_day")
pybullet_pbr.set_antialiasing("off")
pybullet_pbr.ssao(enabled=False)   # measuring shading detail, not occlusion

view = p.computeViewMatrixFromYawPitchRoll([0, 0, 0], 3.0, 40, -45, 0, 2)
proj = p.computeProjectionMatrixFOV(45, 4 / 3, 0.1, 100)


def render():
    p.stepSimulation()
    w, h, rgb, _, _ = p.getCameraImage(400, 300, view, proj,
                                       renderer=p.ER_BULLET_HARDWARE_OPENGL)
    img = np.asarray(rgb, dtype=np.uint8).reshape(h, w, 4)[:, :, :3].astype(np.float64)
    return (img @ [0.2126, 0.7152, 0.0722])[110:210, 140:280]   # inside the quad


def detail(crop):
    """High-frequency detail only. A large quad picks up a smooth gradient of sky
    reflection across it, which a plain standard deviation would count as detail;
    differences between neighbouring pixels ignore that and see only the bumps."""
    return float(np.abs(np.diff(crop, axis=1)).mean())


pybullet_pbr.set_material(quad, base_color=[0.8, 0.78, 0.74, 1], metallic=0.0,
                          roughness=0.4)
plain = render()
# The quad is not perfectly uniform even untextured - it is large and catches a
# gradient of sky reflection - so every check below is relative to this baseline.

# 1. A normal map has to produce shading detail where the geometry has none.
pybullet_pbr.set_material(quad, normal_texture=NORMAL)
bumpy = render()
assert detail(bumpy) > detail(plain) * 3.0, \
    "the normal map produces no shading detail (%.2f vs %.2f)" % (detail(bumpy), detail(plain))

# 2. Its strength is adjustable, and 0 is the same as having no map.
pybullet_pbr.set_normal_scale(3.0)
strong = render()
pybullet_pbr.set_normal_scale(0.0)
flat = render()
pybullet_pbr.set_normal_scale(1.0)
assert detail(strong) > detail(bumpy), \
    "normal_scale does not deepen the effect (%.2f vs %.2f)" % (detail(strong), detail(bumpy))
assert detail(flat) < detail(bumpy) * 0.6, \
    "normal_scale=0 does not flatten the map (%.2f vs %.2f)" % (detail(flat), detail(bumpy))

# 3. A roughness map varies the finish across one surface.
pybullet_pbr.set_material(quad, roughness_texture=ROUGHNESS)
varied = render()
assert np.abs(varied - bumpy).mean() > 1.0, "the roughness map changes nothing"

# 4. Missing files fail loudly rather than rendering something wrong.
for bad in ("no_such_texture.png",):
    try:
        pybullet_pbr.load_texture(bad)
        raise AssertionError("a missing texture was accepted")
    except FileNotFoundError:
        pass

# 5. Registering the same file twice reuses one handle.
first = pybullet_pbr.load_texture(NORMAL)
assert pybullet_pbr.load_texture(NORMAL) == first, "textures are not de-duplicated"

pybullet_pbr.set_antialiasing("fxaa")
pybullet_pbr.ssao(enabled=True)
p.disconnect()
print("OK: flat %.2f -> normal-mapped %.2f (scale 3: %.2f, scale 0: %.2f), "
      "roughness map delta %.2f"
      % (detail(plain), detail(bumpy), detail(strong), detail(flat),
         np.abs(varied - bumpy).mean()))
