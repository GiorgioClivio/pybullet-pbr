"""Sun shadows: placement, no acne, camera-independence, and a finite sun's penumbra."""
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

proj = p.computeProjectionMatrixFOV(60, 4 / 3, 0.1, 100)


def render(yaw=45, pitch=-35):
    view = p.computeViewMatrixFromYawPitchRoll([0, 0, 0], 4.0, yaw, pitch, 0, 2)
    p.stepSimulation()
    w, h, rgb, _, _ = p.getCameraImage(320, 240, view, proj,
                                       renderer=p.ER_BULLET_HARDWARE_OPENGL)
    img = np.asarray(rgb, dtype=np.uint8).reshape(h, w, 4)[:, :, :3].astype(np.float64)
    return img @ [0.2126, 0.7152, 0.0722]


# 1. An empty lit plane must not shadow itself: no acne speckle.
bare = render()
ground = bare[bare > 20]  # ignore the sky band
dark_outliers = (ground < np.median(ground) * 0.6).mean()
assert dark_outliers < 0.01, "shadow acne on a flat lit plane: %.1f%% dark pixels" % (
    dark_outliers * 100)

# 2. A box casts a real shadow: the scene darkens a lot compared to shadows-off.
box = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.4, 0.4, 0.4])
p.createMultiBody(baseMass=0, baseCollisionShapeIndex=box, basePosition=[0, 0, 0.4])
with_shadow = render()
p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0)
without_shadow = render()
p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)
again = render()

darkened = (without_shadow - with_shadow) > 20
assert darkened.mean() > 0.02, \
    "the box casts almost no shadow (%.2f%% of pixels darkened)" % (darkened.mean() * 100)
assert np.abs(again - with_shadow).mean() < 1.0, "shadows did not come back after toggling"

# The shadow must sit on the ground away from the light, not on top of the box.
ys, xs = np.nonzero(darkened)
assert ys.mean() > with_shadow.shape[0] * 0.45, "shadow is not on the ground below the box"

# 3. Strength control: 0 must match the shadows-off image.
pybullet_pbr.set_shadows(strength=0.0)
off_by_strength = render()
pybullet_pbr.set_shadows(strength=1.0)
assert np.abs(off_by_strength - without_shadow).mean() < 1.0, \
    "strength=0 does not match COV_ENABLE_SHADOWS=0"

# 4. Camera-independent: orbiting a fixed target must not change how much is shadowed.
fractions = []
for yaw in (30, 45, 60, 75):
    img = render(yaw=yaw)
    lit = img[img > 20]
    fractions.append((lit < np.median(lit) * 0.6).mean())
spread = max(fractions) - min(fractions)
assert spread < 0.05, "shadowed fraction swims with the camera: %s" % (
    [round(f, 3) for f in fractions])

# 5. The sun has an angular size, so a shadow edge softens with distance from its
#    caster. Measured on a tall thin post, where the far end of the shadow is many
#    metres from the geometry that casts it.
p.resetSimulation()
p.setAdditionalSearchPath(pybullet_data.getDataPath())
plane = p.loadURDF("plane.urdf")
pybullet_pbr.set_material(plane, base_color=[0.8, 0.8, 0.8, 1], metallic=0, roughness=0.9)
post = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.06, 0.06, 1.6])
p.createMultiBody(0, post, basePosition=[0, 0, 1.6])
pybullet_pbr.set_sun_direction(elevation=32, azimuth=180)
post_view = p.computeViewMatrixFromYawPitchRoll([1.6, 0, 0.1], 5.0, 90, -22, 0, 2)


def shadow_edge_sharpness(angular_diameter):
    """Steepest brightness step across the shadow, in the half of the frame nearest the
    camera - which is where the shadow is furthest from the post that casts it, so a
    finite sun should have visibly softened it."""
    pybullet_pbr.set_sun_angular_diameter(angular_diameter)
    p.stepSimulation()
    w, h, rgb, _, _ = p.getCameraImage(400, 300, post_view, proj,
                                       renderer=p.ER_BULLET_HARDWARE_OPENGL)
    img = np.asarray(rgb, dtype=np.uint8).reshape(h, w, 4)[:, :, :3].astype(np.float64)
    lum = img @ [0.2126, 0.7152, 0.0722]
    return float(np.abs(np.diff(lum[h // 2:], axis=1)).max())


hard = shadow_edge_sharpness(0.0)
real = shadow_edge_sharpness(0.53)
wide = shadow_edge_sharpness(4.0)
assert hard > 20, "no shadow edge to measure (%.0f)" % hard
assert real <= hard, \
    "a 0.53-degree sun sharpens the shadow instead of softening it (%.0f vs %.0f)" % (
        real, hard)
assert wide < hard * 0.75, \
    "a wider sun does not soften the shadow edge (%.0f vs %.0f)" % (wide, hard)
pybullet_pbr.set_sun_angular_diameter(0.53)

p.disconnect()
print("OK: acne %.2f%%, shadowed %.1f%% of pixels, strength=0 matches off, "
      "orbit spread %.3f, shadow edge %.0f/%.0f/%.0f (sun 0/0.53/4 deg)"
      % (dark_outliers * 100, darkened.mean() * 100, spread, hard, real, wide))
