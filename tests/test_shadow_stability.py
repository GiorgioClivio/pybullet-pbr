"""Shadow stability: a moving camera target must not make static shadows shimmer.

The GUI's sun shadow map is centred on the debug camera target. A follow camera moves
that target every frame; if the shadow frustum slides by fractions of a texel, the
rasterised edges of static shadows change from frame to frame (flicker).

getCameraImage cannot show this: it renders with a temporary camera whose target is
the origin. So this records the real GUI framebuffer (STATE_LOGGING_VIDEO_MP4, needs
ffmpeg). The camera *position* and view direction stay fixed while the target slides
along the view ray, so the image is unchanged except through the shadow map.

    python3 tests/test_shadow_stability.py   # prints OK: ... or raises AssertionError

Measured 2026-10-19 (1024 px over 12 m, hard PCF): without texel snapping 14-32 px of
~730 shadow px changed per sub-texel step; with snapping 1-4 px.
"""
import os
import subprocess
import sys
import tempfile
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import pybullet_pbr

pybullet_pbr.activate()

import numpy as np
import pybullet as p
import pybullet_data

RESOLUTION, WORLD_SIZE = 1024, 6             # a typical fast interactive setting
TEXEL = 2 * WORLD_SIZE / RESOLUTION
YAW, PITCH, DISTANCE = 40, -45, 4.5
W, H = 640, 480

p.connect(p.GUI, options=f"--width={W} --height={H}")
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
p.configureDebugVisualizer(shadowMapResolution=RESOLUTION, shadowMapWorldSize=WORLD_SIZE)
pybullet_pbr.set_scene("earth_noon")
pybullet_pbr.set_shadows(softness=0)          # hard edges show shimmer most
pybullet_pbr.set_antialiasing("fxaa")         # TAA would blend frames and hide it
p.loadURDF("plane.urdf")
for x, y, yaw in [(0.0, 0.0, 0.3), (1.2, -0.8, 0.7), (-1.0, 0.9, 1.1)]:
    box = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.3, 0.12, 0.25])
    p.createMultiBody(0, box, basePosition=[x, y, 0.25],
                      baseOrientation=p.getQuaternionFromEuler([0, 0, yaw]))
# Unit vector from camera to target for this yaw/pitch (PyBullet's convention).
view = np.array(p.computeViewMatrixFromYawPitchRoll([0, 0, 0], DISTANCE, YAW, PITCH, 0, 2)).reshape(4, 4)
forward = -view[:3, 2]                        # view matrix is column-major: row 2 of R
camera = -forward * DISTANCE


def hold(along, seconds=0.6):
    """Keep the camera position fixed; slide the target `along` metres down the view ray."""
    target = camera + forward * (DISTANCE + along)
    p.resetDebugVisualizerCamera(DISTANCE + along, YAW, PITCH, target.tolist())
    end = time.time() + seconds  # the GUI renders asynchronously (single-step rendering
    while time.time() < end:     # stops the video logger), so hold for wall time
        p.stepSimulation()
        time.sleep(1 / 120)


def record(alongs, attempts=4):
    """Changed-pixel count per recorded frame versus the first held frame.

    A window that the compositor throttles can record only a few frames; retry then.
    """
    for _ in range(attempts):
        counts = record_once(alongs)
        if counts is not None:
            return counts
    raise AssertionError("GUI recording captured too few frames in %d attempts" % attempts)


def record_once(alongs):
    path = os.path.join(tempfile.mkdtemp(), "frames.mp4")
    hold(0.0, 1.0)  # let the window settle; first frames can be blank
    log = p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, path)
    hold(0.0)
    for along in alongs:
        hold(along)
    p.stopStateLogging(log)
    time.sleep(1.0)
    raw = subprocess.run(["ffmpeg", "-loglevel", "error", "-i", path, "-f", "rawvideo",
                          "-pix_fmt", "gray", "-"], capture_output=True, check=True).stdout
    frames = np.frombuffer(raw, np.uint8).reshape(-1, H, W).astype(np.int16)
    if len(frames) < 30:
        return None
    return [int((np.abs(f - frames[5]) > 25).sum()) for f in frames[5:]]


# Sub-texel target motion: 0.37-texel steps, as a walking robot's follow camera makes.
sub = record([0.37 * k * TEXEL for k in range(1, 9)])
# Sanity: moving the target by more than the shadow map must change the shadows,
# otherwise the recording does not exercise the shadow frustum at all.
far = record([3 * WORLD_SIZE])
print("changed px per frame (every 6th): sub-texel", sub[::6], "far", far[::6], flush=True)
assert far and max(far) > 300, "target motion does not reach the shadow map (max %d px)" % max(far)
worst = max(sub)
assert worst < 8, "static shadows shimmer under sub-texel target motion: up to %d px changed" % worst
print("OK: static shadows stable under sub-texel camera-target motion "
      "(max %d px changed; far-shift sanity %d px)" % (worst, max(far)))
