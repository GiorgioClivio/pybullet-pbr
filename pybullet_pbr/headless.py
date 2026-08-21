"""Headless (no window) rendering through the EGL plugin.

    import pybullet_pbr
    pybullet_pbr.activate()

    import pybullet as p
    p.connect(p.DIRECT)
    pybullet_pbr.enable_headless()          # after connect, before rendering

    w, h, rgb, depth, seg = p.getCameraImage(
        640, 480, view, proj, renderer=p.ER_BULLET_HARDWARE_OPENGL)

Without this, ``p.DIRECT`` renders with PyBullet's software TinyRenderer and none of
this package applies. With it, offscreen camera images come from the same PBR pipeline
as the GUI viewport - which is what makes generated datasets match what you see while
developing.

Needs a GPU reachable through EGL. On a headless server that usually means the NVIDIA
driver's libEGL plus ``EGL_PLATFORM=device``, or a virtual display; on a workstation it
works as-is.
"""
import os

_plugin = None


def plugin_path():
    """Path to the locally built eglRenderer extension, or None if it is missing."""
    from . import binary_path
    import sys

    so = binary_path()
    if so is None:
        return None
    tag = "cpython-%d%d" % sys.version_info[:2]
    directory = os.path.dirname(so)
    for name in sorted(os.listdir(directory)):
        if name.startswith("eglRenderer.") and name.endswith(".so") and tag in name:
            return os.path.join(directory, name)
    return None


def enable_headless(physicsClientId=0):
    """Route offscreen rendering through this renderer. Call after connect().

    Returns the plugin id. Safe to call twice; harmless in GUI mode, where
    getCameraImage already uses this renderer.
    """
    global _plugin
    import sys

    p = sys.modules.get("pybullet")
    if p is None:
        raise RuntimeError("import pybullet first (after pybullet_pbr.activate())")

    path = plugin_path()
    if path is None:
        raise RuntimeError("eglRenderer was not built; re-run ./build.sh")

    if _plugin is None:
        _plugin = p.loadPlugin(path, "_eglRendererPlugin", physicsClientId=physicsClientId)
        if _plugin < 0:
            raise RuntimeError(
                "the EGL renderer plugin failed to load. Check that a GPU is reachable "
                "through EGL (on a headless server: NVIDIA's libEGL, or a virtual display)")
        # The plugin is a separate extension with its own copy of the renderer state, so
        # bring it under the same settings as everything else.
        from .config import attach_library
        attach_library(path)
    return _plugin


def is_headless_enabled():
    return _plugin is not None
