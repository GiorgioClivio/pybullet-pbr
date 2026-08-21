"""Modern renderer for PyBullet's interactive GUI.

Usage:
    import pybullet_pbr; pybullet_pbr.activate()
    import pybullet as p; p.connect(p.GUI)
"""
import importlib.util
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_BUILD = os.path.join(os.path.dirname(_HERE), "build")

__all__ = ["activate", "binary_path", "is_active", "configure", "set_exposure",
           "set_tonemapper", "set_diagnostic", "set_sun", "get_exposure",
           "get_tonemapper", "get_diagnostic", "get_sun", "TONEMAPPERS",
           "set_material", "clear_material", "set_default_material", "load_texture",
           "set_normal_scale", "get_normal_scale", "TEXTURE_SLOTS",
           "set_auto_exposure", "get_auto_exposure", "set_sensor", "get_sensor",
           "set_white_balance", "get_white_balance",
           "get_default_material", "set_shadows", "get_shadows", "ssao", "get_ssao", "DIAGNOSTICS",
           "set_antialiasing", "get_antialiasing", "ANTIALIASING",
           "set_environment", "get_environment", "get_taa_blend",
           "set_sky", "get_sky", "set_sun_color", "get_sun_color",
           "set_scene", "scene_names", "SCENES", "MATERIALS", "material_names",
           "POINT", "SPOT", "add_light", "add_point_light", "add_spot_light",
           "move_light", "update_light", "remove_light", "clear_lights", "get_light",
           "light_count", "max_lights", "attach_light", "detach_light", "sync_lights",
           "look_at", "set_light_shadow_bias", "get_light_shadow_bias", "aim_light", "direction_from_angles",
           "angles_from_direction", "set_ev", "get_ev", "set_camera_exposure",
           "kelvin_to_rgb", "photometry",
           "enable_headless", "is_headless_enabled", "config",
           "set_sun_direction", "get_sun_direction", "set_sun_angular_diameter",
           "get_sun_angular_diameter"]


def binary_path():
    """Path to the locally built pybullet extension, or None if not built."""
    import sysconfig

    suffix = sysconfig.get_config_var("EXT_SUFFIX") or ".so"
    candidate = os.path.join(_BUILD, "pybullet" + suffix)
    return candidate if os.path.exists(candidate) else None


def is_active():
    mod = sys.modules.get("pybullet")
    return mod is not None and getattr(mod, "__file__", None) == binary_path()


def activate():
    """Install the locally built pybullet under the name ``pybullet``.

    Must be called before any ``import pybullet``.
    """
    if is_active():
        return sys.modules["pybullet"]
    if "pybullet" in sys.modules:
        raise RuntimeError(
            "pybullet_pbr.activate() must be called before importing pybullet "
            "(already imported from %r)" % getattr(sys.modules["pybullet"], "__file__", "?"))

    so = binary_path()
    if so is None:
        raise RuntimeError(
            "no local pybullet build for this interpreter in %s - run ./build.sh" % _BUILD)

    spec = importlib.util.spec_from_file_location("pybullet", so)
    module = importlib.util.module_from_spec(spec)
    sys.modules["pybullet"] = module
    try:
        spec.loader.exec_module(module)
    except BaseException:
        del sys.modules["pybullet"]
        raise
    return module


from .config import (TONEMAPPERS, configure, get_default_material, get_diagnostic,  # noqa: E402
                     get_exposure, get_sun, get_tonemapper, set_default_material,
                     set_diagnostic, set_exposure, set_shadows, set_sun, set_ev, get_ev,
                     set_camera_exposure, set_sun_direction, get_sun_direction,
                     set_sun_angular_diameter, get_sun_angular_diameter,
                     set_normal_scale, get_normal_scale, set_auto_exposure,
                     get_auto_exposure, set_sensor, get_sensor,
                     set_white_balance, get_white_balance,
                     set_tonemapper, get_shadows, ssao, get_ssao, DIAGNOSTICS,
                     set_antialiasing, get_antialiasing, ANTIALIASING,
                     set_environment, get_environment, get_taa_blend,
                     set_sky, get_sky, set_sun_color, get_sun_color)
from .materials import (MATERIALS, TEXTURE_SLOTS, clear_material,  # noqa: E402
                        load_texture, material_names, set_material)
from .scenes import SCENES, scene_names, set_scene  # noqa: E402
from .lights import (POINT, SPOT, add_light, add_point_light, add_spot_light,  # noqa: E402
                     attach_light, clear_lights, detach_light, get_light, light_count,
                     look_at, max_lights, move_light, remove_light, set_light_shadow_bias,
                     get_light_shadow_bias,
                     sync_lights, update_light, aim_light, direction_from_angles,
                     angles_from_direction)
from . import config, photometry  # noqa: E402
from .headless import enable_headless, is_headless_enabled  # noqa: E402
from .photometry import kelvin_to_rgb  # noqa: E402
