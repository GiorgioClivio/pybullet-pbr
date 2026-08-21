"""Renderer settings.

The renderer lives inside the locally built pybullet extension; ctypes reaches its
exported ``pbr_*`` setters directly, so no PyBullet API is extended. Settings can
be changed at any time and take effect on the next frame.
"""
import ctypes
import os

TONEMAPPERS = {"none": 0, "reinhard": 1, "aces": 2}

# The renderer's settings live in process globals, and there is one copy per loaded
# extension: pybullet's, and the headless eglRenderer plugin's. Setters therefore go to
# every attached library and are journaled, so a library attached later (which is what
# enable_headless() does) can be brought up to the current state. Queries only ever ask
# the first one.
_libs = []
_journal = []

_SIGNATURES = (
    ("pbr_set_exposure", [ctypes.c_float], None),
    ("pbr_get_exposure", [], ctypes.c_float),
    ("pbr_set_tonemapper", [ctypes.c_int], None),
    ("pbr_get_tonemapper", [], ctypes.c_int),
    ("pbr_set_diagnostic", [ctypes.c_int], None),
    ("pbr_get_diagnostic", [], ctypes.c_int),
    ("pbr_set_sun_intensity", [ctypes.c_float], None),
    ("pbr_get_sun_intensity", [], ctypes.c_float),
    ("pbr_set_ambient_intensity", [ctypes.c_float], None),
    ("pbr_get_ambient_intensity", [], ctypes.c_float),
    ("pbr_set_default_material", [ctypes.c_float, ctypes.c_float], None),
    ("pbr_get_default_metallic", [], ctypes.c_float),
    ("pbr_get_default_roughness", [], ctypes.c_float),
    ("pbr_set_shadow_params", [ctypes.c_float] * 4, None),
    ("pbr_get_shadow_bias", [], ctypes.c_float),
    ("pbr_get_shadow_slope_bias", [], ctypes.c_float),
    ("pbr_get_shadow_softness", [], ctypes.c_float),
    ("pbr_get_shadow_strength", [], ctypes.c_float),
    ("pbr_set_ssao", [ctypes.c_int, ctypes.c_float, ctypes.c_float, ctypes.c_float], None),
    ("pbr_get_ssao_enabled", [], ctypes.c_int),
    ("pbr_get_ssao_radius", [], ctypes.c_float),
    ("pbr_get_ssao_strength", [], ctypes.c_float),
    ("pbr_get_ssao_bias", [], ctypes.c_float),
    ("pbr_set_environment", [ctypes.c_char_p, ctypes.c_float, ctypes.c_int], None),
    ("pbr_get_ibl_enabled", [], ctypes.c_int),
    ("pbr_get_ibl_strength", [], ctypes.c_float),
    ("pbr_get_show_background", [], ctypes.c_int),
    ("pbr_get_environment_path", [], ctypes.c_char_p),
    ("pbr_set_sun_color", [ctypes.c_float] * 3, None),
    ("pbr_set_sun_direction", [ctypes.c_float] * 3, None),
    ("pbr_get_sun_direction", [ctypes.POINTER(ctypes.c_float)], None),
    ("pbr_set_sun_angular_diameter", [ctypes.c_float], None),
    ("pbr_get_sun_angular_diameter", [], ctypes.c_float),
    ("pbr_get_sun_color", [ctypes.POINTER(ctypes.c_float)], None),
    ("pbr_set_sky", [ctypes.POINTER(ctypes.c_float)] * 3 + [ctypes.c_float, ctypes.c_int], None),
    ("pbr_get_sky", [ctypes.POINTER(ctypes.c_float)], None),
    ("pbr_set_light", [ctypes.c_int, ctypes.c_int] + [ctypes.POINTER(ctypes.c_float)] * 3
     + [ctypes.c_float] * 5 + [ctypes.c_int], ctypes.c_int),
    ("pbr_move_light", [ctypes.c_int] + [ctypes.POINTER(ctypes.c_float)] * 2, None),
    ("pbr_remove_light", [ctypes.c_int], None),
    ("pbr_clear_lights", [], None),
    ("pbr_get_light_count", [], ctypes.c_int),
    ("pbr_get_max_lights", [], ctypes.c_int),
    ("pbr_get_light", [ctypes.c_int, ctypes.POINTER(ctypes.c_float)], ctypes.c_int),
    ("pbr_set_light_shadow_bias", [ctypes.c_float], None),
    ("pbr_get_light_shadow_bias", [], ctypes.c_float),
    ("pbr_register_texture", [ctypes.c_char_p, ctypes.c_int], ctypes.c_int),
    ("pbr_texture_count", [], ctypes.c_int),
    ("pbr_set_normal_scale", [ctypes.c_float], None),
    ("pbr_get_normal_scale", [], ctypes.c_float),
    ("pbr_set_material_by_body",
     [ctypes.c_int, ctypes.c_int, ctypes.c_float, ctypes.c_float, ctypes.c_float],
     ctypes.c_int),
    ("pbr_instance_map_size", [], ctypes.c_int),
    ("pbr_set_antialiasing", [ctypes.c_int], None),
    ("pbr_get_antialiasing", [], ctypes.c_int),
    ("pbr_set_auto_exposure",
     [ctypes.c_int, ctypes.c_float, ctypes.c_float, ctypes.c_float], None),
    ("pbr_get_auto_exposure", [], ctypes.c_int),
    ("pbr_get_exposure_key", [], ctypes.c_float),
    ("pbr_set_sensor", [ctypes.c_float] * 5, None),
    ("pbr_get_sensor", [ctypes.POINTER(ctypes.c_float)], None),
    ("pbr_set_white_balance", [ctypes.c_float] * 3, None),
    ("pbr_get_white_balance", [ctypes.POINTER(ctypes.c_float)], None),
    ("pbr_set_taa_blend", [ctypes.c_float], None),
    ("pbr_get_taa_blend", [], ctypes.c_float),
)

# Every call that changes renderer state, listed rather than inferred from the name:
# pbr_register_texture is a mutator despite not being called "set", and getting that
# wrong means a second renderer silently misses it.
_MUTATORS = frozenset(
    [name for name, _, _ in _SIGNATURES if name.startswith("pbr_set")]
    + ["pbr_move_light", "pbr_remove_light", "pbr_clear_lights", "pbr_register_texture"])


def _open(path):
    lib = ctypes.CDLL(path)
    for name, argtypes, restype in _SIGNATURES:
        fn = getattr(lib, name)
        fn.argtypes = argtypes
        fn.restype = restype
    return lib


class _Renderers(object):
    """Dispatches setters to every attached renderer, queries to the first."""

    def __getattr__(self, name):
        def call(*args):
            _ensure_primary()
            if name not in _MUTATORS:
                return getattr(_libs[0], name)(*args)
            # A light with slot -1 picks the next free slot; resolve it once so every
            # library ends up with the same slot for the same light.
            if name == "pbr_set_light" and args[0] < 0:
                resolved = getattr(_libs[0], name)(*args)
                args = (resolved,) + args[1:]
                for lib in _libs[1:]:
                    getattr(lib, name)(*args)
                _journal.append((name, args))
                return resolved
            result = None
            for index, lib in enumerate(_libs):
                value = getattr(lib, name)(*args)
                if index == 0:
                    result = value
            _journal.append((name, args))
            return result

        return call


_multi = _Renderers()


def _ensure_primary():
    if not _libs:
        from . import binary_path
        so = binary_path()
        if so is None:
            raise RuntimeError("no local pybullet build - run ./build.sh")
        _libs.append(_open(so))


def attach_library(path):
    """Bring another renderer library under the same settings.

    Used by enable_headless() for the EGL plugin, which is a separate extension with
    its own copy of the renderer state.
    """
    _ensure_primary()   # the main extension must stay first: queries go to _libs[0]
    lib = _open(path)
    _libs.append(lib)
    for name, args in _journal:
        getattr(lib, name)(*args)
    return lib


def _native():
    _ensure_primary()
    return _multi


def set_exposure(value):
    """Raw linear exposure multiplier applied before tone mapping.

    Prefer :func:`set_ev` or :func:`set_camera_exposure`: the renderer works in real
    photometric units, so a physically meaningful exposure is a photographic one.
    """
    _native().pbr_set_exposure(float(value))


def set_ev(ev):
    """Set exposure from an EV100 value (higher = darker image).

    Reference points: 15 is direct midday sun, 12 an overcast day, 8 a bright room,
    3 a candlelit room, -1 a full-moon night.
    """
    from .photometry import ev_to_exposure
    _native().pbr_set_exposure(ev_to_exposure(ev))


def get_ev():
    from .photometry import exposure_to_ev
    return exposure_to_ev(_native().pbr_get_exposure())


def set_camera_exposure(aperture=16.0, shutter=1.0 / 125.0, iso=100.0):
    """Set exposure the way a camera is set: f-number, shutter in seconds, ISO.

    The defaults are the "sunny 16" rule, which correctly exposes direct midday sun.
    """
    from .photometry import ev100
    set_ev(ev100(aperture, shutter, iso))


def get_exposure():
    return _native().pbr_get_exposure()


def set_tonemapper(name):
    """One of 'none', 'reinhard', 'aces'."""
    try:
        _native().pbr_set_tonemapper(TONEMAPPERS[name])
    except KeyError:
        raise ValueError("unknown tonemapper %r, expected one of %s"
                         % (name, sorted(TONEMAPPERS)))


def get_tonemapper():
    value = _native().pbr_get_tonemapper()
    return next(k for k, v in TONEMAPPERS.items() if v == value)


DIAGNOSTICS = {False: 0, None: 0, "off": 0, True: 1, "normals": 1, "ao": 2}


def set_diagnostic(mode):
    """Debug views: False/'off', True/'normals' (world normals as RGB), or 'ao'."""
    try:
        value = DIAGNOSTICS[mode]
    except (KeyError, TypeError):
        raise ValueError("unknown diagnostic %r, expected one of %s"
                         % (mode, ["off", "normals", "ao"]))
    _native().pbr_set_diagnostic(value)


def get_diagnostic():
    return {0: "off", 1: "normals", 2: "ao"}[_native().pbr_get_diagnostic()]


DEFAULT_WHITE_BALANCE_KELVIN = 5500.0


def set_white_balance(kelvin=None, rgb=None):
    """Balance the camera for an illuminant, as a colour temperature.

    Lights here are physical, so a 5500 K sun really is warmer than sRGB's 6500 K white
    point and a sunlit surface really does render warm. A camera cancels that by
    dividing through the colour of the illuminant it is balanced for. Balanced to the
    sun, sunlit surfaces come out neutral while everything else keeps its true relative
    tint: the sky stays blue, a tungsten lamp stays warm.

    Scene presets balance for their own sun. Pass ``kelvin=None, rgb=None`` for no
    correction at all, which is the raw physical image.
    """
    if rgb is None:
        from .photometry import kelvin_to_rgb
        rgb = (1.0, 1.0, 1.0) if kelvin is None else kelvin_to_rgb(kelvin)
    _native().pbr_set_white_balance(float(rgb[0]), float(rgb[1]), float(rgb[2]))


def get_white_balance():
    buf = (ctypes.c_float * 3)()
    _native().pbr_get_white_balance(buf)
    return tuple(buf)


def set_auto_exposure(enabled=True, key=0.18, min_ev=-6.0, max_ev=20.0):
    """Let the camera meter the scene itself, the way a real one does.

    ``key`` is the average luminance it aims for (0.18 is mid grey). The result is
    clamped between ``min_ev`` and ``max_ev``, which is how you model a sensor that
    simply cannot expose for a permanently shadowed region.

    Metering is instantaneous - there is no adaptation delay yet, so a lamp swinging
    into frame changes the exposure on the next frame rather than over a second.
    """
    _native().pbr_set_auto_exposure(int(bool(enabled)), float(key), float(min_ev),
                                    float(max_ev))


def get_auto_exposure():
    return {"enabled": bool(_native().pbr_get_auto_exposure()),
            "key": _native().pbr_get_exposure_key()}


def set_sensor(bloom=None, bloom_threshold=None, vignette=None, noise=None,
               motion_blur=None):
    """Camera imperfections, for testing vision against something less than perfect.

    ``bloom`` is how much bright light spills into its surroundings, above
    ``bloom_threshold`` cd/m². ``vignette`` darkens the corners. ``noise`` adds
    shot-noise-like grain that is strongest in the dark parts of the frame, as on a
    real sensor. ``motion_blur`` is the shutter opening as a fraction of the frame
    time, smeared along each pixel's true screen-space motion.
    """
    current = get_sensor()
    _native().pbr_set_sensor(
        float(current["bloom"] if bloom is None else bloom),
        float(current["bloom_threshold"] if bloom_threshold is None else bloom_threshold),
        float(current["vignette"] if vignette is None else vignette),
        float(current["noise"] if noise is None else noise),
        float(current["motion_blur"] if motion_blur is None else motion_blur))


def get_sensor():
    buf = (ctypes.c_float * 5)()
    _native().pbr_get_sensor(buf)
    return {"bloom": buf[0], "bloom_threshold": buf[1], "vignette": buf[2],
            "noise": buf[3], "motion_blur": buf[4]}


def set_normal_scale(scale):
    """How strongly normal maps perturb the surface. 1 is the map as authored."""
    _native().pbr_set_normal_scale(float(scale))


def get_normal_scale():
    return _native().pbr_get_normal_scale()


def set_sun_direction(direction=None, elevation=None, azimuth=None):
    """Where the sun is, as a direction vector or as elevation/azimuth in degrees.

    Elevation is degrees above the horizon (negative puts it below), azimuth degrees
    around from +X. This is set on the renderer directly, so it works headless as well
    as in the GUI.
    """
    if direction is None:
        import math
        el = math.radians(0.0 if elevation is None else elevation)
        az = math.radians(0.0 if azimuth is None else azimuth)
        horizontal = math.cos(el)
        direction = (horizontal * math.cos(az), horizontal * math.sin(az), math.sin(el))
    x, y, z = direction
    _native().pbr_set_sun_direction(float(x), float(y), float(z))


def get_sun_direction():
    buf = (ctypes.c_float * 3)()
    _native().pbr_get_sun_direction(buf)
    return tuple(buf)


def set_sun_angular_diameter(degrees):
    """Angular size of the sun's disc, which sets how fast shadows soften with distance.

    0.53 degrees as seen from Earth or the Moon, 0.35 from Mars. 0 makes an idealised
    point sun with uniformly hard shadows.
    """
    _native().pbr_set_sun_angular_diameter(float(degrees))


def get_sun_angular_diameter():
    return _native().pbr_get_sun_angular_diameter()


def set_sun(intensity=None, ambient=None, lux=None, kelvin=None):
    """The directional sun.

    ``lux`` is illuminance on a surface facing it - 100 000 for clear midday sun on
    Earth, 128 000 on the Moon (no atmosphere), around 55 000 on Mars. ``kelvin`` sets
    its colour temperature. ``intensity`` is the same quantity as ``lux`` and stays for
    code written before the units were physical; ``ambient`` is the flat fallback used
    only when no environment is active.
    """
    if lux is not None:
        _native().pbr_set_sun_intensity(float(lux))
    elif intensity is not None:
        _native().pbr_set_sun_intensity(float(intensity))
    if kelvin is not None:
        from .photometry import kelvin_to_rgb
        set_sun_color(kelvin_to_rgb(kelvin))
    if ambient is not None:
        _native().pbr_set_ambient_intensity(float(ambient))


def get_sun():
    return _native().pbr_get_sun_intensity(), _native().pbr_get_ambient_intensity()


def set_default_material(metallic=0.0, roughness=0.6):
    """Material used by every shape that has no explicit set_material() call."""
    _native().pbr_set_default_material(float(metallic), float(roughness))


def get_default_material():
    return _native().pbr_get_default_metallic(), _native().pbr_get_default_roughness()


ANTIALIASING = {"off": 0, "fxaa": 1, "taa": 2}

_DEFAULT_SKY = object()


def set_environment(path=_DEFAULT_SKY, strength=1.0, background=None):
    """Image-based lighting from an environment map.

    * ``set_environment()`` - the built-in procedural sky (the default).
    * ``set_environment("studio.hdr", strength=0.4)`` - an equirectangular HDR file.
    * ``set_environment(None)`` - no environment lighting at all, falling back to the
      flat ambient term. Useful when indirect light should be almost absent.

    ``background`` draws the environment behind the scene so the sky you see is the
    sky the objects are lit by; it defaults to on whenever an environment is active.
    The IBL textures are rebuilt on the next frame, which takes a moment; after that
    it costs nothing per frame.
    """
    if path is None:
        strength = 0.0
        path = ""
    elif path is _DEFAULT_SKY:
        path = ""
    else:
        path = os.path.abspath(os.path.expanduser(str(path)))
        if not os.path.exists(path):
            raise FileNotFoundError(path)
    if background is None:
        background = strength > 0.0
    _native().pbr_set_environment(path.encode("utf-8"), float(strength), int(bool(background)))


def get_environment():
    n = _native()
    raw = n.pbr_get_environment_path()
    return {"path": (raw.decode("utf-8") or None) if raw else None,
            "enabled": bool(n.pbr_get_ibl_enabled()),
            "strength": n.pbr_get_ibl_strength(),
            "background": bool(n.pbr_get_show_background())}


def set_antialiasing(mode, blend=None):
    """One of 'off', 'fxaa' or 'taa'.

    TAA jitters the projection sub-pixel and accumulates frames, reprojecting the
    previous one with per-object motion vectors; ``blend`` (0..1, default 0.9) is how
    much of that history a pixel keeps. It resolves edges better than FXAA and is
    stable on a still camera, but needs the scene to be animating - in a paused
    simulation there is nothing new to accumulate.
    """
    try:
        value = ANTIALIASING[mode]
    except (KeyError, TypeError):
        raise ValueError("unknown antialiasing %r, expected one of %s"
                         % (mode, sorted(ANTIALIASING)))
    if blend is not None:
        _native().pbr_set_taa_blend(float(blend))
    _native().pbr_set_antialiasing(value)


def get_antialiasing():
    value = _native().pbr_get_antialiasing()
    return next(k for k, v in ANTIALIASING.items() if v == value)


def get_taa_blend():
    return _native().pbr_get_taa_blend()


def ssao(enabled=None, radius=None, strength=None, bias=None):
    """Screen-space ambient occlusion.

    ``radius`` is the sampling hemisphere radius in world units (metres), ``strength``
    scales the effect, ``bias`` keeps flat surfaces from occluding themselves. AO is
    applied to the ambient term only, so it darkens contact regions without touching
    direct sunlight.
    """
    cur = get_ssao()
    _native().pbr_set_ssao(
        int(cur["enabled"] if enabled is None else bool(enabled)),
        float(cur["radius"] if radius is None else radius),
        float(cur["strength"] if strength is None else strength),
        float(cur["bias"] if bias is None else bias))


def get_ssao():
    n = _native()
    return {"enabled": bool(n.pbr_get_ssao_enabled()), "radius": n.pbr_get_ssao_radius(),
            "strength": n.pbr_get_ssao_strength(), "bias": n.pbr_get_ssao_bias()}


def set_sun_color(color):
    """Colour of the directional sun, as (r, g, b). Intensity is separate."""
    r, g, b = color
    _native().pbr_set_sun_color(float(r), float(g), float(b))


def get_sun_color():
    buf = (ctypes.c_float * 3)()
    _native().pbr_get_sun_color(buf)
    return tuple(buf)


def set_sky(zenith, horizon, ground, glow=0.01, airless=False):
    """Appearance of the procedural sky, which is also what lights the scene.

    ``airless`` gives an airless body: black sky, and the only indirect light is what
    the ground bounces, so shadows go nearly black instead of filling with sky blue.
    Changing any of this rebuilds the IBL textures, so the light always matches the sky.
    """
    def vec(v):
        return (ctypes.c_float * 3)(*[float(c) for c in v])

    _native().pbr_set_sky(vec(zenith), vec(horizon), vec(ground), float(glow),
                          int(bool(airless)))


def get_sky():
    buf = (ctypes.c_float * 11)()
    _native().pbr_get_sky(buf)
    return {"zenith": tuple(buf[0:3]), "horizon": tuple(buf[3:6]), "ground": tuple(buf[6:9]),
            "glow": buf[9], "airless": bool(buf[10])}


def set_shadows(bias=None, slope_bias=None, softness=None, strength=None):
    """Tune the directional shadow map.

    ``bias``/``slope_bias`` are in world units (metres): a constant offset plus one
    proportional to the surface slope, which is what stops grazing-lit surfaces from
    shadowing themselves. ``softness`` is the PCF kernel radius in shadow-map texels.
    ``strength`` scales the shadowing, 1 removing all direct sun.

    Shadows are switched on/off with PyBullet itself
    (``p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0)``), and its
    ``shadowMapResolution``/``shadowMapWorldSize`` still apply. Its ``shadowMapIntensity``
    does not - use ``strength`` here.
    """
    cur = get_shadows()
    _native().pbr_set_shadow_params(
        float(cur["bias"] if bias is None else bias),
        float(cur["slope_bias"] if slope_bias is None else slope_bias),
        float(cur["softness"] if softness is None else softness),
        float(cur["strength"] if strength is None else strength))


def get_shadows():
    n = _native()
    return {"bias": n.pbr_get_shadow_bias(), "slope_bias": n.pbr_get_shadow_slope_bias(),
            "softness": n.pbr_get_shadow_softness(), "strength": n.pbr_get_shadow_strength()}


def configure(exposure=None, tonemapper=None, diagnostic=None, sun=None, ambient=None,
              metallic=None, roughness=None, antialiasing=None, environment=_DEFAULT_SKY,
              environment_strength=None):
    """Set several options at once; None leaves an option unchanged.

    ``metallic``/``roughness`` here set the *defaults* for shapes without an
    explicit material; use pybullet_pbr.set_material() for a single body.
    """
    if exposure is not None:
        set_exposure(exposure)
    if tonemapper is not None:
        set_tonemapper(tonemapper)
    if diagnostic is not None:
        set_diagnostic(diagnostic)
    if sun is not None or ambient is not None:
        set_sun(sun, ambient)
    if antialiasing is not None:
        set_antialiasing(antialiasing)
    if environment is not _DEFAULT_SKY or environment_strength is not None:
        cur = get_environment()
        if environment is not _DEFAULT_SKY:
            path = environment
        else:
            # None here means the procedural sky, not "switch the environment off"
            path = cur["path"] if cur["path"] else _DEFAULT_SKY
        set_environment(path, cur["strength"] if environment_strength is None
                        else environment_strength)
    if metallic is not None or roughness is not None:
        cur_m, cur_r = get_default_material()
        set_default_material(cur_m if metallic is None else metallic,
                             cur_r if roughness is None else roughness)
