"""Lighting environments for robotics scenarios.

    pybullet_pbr.set_scene("moon_polar")

Each preset sets the sun (direction, colour, intensity), the sky, how much indirect
light there is, and the exposure. They are starting points chosen to be *physically
plausible* rather than exact radiometry: solar irradiance falls off with distance from
the sun, airless bodies have black skies and near-black shadows, and atmospheres tint
both the sky and the sun near the horizon.

Every value can be overridden afterwards with the individual setters, or per call:

    pybullet_pbr.set_scene("mars_day", exposure=1.1)
"""
import math

from .photometry import DAYLIGHT_LUX, MARS_NOON_LUX, MOON_SURFACE_LUX, kelvin_to_rgb

_EARTH_SUN = DAYLIGHT_LUX
_MOON_SUN = MOON_SURFACE_LUX
_MARS_SUN = MARS_NOON_LUX

#: Chromaticities above are relative; the lower hemisphere of the sky carries ground
#: bounce, and on Earth it is kept neutral. Tinting it warm puts a warm cast on every
#: surface in the scene, since bounce reaches the parts direct sun does not.
#:
#: Colour temperature the camera is balanced for unless a scene says otherwise. 5500 K
#: is the "daylight" setting on a real camera: noon renders neutral, a sunset stays
#: warm, and a tungsten lamp at night stays warm relative to it.
DAYLIGHT_BALANCE_KELVIN = 5500.0

SCENES = {
    # --- Earth ----------------------------------------------------------------
    "earth_noon": dict(
        sun_elevation=78, sun_azimuth=140, sun_kelvin=5800, sun=_EARTH_SUN * 1.1,
        sky_luminance=8000.0, zenith=(0.35, 0.55, 1.00), horizon=(0.85, 0.92, 1.00),
        ground=(0.28, 0.28, 0.28), glow=0.02, airless=False, ibl=1.0, ev=14.8,
        shadow_softness=1.2,
        description="Overhead sun, short hard shadows, bright blue sky.",
    ),
    "earth_day": dict(
        sun_elevation=45, sun_azimuth=140, sun_kelvin=5500, sun=_EARTH_SUN,
        sky_luminance=6000.0, zenith=(0.38, 0.58, 1.00), horizon=(0.88, 0.94, 1.00),
        ground=(0.28, 0.28, 0.28), glow=0.02, airless=False, ibl=1.0, ev=14.2,
        shadow_softness=1.5,
        description="Mid-morning daylight. The general-purpose outdoor default.",
    ),
    "earth_sunset": dict(
        sun_elevation=4, sun_azimuth=95, sun_kelvin=2000, sun=_EARTH_SUN * 0.12,
        sky_luminance=900.0, zenith=(0.22, 0.32, 0.80), horizon=(1.30, 0.72, 0.36),
        ground=(0.25, 0.20, 0.16), glow=0.30, airless=False, ibl=1.0, ev=12.0,
        shadow_softness=2.2,
        description="Low warm sun, very long shadows, orange horizon.",
    ),
    "earth_night": dict(
        sun_elevation=38, sun_azimuth=250, sun_kelvin=4100, sun=0.25,
        white_balance=3600,   # under-balanced on purpose: moonlight reads cool
        sky_luminance=0.0015, zenith=(0.45, 0.60, 1.30), horizon=(0.70, 0.80, 1.20),
        ground=(0.30, 0.30, 0.35), glow=0.01, airless=False, ibl=1.0, ev=-1.0,
        shadow_softness=2.0,
        description="Moonlit night: a full moon is 0.25 lux, so the camera works at EV -1.",
    ),
    "earth_indoor": dict(
        sun_elevation=62, sun_azimuth=200, sun_kelvin=4000, sun=300.0,
        sky_luminance=120.0, zenith=(0.95, 0.95, 1.00), horizon=(1.00, 0.98, 0.95),
        ground=(0.74, 0.74, 0.74), glow=0.0, airless=False, ibl=1.0, ev=9.0,
        shadow_softness=3.0,
        description="Closed environment lit to office levels (~500 lux), soft and even.",
    ),
    # --- Moon -----------------------------------------------------------------
    "moon_equatorial": dict(
        white_balance=5800, sun_elevation=70, sun_azimuth=140, sun_kelvin=5800, sun=_MOON_SUN,
        sky_luminance=0.0, zenith=(0.0, 0.0, 0.0), horizon=(0.0, 0.0, 0.0),
        ground=(1.0, 0.95, 0.88), glow=0.0, airless=True, ibl=1.0, ev=15.0,
        shadow_softness=0.8, ground_bounce=3400.0,
        description="High sun, black sky, brutal contrast. Only regolith bounce fills shadows.",
    ),
    "moon_polar": dict(
        white_balance=5800, sun_elevation=3, sun_azimuth=90, sun_kelvin=5800, sun=_MOON_SUN,
        sky_luminance=0.0, zenith=(0.0, 0.0, 0.0), horizon=(0.0, 0.0, 0.0),
        ground=(1.0, 0.95, 0.88), glow=0.0, airless=True, ibl=1.0, ev=11.3,
        shadow_softness=0.8, ground_bounce=190.0,
        description="Grazing sun at the poles: extremely long shadows against a black sky.",
    ),
    "moon_aristarchus": dict(
        white_balance=5800, sun_elevation=38, sun_azimuth=115, sun_kelvin=5800,
        sun=_MOON_SUN, sky_luminance=0.0, zenith=(0.0, 0.0, 0.0),
        horizon=(0.0, 0.0, 0.0), ground=(1.0, 0.97, 0.94), glow=0.0, airless=True,
        ibl=1.0, ev=14.4, shadow_softness=0.8, ground_bounce=4000.0,
        description="Aristarchus Plateau (23.7N, 47.4W): mid-latitude sun, and a surface "
                    "far brighter than the mare around it, so shadows fill more.",
    ),
    "moon_psr": dict(
        white_balance=5800, sun_elevation=-8, sun_azimuth=90, sun_kelvin=5800, sun=0.0,
        sky_luminance=0.0, zenith=(0.0, 0.0, 0.0), horizon=(0.0, 0.0, 0.0),
        ground=(1.0, 0.97, 0.92), glow=0.0, airless=True, ibl=1.0, ev=-2.0,
        shadow_softness=1.0, ground_bounce=0.05,
        description="Permanently shadowed region: no direct sun at all, only faint "
                    "secondary illumination. Sensor-limited, as in reality.",
    ),
    # --- Mars -----------------------------------------------------------------
    "mars_day": dict(
        sun_angular_diameter=0.35, sun_elevation=50, sun_azimuth=140, sun_kelvin=4700, sun=_MARS_SUN,
        sky_luminance=2600.0, zenith=(1.05, 0.82, 0.58), horizon=(1.25, 0.98, 0.70),
        ground=(0.60, 0.40, 0.26), glow=0.10, airless=False, ibl=1.0, ev=13.6,
        shadow_softness=2.5,
        description="Butterscotch dusty sky, weaker sun, shadows filled by scattered dust.",
    ),
    "mars_dusty": dict(
        sun_angular_diameter=0.35, sun_elevation=35, sun_azimuth=140, sun_kelvin=4200, sun=_MARS_SUN * 0.25,
        sky_luminance=1500.0, zenith=(1.10, 0.86, 0.60), horizon=(1.20, 0.98, 0.72),
        ground=(0.62, 0.44, 0.28), glow=0.20, airless=False, ibl=1.4, ev=12.6,
        shadow_softness=4.0,
        description="Dust storm: sun heavily attenuated, light becomes almost ambient.",
    ),
    "mars_sunset": dict(
        sun_angular_diameter=0.35, sun_elevation=3, sun_azimuth=95, sun_kelvin=6200, sun=_MARS_SUN * 0.08,
        sky_luminance=500.0, zenith=(0.90, 0.80, 0.72), horizon=(0.85, 0.90, 1.15),
        ground=(0.45, 0.32, 0.22), glow=0.35, airless=False, ibl=1.0, ev=11.4,
        shadow_softness=3.0,
        description="Martian sunset: dust scatters red forward, so the sky near the sun "
                    "goes blue - the opposite of Earth.",
    ),
}


def scene_names():
    """Every available scene, as name -> one-line description."""
    return {name: cfg["description"] for name, cfg in SCENES.items()}


def _sun_vector(elevation_deg, azimuth_deg, up_axis=2, distance=60.0):
    """Direction to the sun, in PyBullet's world frame."""
    el = math.radians(elevation_deg)
    az = math.radians(azimuth_deg)
    horizontal = math.cos(el)
    if up_axis == 1:  # Y up
        v = (horizontal * math.cos(az), math.sin(el), horizontal * math.sin(az))
    else:             # Z up, PyBullet's default
        v = (horizontal * math.cos(az), horizontal * math.sin(az), math.sin(el))
    return [c * distance for c in v]


def set_scene(name, physicsClientId=0, **overrides):
    """Apply a lighting scenario by name. See SCENES for what is available.

    Any preset field can be overridden: ``set_scene("moon_polar", sun_elevation=1.5)``.
    Call it any time after connecting; it takes effect on the next frame.
    """
    from . import config

    try:
        cfg = dict(SCENES[name])
    except KeyError:
        raise ValueError("unknown scene %r, expected one of %s"
                         % (name, sorted(SCENES)))
    unknown = set(overrides) - set(cfg)
    if unknown:
        raise ValueError("unknown scene setting(s) %s; valid keys are %s"
                         % (sorted(unknown), sorted(k for k in cfg if k != "description")))
    cfg.update(overrides)

    # Sky colours are relative; sky_luminance (cd/m2) is what puts them on the same
    # physical scale as the sun's lux, so the balance between key light and fill is
    # the real one rather than an artistic guess.
    luminance = cfg["sky_luminance"]
    bounce = cfg.get("ground_bounce")
    scale = lambda rgb, k: tuple(float(c) * k for c in rgb)
    config.set_sky(scale(cfg["zenith"], luminance), scale(cfg["horizon"], luminance),
                   scale(cfg["ground"], luminance if bounce is None else bounce),
                   cfg["glow"], cfg["airless"])
    config.set_sun_color(kelvin_to_rgb(cfg["sun_kelvin"]))
    # A fixed daylight balance, the way a camera left on its "daylight" preset behaves.
    # Balancing to each scene's own sun instead would neutralise a sunset - divide a
    # 2000 K key light out of the frame and what is left is the blue sky fill, so the
    # golden hour comes out blue. Cameras do not do that, and neither do we.
    config.set_white_balance(cfg.get("white_balance", DAYLIGHT_BALANCE_KELVIN))
    config.set_sun_direction(elevation=cfg["sun_elevation"], azimuth=cfg["sun_azimuth"])
    config.set_sun_angular_diameter(cfg.get("sun_angular_diameter", 0.53))
    config.set_sun(lux=cfg["sun"])
    config.set_ev(cfg["ev"])
    config.set_shadows(softness=cfg["shadow_softness"])
    config.set_environment(strength=cfg["ibl"])

    # Also mirror the sun onto PyBullet's own light position, so anything else that
    # reads it stays consistent. (The renderer uses the direction set above, which is
    # what makes scenes work headless, where this call does nothing.)
    import sys
    pb = sys.modules.get("pybullet")
    if pb is not None:
        try:
            pb.configureDebugVisualizer(
                lightPosition=_sun_vector(cfg["sun_elevation"], cfg["sun_azimuth"]),
                physicsClientId=physicsClientId)
        except Exception:
            pass  # not connected yet; the caller can re-apply after connect()
    return cfg
