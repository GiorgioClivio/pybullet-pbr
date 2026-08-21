"""Physical light units: colour temperature, luminous flux, and camera exposure.

The renderer works in real photometric units — illuminance in lux and luminance in
cd/m² — so a light you specify in lumens behaves like that lamp would, and a scene lit
by 100 000 lux daylight is genuinely 10 000x brighter than one lit by a 10 lux corridor.
Nothing would be visible without a matching camera exposure, so exposure is photographic
too: aperture, shutter and ISO, or an EV100 value directly.

    pybullet_pbr.set_camera_exposure(aperture=16, shutter=1/125, iso=100)   # sunny 16
    pybullet_pbr.set_ev(15)                                                 # the same thing

This is the same model film cameras, Isaac Sim and Blender use, and it is what makes a
simulated camera's response to a scene meaningful rather than arbitrary.
"""
import math

#: Renderer units per cd/m². Kept at 1 - the shading maths is already in physical units,
#: and exposure does the scaling. Exposed because it documents the convention.
RENDER_UNITS_PER_NIT = 1.0

#: Standard illuminance references, in lux, for picking sun values.
DAYLIGHT_LUX = 100000.0     # clear midday sun on Earth's surface
MOON_SURFACE_LUX = 128000.0  # solar constant with no atmosphere to absorb it
MARS_NOON_LUX = 55000.0      # Earth's, at 1/2.3 the solar irradiance, minus dust
FULL_MOON_LUX = 0.25         # moonlight on Earth
OFFICE_LUX = 500.0


def kelvin_to_rgb(kelvin):
    """Colour of a blackbody at ``kelvin``, as linear RGB normalised to luminance 1.

    Uses Krystek's approximation of the Planckian locus in CIE 1960 uv, which is
    accurate over the 1000-15000 K range that covers every practical light source:
    candle flame ~1800 K, tungsten ~2700 K, daylight ~5500 K, overcast sky ~7000 K.
    """
    kelvin = max(1000.0, min(15000.0, float(kelvin)))

    # Planckian locus in CIE 1960 (u, v)
    k2 = kelvin * kelvin
    u = (0.860117757 + 1.54118254e-4 * kelvin + 1.28641212e-7 * k2) / \
        (1.0 + 8.42420235e-4 * kelvin + 7.08145163e-7 * k2)
    v = (0.317398726 + 4.22806245e-5 * kelvin + 4.20481691e-8 * k2) / \
        (1.0 - 2.89741816e-5 * kelvin + 1.61456053e-7 * k2)

    denominator = 2.0 * u - 8.0 * v + 4.0
    x = 3.0 * u / denominator
    y = 2.0 * v / denominator
    z = 1.0 - x - y

    # xyY (Y = 1) -> XYZ -> linear sRGB
    big_y = 1.0
    big_x = (x / y) * big_y
    big_z = (z / y) * big_y
    r = 3.2404542 * big_x - 1.5371385 * big_y - 0.4985314 * big_z
    g = -0.9692660 * big_x + 1.8760108 * big_y + 0.0415560 * big_z
    b = 0.0556434 * big_x - 0.2040259 * big_y + 1.0572252 * big_z

    rgb = [max(0.0, c) for c in (r, g, b)]
    # Normalise to luminance 1 so kelvin sets colour and lumens sets brightness,
    # independently - otherwise every temperature change would also change exposure.
    luminance = 0.2126 * rgb[0] + 0.7152 * rgb[1] + 0.0722 * rgb[2]
    if luminance > 1e-6:
        rgb = [c / luminance for c in rgb]
    return tuple(rgb)


def lumens_to_candela(lumens):
    """Luminous flux to luminous intensity, spread over the whole sphere.

    Spot lights use the same conversion as point lights on purpose: narrowing a beam
    then does not silently brighten it, so you can frame a shot without re-balancing
    the exposure. (Unreal Engine makes the same choice.)
    """
    return float(lumens) / (4.0 * math.pi)


def candela_to_lumens(candela):
    return float(candela) * 4.0 * math.pi


def ev100(aperture=16.0, shutter=1.0 / 125.0, iso=100.0):
    """Exposure value at ISO 100 for a set of camera settings.

    The defaults are the "sunny 16" rule: f/16 at 1/125 s, ISO 100, which correctly
    exposes a subject in direct midday sun.
    """
    return math.log2((aperture * aperture) / float(shutter) * 100.0 / float(iso))


def ev_to_exposure(ev):
    """Linear multiplier that maps scene luminance to display range at this EV.

    The 1.2 is the standard saturation-based calibration constant, the same one film
    and game engines use, so 18% grey under correctly exposed light lands at mid grey.
    """
    return 1.0 / (1.2 * (2.0 ** float(ev)))


def exposure_to_ev(exposure):
    return math.log2(1.0 / (1.2 * float(exposure)))


def ev_for_illuminance(lux, albedo=0.18):
    """EV100 that correctly exposes a surface of ``albedo`` receiving ``lux``.

    The counterpart of picking a scene's exposure by hand: give it the light you
    actually have and it returns the camera setting that renders it as mid grey.
    """
    luminance = max(float(lux), 1e-9) * float(albedo) / math.pi
    return math.log2(max(luminance, 1e-9) / (1.2 * 0.18))


def ev_for_light(lumens, distance, albedo=0.18):
    """EV100 that exposes for a lamp of ``lumens`` lighting a surface ``distance`` away.

    Use it when a scene's own exposure is metered for ambient light the robot does not
    have - a permanently shadowed region, say, where the camera must expose for the
    rover's own headlight.
    """
    return ev_for_illuminance(illuminance_at(lumens, distance), albedo)


def illuminance_at(lumens, distance):
    """Lux falling on a surface facing a point light of ``lumens`` at ``distance``."""
    distance = max(float(distance), 1e-6)
    return lumens_to_candela(lumens) / (distance * distance)
