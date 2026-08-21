"""Movable point and spot lights that cast real shadows.

    head = pybullet_pbr.add_spot_light(position=[0, 0, 2], yaw=0, pitch=-90,
                                       lumens=3000, kelvin=5000, cone_angle=35)
    pybullet_pbr.attach_light(head, robot, link=-1, offset=[0.3, 0, 0.4])

    while True:
        p.stepSimulation()
        pybullet_pbr.sync_lights()      # keeps attached lights on their body

Lights are specified the way real ones are:

======================  ==================================================
``lumens``              luminous flux, the number on a lamp's box. A car
                        headlight is ~1200 lm, a 60 W bulb ~800 lm, a work
                        floodlight ~10000 lm.
``kelvin``              colour temperature: 1800 candle, 2700 warm white,
                        4000 neutral, 5500 daylight, 6500 overcast.
``radius``              emitter size in metres. Drives the softness of the
                        shadow it casts and the size of its highlight; 0 is
                        an idealised point.
``cone_angle``          full outer angle of a spot, in degrees.
``cone_softness``       0 is a hard-edged cone, 1 fades from the centre.
``yaw`` / ``pitch``     aim, in degrees. yaw 0 is +X, pitch -90 is straight
                        down. Or pass ``direction=(x, y, z)``.
``range``               metres past which the light is cut off.
======================  ==================================================

Because the units are physical, a lamp only looks right at a matching camera
exposure - see :mod:`pybullet_pbr.photometry` and ``set_camera_exposure``. A scene
preset sets a sensible exposure for you.

There are 8 light slots and a 16-tile shadow atlas. A spot costs one tile, a point
light six (one per cube face), so e.g. two point lights plus four spots fills it.
Lights created with ``shadows=False`` cost no tiles.
"""
import math

from .photometry import kelvin_to_rgb, lumens_to_candela, candela_to_lumens

POINT = 1
SPOT = 2

_attached = {}


def direction_from_angles(yaw, pitch):
    """Unit direction from yaw/pitch in degrees. yaw 0 is +X, pitch -90 is down."""
    y = math.radians(float(yaw))
    p = math.radians(float(pitch))
    horizontal = math.cos(p)
    return (horizontal * math.cos(y), horizontal * math.sin(y), math.sin(p))


def angles_from_direction(direction):
    """Inverse of :func:`direction_from_angles`, as (yaw, pitch) in degrees."""
    x, y, z = direction
    horizontal = math.hypot(x, y)
    return (math.degrees(math.atan2(y, x)), math.degrees(math.atan2(z, horizontal)))


def _native():
    from .config import _native as native
    return native()


def _vec3(v, default=(0.0, 0.0, 0.0)):
    import ctypes
    v = default if v is None else v
    return (ctypes.c_float * 3)(float(v[0]), float(v[1]), float(v[2]))


def add_light(kind=POINT, position=(0, 0, 1), direction=None, yaw=None, pitch=None,
              lumens=1200.0, kelvin=None, color=None, candela=None, radius=0.05,
              range=20.0, cone_angle=40.0, cone_softness=0.4, shadows=True, slot=-1):
    """Create a light and return its slot, or raise if all 8 slots are in use.

    ``kind`` is ``pybullet_pbr.POINT`` (omnidirectional) or ``pybullet_pbr.SPOT``
    (a cone). See the module docstring for what each parameter means. Give the aim as
    either ``yaw``/``pitch`` in degrees or a ``direction`` vector; give the colour as
    either ``kelvin`` or a linear ``color`` triple; give the power as ``lumens`` or,
    if you would rather work in luminous intensity, ``candela``.
    """
    if direction is None:
        if yaw is None and pitch is None:
            direction = (0.0, 0.0, -1.0)
        else:
            direction = direction_from_angles(yaw or 0.0, -90.0 if pitch is None else pitch)

    if color is None:
        color = kelvin_to_rgb(5500 if kelvin is None else kelvin)
    intensity = float(candela) if candela is not None else lumens_to_candela(lumens)

    cone_angle = max(1.0, min(179.0, float(cone_angle)))
    inner_cone = cone_angle * (1.0 - max(0.0, min(1.0, float(cone_softness))))

    got = _native().pbr_set_light(int(slot), int(kind), _vec3(position), _vec3(direction),
                                  _vec3(color, (1, 1, 1)), intensity, float(range),
                                  float(inner_cone), float(cone_angle), float(radius),
                                  int(bool(shadows)))
    if got < 0:
        raise RuntimeError("no free light slot (maximum is %d)" % max_lights())
    return got


def add_spot_light(**kwargs):
    """A cone of light. See :func:`add_light`."""
    kwargs.setdefault("kind", SPOT)
    return add_light(**kwargs)


def add_point_light(**kwargs):
    """An omnidirectional light. See :func:`add_light`."""
    kwargs.setdefault("kind", POINT)
    return add_light(**kwargs)


def move_light(slot, position=None, direction=None):
    """Reposition or re-aim a light. Cheap enough to call every frame."""
    current = get_light(slot)
    _native().pbr_move_light(int(slot),
                             _vec3(current["position"] if position is None else position),
                             _vec3(current["direction"] if direction is None else direction))


def update_light(slot, **kwargs):
    """Change any property of an existing light, leaving the rest alone."""
    current = get_light(slot)
    if current["kind"] == 0:
        raise ValueError("light slot %d is empty" % slot)
    settings = dict(kind=current["kind"], position=current["position"],
                    direction=current["direction"], color=current["color"],
                    candela=current["candela"], range=current["range"],
                    cone_angle=current["cone_angle"], cone_softness=current["cone_softness"],
                    radius=current["radius"], shadows=current["shadows"])
    # a new colour or power supersedes the values read back
    if "kelvin" in kwargs or "color" in kwargs:
        settings.pop("color")
    if "lumens" in kwargs or "candela" in kwargs:
        settings.pop("candela")
    if "yaw" in kwargs or "pitch" in kwargs or "direction" in kwargs:
        settings.pop("direction")
        if "yaw" not in kwargs or "pitch" not in kwargs:
            yaw, pitch = angles_from_direction(current["direction"])
            kwargs.setdefault("yaw", yaw)
            kwargs.setdefault("pitch", pitch)
    settings.update(kwargs)
    return add_light(slot=slot, **settings)


def aim_light(slot, yaw=None, pitch=None):
    """Re-aim a spot light by angle, in degrees."""
    current = get_light(slot)
    current_yaw, current_pitch = angles_from_direction(current["direction"])
    move_light(slot, direction=direction_from_angles(
        current_yaw if yaw is None else yaw, current_pitch if pitch is None else pitch))


def remove_light(slot):
    _attached.pop(slot, None)
    _native().pbr_remove_light(int(slot))


def clear_lights():
    _attached.clear()
    _native().pbr_clear_lights()


def light_count():
    """How many light slots are in use."""
    return sum(1 for slot in range(max_lights()) if get_light(slot)["kind"] != 0)


def max_lights():
    return _native().pbr_get_max_lights()


def get_light(slot):
    """Everything about a light, in the units it was given in."""
    import ctypes
    buf = (ctypes.c_float * 15)()
    shadows = _native().pbr_get_light(int(slot), buf)
    cone_angle = buf[13]
    softness = 1.0 - (buf[12] / cone_angle) if cone_angle > 1e-6 else 0.0
    yaw, pitch = angles_from_direction(tuple(buf[4:7]))
    return {"kind": int(buf[0]), "position": tuple(buf[1:4]), "direction": tuple(buf[4:7]),
            "yaw": yaw, "pitch": pitch, "color": tuple(buf[7:10]),
            "candela": buf[10], "lumens": candela_to_lumens(buf[10]),
            "range": buf[11], "cone_angle": cone_angle,
            "cone_softness": max(0.0, min(1.0, softness)),
            "radius": buf[14], "shadows": bool(shadows)}


def set_light_shadow_bias(metres):
    """Distance bias for point and spot light shadows, in metres.

    Raise it if surfaces shadow themselves. This is separate from the sun's bias, which
    is part of :func:`pybullet_pbr.set_shadows`.
    """
    _native().pbr_set_light_shadow_bias(float(metres))


def get_light_shadow_bias():
    return _native().pbr_get_light_shadow_bias()


def attach_light(slot, body, link=-1, offset=(0, 0, 0), direction=(1, 0, 0),
                 physicsClientId=0):
    """Make a light ride on a body, in that body's local frame.

    ``offset`` is where the light sits relative to the link's frame and ``direction``
    which way a spot points in that frame, so both rotate with the robot. Call
    :func:`sync_lights` once per simulation step to apply it.
    """
    _attached[slot] = dict(body=body, link=link, offset=tuple(offset),
                           direction=tuple(direction), client=physicsClientId)
    sync_lights()


def detach_light(slot):
    _attached.pop(slot, None)


def sync_lights():
    """Move every attached light onto its body. Call once per simulation step."""
    if not _attached:
        return
    import sys
    p = sys.modules.get("pybullet")
    if p is None:
        return

    for slot, a in list(_attached.items()):
        try:
            if a["link"] == -1:
                position, orientation = p.getBasePositionAndOrientation(
                    a["body"], physicsClientId=a["client"])
            else:
                state = p.getLinkState(a["body"], a["link"], physicsClientId=a["client"])
                position, orientation = state[4], state[5]
        except Exception:
            continue  # body gone: leave the light where it was rather than crashing

        world_offset, _ = p.multiplyTransforms(position, orientation, a["offset"],
                                               (0, 0, 0, 1))
        # rotate the aim vector by the link's orientation, without its translation
        aim, _ = p.multiplyTransforms((0, 0, 0), orientation, a["direction"], (0, 0, 0, 1))
        move_light(slot, world_offset, aim)


def look_at(slot, target):
    """Aim a spot light at a world point."""
    light = get_light(slot)
    position = light["position"]
    d = [target[i] - position[i] for i in range(3)]
    length = math.sqrt(sum(c * c for c in d)) or 1.0
    move_light(slot, position, [c / length for c in d])
