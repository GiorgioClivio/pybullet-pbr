"""Per-object material assignment.

Materials travel to the renderer through PyBullet's existing
``changeVisualShape(specularColor=...)`` channel, tagged with a marker value so the
renderer can tell a PBR payload from an ordinary specular colour, and in parallel
through a direct (body, link) call that the headless renderer uses. No PyBullet API is
extended.

Two consequences worth knowing: setting ``specularColor`` yourself overrides a material,
and textures apply per registered graphics *shape*, so two links built from one shared
shape share their maps.
"""
_MARKER = 2.0
_EMISSIVE_MARKER = 3.0
_TEXTURE_MARKER = 4.0
_CLEAR_MARKER = 5.0

#: Extra material maps, in the order the renderer's slots are numbered.
TEXTURE_SLOTS = {"normal": 0, "roughness": 1, "metallic": 2, "ao": 3}

#: Named materials: (metallic, roughness, base colour RGB).
#:
#: Base colours are sRGB, so they are the *encoded* form of the surface's reflectance,
#: not the reflectance itself: lunar mare regolith reflects about 9% of the light that
#: hits it, which is 0.33 in sRGB, not 0.09. Getting that wrong makes every natural
#: surface come out several times too dark.
#:
#: Reflectances used: lunar mare 0.09, lunar highland 0.16, Aristarchus-class fresh
#: ejecta 0.26, lunar pyroclastic mantle 0.07, Martian soil 0.25 in red falling to 0.06
#: in blue, concrete 0.26, fresh snow 0.85.
MATERIALS = {
    "aluminium":      (1.0, 0.25, (0.77, 0.78, 0.80)),
    "steel":          (1.0, 0.35, (0.66, 0.67, 0.69)),
    "metal_polished": (1.0, 0.08, (0.72, 0.73, 0.75)),
    "metal_brushed":  (1.0, 0.40, (0.62, 0.63, 0.65)),
    "gold_foil":      (1.0, 0.22, (1.00, 0.77, 0.34)),
    "copper":         (1.0, 0.30, (0.95, 0.64, 0.54)),
    "plastic_white":  (0.0, 0.40, (0.90, 0.90, 0.88)),
    "plastic_black":  (0.0, 0.45, (0.06, 0.06, 0.07)),
    "rubber":         (0.0, 0.92, (0.05, 0.05, 0.06)),
    "carbon_fiber":   (0.0, 0.30, (0.05, 0.05, 0.06)),
    "solar_panel":    (0.6, 0.14, (0.04, 0.05, 0.14)),
    "lunar_regolith": (0.0, 0.96, (0.33, 0.325, 0.318)),   # mare, 9% reflectance
    "lunar_highland": (0.0, 0.95, (0.44, 0.435, 0.425)),   # 16%
    "lunar_bright":   (0.0, 0.93, (0.55, 0.545, 0.535)),   # fresh crater ejecta, 26%
    "lunar_pyroclastic": (0.0, 0.97, (0.30, 0.275, 0.255)),  # dark mantling, slightly red
    "mars_regolith":  (0.0, 0.93, (0.54, 0.40, 0.30)),
    "rock_basalt":    (0.0, 0.86, (0.16, 0.15, 0.15)),
    "rock_light":     (0.0, 0.82, (0.48, 0.45, 0.41)),
    "sand":           (0.0, 0.88, (0.62, 0.53, 0.38)),
    "concrete":       (0.0, 0.85, (0.55, 0.54, 0.52)),
    "ice":            (0.0, 0.14, (0.82, 0.90, 0.94)),
    "water":          (0.0, 0.04, (0.02, 0.05, 0.07)),
    "snow":           (0.0, 0.55, (0.90, 0.92, 0.95)),
}


def material_names():
    """Every named material, as name -> (metallic, roughness, base colour)."""
    return dict(MATERIALS)


def _pybullet():
    import sys
    mod = sys.modules.get("pybullet")
    if mod is None:
        raise RuntimeError("import pybullet first (after pybullet_pbr.activate())")
    return mod


def set_material(body, link=-1, preset=None, metallic=None, roughness=None,
                 base_color=None, emissive=None, base_color_texture=None,
                 normal_texture=None, roughness_texture=None, metallic_texture=None,
                 ao_texture=None, physicsClientId=0):
    """Assign a PBR material to a body link.

    ``preset`` names one of :data:`MATERIALS` ("lunar_regolith", "aluminium", "ice", ...)
    and any of ``metallic`` / ``roughness`` / ``base_color`` given alongside it override
    that preset's value. Without a preset, whichever of metallic/roughness you omit falls
    back to the global default - it is not preserved from a previous call.

    ``emissive`` makes the surface radiate light of its own, in cd/m² - an LED panel is
    around 1e4, a lit screen 300, a glowing marker whatever reads well. It is emitted
    light, not received, so it stays visible with every lamp switched off. (It lights
    only itself: it does not illuminate its surroundings.)

    The texture arguments take image paths - any image, the ones shipped in ``assets``
    are only defaults. ``base_color_texture=False`` removes an existing one, which is
    what you want when applying a ground material over PyBullet's checkerboard plane;
    note that removal goes through PyBullet's own texture channel and the headless EGL
    converter ignores it, so headless, assign a plain white image instead of removing. ``base_color_texture`` goes through
    PyBullet's own texture channel; the others are extra maps this renderer adds.
    Roughness, metallic and AO maps *multiply* the material's values, so a map darkens
    or roughens what the material already says it is. Normal maps use a tangent frame
    derived per pixel, so meshes need no tangent attribute and untextured or
    degenerate UVs degrade to the plain geometric normal.

    Unlike the other settings, textures apply per registered graphics *shape*, matching
    how PyBullet treats its own base-colour texture: two bodies built from one shared
    collision shape will share their maps.

    ``link=None`` applies to the base and every link of the body, which is usually what
    you want for a whole robot.
    """
    p = _pybullet()

    if preset is not None:
        try:
            preset_metallic, preset_roughness, preset_color = MATERIALS[preset]
        except KeyError:
            raise ValueError("unknown material %r, expected one of %s"
                             % (preset, sorted(MATERIALS)))
        metallic = preset_metallic if metallic is None else metallic
        roughness = preset_roughness if roughness is None else roughness
        if base_color is None:
            base_color = list(preset_color) + [1.0]

    if link is None:
        links = [-1] + list(range(p.getNumJoints(body, physicsClientId=physicsClientId)))
        for index in links:
            set_material(body, index, metallic=metallic, roughness=roughness,
                         base_color=base_color, emissive=emissive,
                         base_color_texture=base_color_texture,
                         normal_texture=normal_texture,
                         roughness_texture=roughness_texture,
                         metallic_texture=metallic_texture, ao_texture=ao_texture,
                         physicsClientId=physicsClientId)
        return
    if base_color is not None:
        p.changeVisualShape(body, link, rgbaColor=list(base_color),
                            physicsClientId=physicsClientId)
    if metallic is not None or roughness is not None:
        from .config import get_default_material
        default_metallic, default_roughness = get_default_material()
        m = default_metallic if metallic is None else float(metallic)
        r = default_roughness if roughness is None else float(roughness)
        _send(p, body, link, m, r, _MARKER, physicsClientId)
    if emissive is not None:
        _send(p, body, link, float(emissive), 0.0, _EMISSIVE_MARKER, physicsClientId)

    if base_color_texture is not None:
        # False removes whatever texture is already there. Base colour multiplies the
        # texture, so a preset applied over PyBullet's checkerboard plane comes out at
        # half the albedo the preset names - which reads as "the scene is too dark".
        texture = -1 if base_color_texture is False \
            else p.loadTexture(str(base_color_texture), physicsClientId=physicsClientId)
        p.changeVisualShape(body, link, textureUniqueId=texture,
                            physicsClientId=physicsClientId)
    for name, path in (("normal", normal_texture), ("roughness", roughness_texture),
                       ("metallic", metallic_texture), ("ao", ao_texture)):
        if path is None:
            continue
        handle = -1 if path is False else load_texture(path)
        _send(p, body, link, float(TEXTURE_SLOTS[name]), float(handle), _TEXTURE_MARKER,
              physicsClientId)


def load_texture(path, srgb=False):
    """Register an image with the renderer and return its handle.

    The file is read on the render thread the first time it is used, so this can be
    called before anything is rendered. ``srgb`` should be true only for colour maps;
    normal, roughness, metallic and AO maps carry measurements and must stay linear.
    """
    import os

    path = os.path.abspath(os.path.expanduser(str(path)))
    if not os.path.exists(path):
        raise FileNotFoundError(path)
    from .config import _native
    handle = _native().pbr_register_texture(path.encode("utf-8"), int(bool(srgb)))
    if handle < 0:
        raise RuntimeError("could not register texture %r" % path)
    return handle


def _send(p, body, link, a, b, marker, physicsClientId):
    """Deliver one material payload by both routes.

    In the GUI the physics server resolves changeVisualShape(specularColor=...) to the
    right graphics instance for us. A plugin renderer never receives specular colours,
    so the headless path is addressed by (body, link) instead, using the instance map
    the EGL converter reports. Each route is a no-op for the renderer that does not use
    it, so sending both keeps one API working everywhere.
    """
    p.changeVisualShape(body, link, specularColor=[a, b, marker],
                        physicsClientId=physicsClientId)
    from .config import _native
    _native().pbr_set_material_by_body(int(body), int(link), a, b, marker)


def clear_material(body, link=-1, physicsClientId=0):
    """Drop back to the configured default material for this link."""
    _send(_pybullet(), body, link, 0.0, 0.0, _CLEAR_MARKER, physicsClientId)
