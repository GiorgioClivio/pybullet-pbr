# Development notes

Internals and the things that are easy to get wrong. `README.md` documents the public
API; this file is for anyone changing the renderer itself.

## What this is

`pybullet_pbr` replaces PyBullet's fixed-function renderer with a physically based one,
in the GUI viewport and in offscreen cameras, without modifying the user's installed
PyBullet.

## Layout and commands

```
build.sh / clean.sh     fetch Bullet, patch it, build into ./build (never installs, never sudo)
patches/apply.py        idempotent patch of the bullet/ checkout (five small edits, below)
pybullet_pbr/           Python package: activate() loads build/pybullet*.so as `pybullet`
renderer/               PBRInstancingRenderer.cpp, Environment.h, Lights.h, RenderTarget.h,
                        shaders/*.vert|frag (read from disk at runtime)
tests/                  acceptance tests; each asserts a measured image property
run_tests.sh            runs them all, one result line each
showcase.py             interactive tour; --shots renders it all to build/showcase/
example_robotics.py     worked integration example
```

`PYBULLET_PBR_DISABLE=1` falls back to the stock renderer — the fastest A/B check.
`PYBULLET_PBR_SHADER_DIR` overrides the compiled-in shader path, so shader edits need no
rebuild. Build quirk: GCC 15 errors on pybullet.c's `PyArray_DATA(PyObject*)`; build.sh
passes `-Wno-incompatible-pointer-types` via `CFLAGS`. Do not "fix" it in Bullet's source.

## The Bullet patch (keep it this small)

`patches/apply.py` makes exactly five idempotent edits:
1. moves `b3GraphicsInstance` / `InternalDataRenderer` out of `GLInstancingRenderer.cpp`
   into `GLInstancingRendererInternalStructs.h` so a subclass can see them;
2. makes `GLInstancingRenderer`'s data members `protected`;
3. adds a **weak** `b3CreateInstancingRenderer()` hook in `SimpleOpenGL3App.cpp` (returns 0
   → stock renderer; our strong definition wins when linked);
4. the same hook in the EGL plugin's converter, plus a `pbr_register_instance()` callback;
5. adds `renderer/*.cpp` and `-DPBR_SHADER_DIR` to `bullet/setup.py`.

Anything new belongs in `renderer/`, not in Bullet.

## Frame structure (renderSceneInternal)

sun shadow map → light shadow atlas → environment sky into the HDR target → forward scene
pass → SSAO → depth-aware blur → bloom → depth blitted back to the previously bound
framebuffer so Bullet's debug lines and text still depth-test → tone map (applying AO,
white balance, auto-exposure, bloom, vignette, noise, motion blur) → FXAA or TAA resolve.

All of it happens *before* Bullet draws its debug/UI overlay, so overlays are never
filtered. Depth is a texture, not a renderbuffer, so screen-space passes can sample it;
it must stay blit-compatible with the default framebuffer or the overlay breaks.

Only the colour and shadow render modes are ours. Segmentation mask, projective texture
and reflections are delegated to `GLInstancingRenderer::renderSceneInternal`, which is what
keeps `getCameraImage()` working.

## Renderer integration facts

- Vertex format is 9 floats: `position.xyzw`, `normal.xyz`, `uv.xy`. There is no tangent
  attribute; normal mapping derives its frame from screen-space derivatives instead.
- All instance data lives in one VBO (`m_data->m_vbo`): shape vertices occupy the first
  `m_maxShapeCapacityInBytes`, then four tightly packed `totalNumInstances * vec4` arrays —
  position, quaternion, colour, scale. Attribute slots 0..6 with divisors `{0,1,1,0,0,1,1}`.
  Slots 7, 8 and 9 come from our own buffers: previous-frame position, previous-frame
  quaternion, and the per-instance material.
- Instances of a shape are contiguous; the base offset is the running sum of
  `m_numGraphicsInstances` over preceding shapes.
- Graphics instance ids are reused after a reset, so `removeAllInstances()` clears
  everything keyed by one. Without that a new body inherits the previous holder's material.

## Materials: how the per-instance keying works

Only the physics server can map (body, link) to a graphics instance. The way in is that
`changeVisualShape(specularColor=...)` arrives at
`writeSingleInstanceSpecularColorToCPU(specular, srcIndex2)` where **`srcIndex2` is the
graphics instance uid**. We override that method and treat the third component as a marker:
2 carries metallic/roughness, 3 emissive, 4 a (texture slot, handle) pair, 5 a clear. The
base class's own version files specular per *shape*, which is why two bodies sharing a
collision shape could not otherwise have different materials.

Materials reach the shader as instanced vertex attribute 9, reordered from uid order into
internal-instance order via `m_publicGraphicsInstances.getHandle(uid)->m_internalInstanceIndex`.
Do **not** index a table by the uid inside the shader: `instance_scale_obUid.w` does not
survive as the uid in this build — measured, it reads over 1000 for uid 0. That is also why
pybullet's own segmentation mask returns all -1 here, with the stock renderer too.

Textures are per registered shape and bound per draw call, which keeps instancing intact.

## Headless rendering

`pybullet_pbr/headless.py` plus `patch_egl()`. The EGL plugin already builds on
`GLInstancingRenderer`, so it is the same factory swap as the GUI, with our sources compiled
into the eglRenderer extension too. Four things that are easy to get wrong:

- **each extension has its own copy of the renderer's globals.** `config.py` dispatches
  every setter to all attached libraries and journals them, so a library attached later is
  replayed up to date; queries go to the first library only. `_MUTATORS` is an explicit
  list, not a name-prefix test — `pbr_register_texture` is a mutator despite not being
  called "set".
- **weak hook declarations must be `extern "C"`.** A plain `__attribute__((weak))` C++
  declaration is name-mangled while our definitions are `extern "C"`, so the pointer stays
  null and the hook is skipped with no error.
- **the server never forwards specular colours to a plugin renderer**, so the
  changeVisualShape transport does not reach the headless path. The EGL converter reports
  (body, link) → graphics instance instead, and materials are addressed by body via
  `pbr_set_material_by_body`. Requests are recorded so they can be set before the instance
  exists — the converter creates instances lazily at the first render.
- the sun direction likewise travels through `configureDebugVisualizer(lightPosition=)` in
  the GUI, which does nothing headless. `pbr_set_sun_direction` is the real path.

An infinite `GEOM_PLANE` crashes the EGL converter; use bounded geometry there.

## Physical units

The shading maths is in physical units, so nothing in the shaders needed converting: sun
intensity *is* illuminance in lux, the result *is* luminance in cd/m², and exposure is
`1/(1.2 * 2^EV100)`. Check it holds: 100 000 lux on an 18% grey surface is 5730 cd/m², and
at sunny-16 (EV 14.97) that lands at 0.149, mid grey.

- lights are given in **lumens**, converted with `lumens/(4*pi)` for both point *and* spot,
  so narrowing a beam does not silently brighten it;
- **cone angles are full angles**; shading compares against the angle from the beam axis,
  which is half of that. Using the full angle there makes every cone twice as wide as
  documented, and `tests/test_lights.py` checks the pool radius against `h*tan(a/2)`;
- sky colours in `scenes.py` are chromaticities scaled by `sky_luminance` in cd/m²;
- a scene's `ev` is metered for its *natural* light, and for the light reaching a
  horizontal surface — beam illuminance times sin(elevation). A robot's own lamp needs
  `photometry.ev_for_light()` instead;
- a material's `base_color` is **sRGB**, so a surface reflecting 9% of the light is 0.33,
  not 0.09. A scene's `ground_bounce` is `albedo * horizontal illuminance / pi` and must be
  recomputed when an albedo changes.

## White balance

Physical colour temperature means a 5500 K sun is genuinely warmer than sRGB's D65 white
point. `tonemap.frag` divides by `whiteBalance`, the colour of the illuminant the camera is
balanced for. Balancing to *each scene's own sun* over-corrects: a sunset divided by its own
2000 K key light leaves the blue sky fill behind and the frame goes blue. Cameras use a
fixed daylight preset, and so do the scenes, via `DAYLIGHT_BALANCE_KELVIN`. Airless scenes
are the exception and balance to their own sun, having no sky fill to preserve.

Two more colour traps: tinting the sky's *lower* hemisphere (the ground-bounce term) warm
puts a warm cast on the whole scene, because bounce reaches what direct sun does not; and a
lamp reads neutral only when its colour temperature matches the camera's white balance.

## Shadows

The sun's depth map is rendered here, not by the base class, with normal back-face culling.
Bullet's own pass culls *front* faces, so a solid object's occluder depth is its far side
and the ground under a sphere tests as lit. The light matrices —
`ortho(-worldSize, worldSize, ..., 1, 300)` and `lookAt(direction*60 + target, target, ...)`
— live in `pbrCreateOrtho`/`pbrCreateLookAt`.

That ortho spans 299 world units, so a bias expressed in raw depth units is ~300x too strong
and silently erases every shadow. Bias is configured in world units and divided by
`shadowFar - shadowNear` before it reaches the shader. `sunPenumbraScale` widens the filter
with distance from the blocker, which is what gives the sun its angular size.

Point and spot lights share one 16-tile **atlas** rather than a texture array, which keeps
every shader at GLSL 330. A spot takes one tile, a point light six. The atlas stores
**distance to the light normalised by range**, not post-projection depth: a point light then
needs no matrices in the fragment shader (cube-face selection is arithmetic on the light
vector, and the six face view matrices must stay in step with `cubeFace()` in forward.frag),
and the bias is in metres for both light types.

## SSAO, bloom, exposure

- SSAO has no normal buffer: `ssao.frag` reconstructs view-space position from depth and the
  normal from its screen-space derivatives. The reconstruction assumes a *symmetric*
  perspective projection, and the sign matters: `viewZ = -P32 / (ndcZ + P22)`, negative in
  front of the eye. Getting it positive produces an almost-white AO buffer.
- The blur's depth rejection compares *linear view* depth. Raw depth-buffer differences are
  meaningless at distance and let silhouettes smear onto the background.
- AO is applied at composite time: `forward.frag` writes the pixel's ambient *share* into
  the HDR alpha and `tonemap.frag` occludes only that share. So HDR alpha is not a
  transparency channel.
- Auto-exposure meters from a small mip over a 3x3 spread of taps. Metering off the single
  top mip level of a non-square texture is not a reliable mean.
- Bloom has its own quarter-res bright-pass and separable Gaussian. The HDR mip chain cannot
  stand in for the blur: a mip averages the whole frame, so one bright source spreads
  everywhere and the halo comes out rectangular.
- `RenderTarget::generateColorMips()` deliberately leaves `GL_TEXTURE_MIN_FILTER` alone.
  Setting a mipmap filter makes every later plain `texture()` read of that attachment depend
  on the chain still being current, and after a resize it is not — those reads return black.
- Motion blur shares TAA's velocity buffer, so previous-frame transforms must be maintained
  whenever *either* is on, or stationary objects smear.

## Environment lighting

`renderer/Environment.h` builds the cubemap, irradiance convolution, prefiltered specular
mip chain and BRDF LUT once, whenever Python marks the environment dirty; the render thread
consumes that flag because only it has a GL context. Every cube face is a fullscreen
triangle with a `face` uniform — no cube mesh, no per-face view matrices. The source is
either an equirectangular HDR (stb_image's `stbi_loadf`) or a procedural sky, both through
`env_capture.frag`.

- **sampler units must be assigned even when IBL is off.** Sampler uniforms default to unit
  0, and a program with a `sampler2D` and a `samplerCube` on the same unit is invalid to
  draw with, so every draw call fails and the scene renders empty.
- the camera basis for the background ray comes from the **view matrix** (its rotation
  transposed), not from `getCameraUpVector()`, which does not reliably return the camera's
  own up — with it the sky has no vertical gradient at all.
- the sky has no sun *disc*: the directional light already supplies the sun highlight, and a
  `pow(cos, 800)` disc is far below cubemap resolution and shows up as a blocky smear.
- the background pass fades the environment's lower hemisphere to black when `airless` is
  set. That hemisphere carries ground bounce, which the scene should *receive* but which
  would otherwise draw as a bright band past the horizon.

## Testing

Every test asserts a measured property of the image, never a golden file, so they survive
driver differences. Two gotchas:

- GUI frames and `getCameraImage()` render at different sizes, and alternating them
  reallocates the render targets each frame. Anything sensitive to accumulated state (TAA,
  exposure metering) is tested headless, where the viewport is constant.
- A spot aimed at a shallow angle projects a long ellipse that legitimately reaches the far
  corners of a frame. Point it straight down to measure cone containment.
