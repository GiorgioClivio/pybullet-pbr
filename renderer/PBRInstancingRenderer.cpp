#include "PBRInstancingRenderer.h"
#include "RenderTarget.h"
#include "Environment.h"
#include "Lights.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <string.h>

#include "../bullet/examples/OpenGLWindow/GLInstancingRendererInternalStructs.h"
#include "../bullet/examples/OpenGLWindow/LoadShader.h"
#include "Bullet3Common/b3Logging.h"
#include "../bullet/examples/ThirdPartyLibs/stb_image/stb_image.h"

#ifndef PBR_SHADER_DIR
#define PBR_SHADER_DIR "shaders"
#endif

// Runtime configuration, written from Python via ctypes (see pybullet_pbr/config.py)
// and read by the render thread each frame.
// Written from Python via ctypes and read by the render thread without a lock: every
// field is a plain scalar, so a torn read costs at most one frame of a stale value.
struct PbrConfig
{
	float m_exposure;
	int m_tonemapper;  // 0 none, 1 reinhard, 2 aces
	int m_diagnostic;  // 1 = world normals, 2 = ambient-occlusion buffer
	float m_sunIntensity;
	float m_ambientIntensity;
	float m_defaultMetallic;
	float m_defaultRoughness;
	float m_shadowBias;       // constant depth bias, in world units
	float m_shadowSlopeBias;  // extra bias proportional to the surface slope, world units
	float m_shadowSoftness;   // PCF kernel radius, in shadow-map texels
	float m_shadowStrength;   // 0 = no shadowing, 1 = shadows remove all direct sun
	int m_ssaoEnabled;
	float m_ssaoRadius;       // world units
	float m_ssaoStrength;
	float m_ssaoBias;         // view-space units, keeps flat surfaces from occluding themselves
	int m_antialiasing;       // 0 off, 1 fxaa, 2 taa
	float m_taaBlend;         // weight given to the reprojected history
	// --- camera sensor ---
	int m_autoExposure;
	float m_exposureKey;      // target average luminance
	float m_exposureMinEv;
	float m_exposureMaxEv;
	float m_bloom;
	float m_bloomThreshold;   // cd/m2
	float m_vignette;
	float m_noise;
	float m_motionBlur;
	float m_whiteBalance[3];
	int m_iblEnabled;
	float m_iblStrength;
	int m_showBackground;
	int m_environmentDirty;   // set from Python, consumed by the render thread
	char m_environmentPath[512];
	float m_sunColor[3];
	float m_skyZenith[3];
	float m_skyHorizon[3];
	float m_skyGround[3];
	float m_skyGlow;
	int m_skyAirless;         // 1 = no atmosphere: black sky, only ground bounce
	PbrLight m_lights[PBR_MAX_LIGHTS];
	int m_lightCount;
	float m_lightShadowBias;  // metres
	float m_normalScale;         // strength of normal maps
	float m_sunAngularDiameter;  // degrees; 0.53 from Earth and the Moon, 0.35 from Mars
	float m_sunDirection[3];     // towards the sun; overrides Bullet's light position
	int m_sunDirectionSet;

	PbrConfig() : m_exposure(0.8f), m_tonemapper(2), m_diagnostic(0),
				  m_sunIntensity(3.f), m_ambientIntensity(0.15f),
				  m_defaultMetallic(0.f), m_defaultRoughness(0.6f),
				  m_shadowBias(0.005f), m_shadowSlopeBias(0.03f), m_shadowSoftness(1.5f),
				  m_shadowStrength(1.f), m_ssaoEnabled(1), m_ssaoRadius(0.3f),
				  m_ssaoStrength(1.f), m_ssaoBias(0.02f), m_antialiasing(1), m_taaBlend(0.9f), m_autoExposure(0),
				  m_exposureKey(0.18f), m_exposureMinEv(-6.f), m_exposureMaxEv(20.f),
				  m_bloom(0.f), m_bloomThreshold(1.f), m_vignette(0.f), m_noise(0.f),
				  m_motionBlur(0.f),
				  m_iblEnabled(1), m_iblStrength(1.f), m_showBackground(1),
				  m_environmentDirty(1)
	{
		m_environmentPath[0] = 0;
		m_sunColor[0] = m_sunColor[1] = m_sunColor[2] = 1.f;
		m_skyZenith[0] = 0.22f; m_skyZenith[1] = 0.34f; m_skyZenith[2] = 0.62f;
		m_skyHorizon[0] = 0.62f; m_skyHorizon[1] = 0.68f; m_skyHorizon[2] = 0.78f;
		m_skyGround[0] = 0.14f; m_skyGround[1] = 0.13f; m_skyGround[2] = 0.12f;
		m_skyGlow = 0.012f;
		m_skyAirless = 0;
		m_lightCount = 0;
		m_lightShadowBias = 0.04f;
		// 5500 K, a camera's daylight preset, so the default look is balanced for the sun
		m_whiteBalance[0] = 1.146f;
		m_whiteBalance[1] = 0.970f;
		m_whiteBalance[2] = 0.864f;
		m_normalScale = 1.f;
		m_sunAngularDiameter = 0.53f;
		m_sunDirection[0] = 0.f; m_sunDirection[1] = 0.f; m_sunDirection[2] = 1.f;
		m_sunDirectionSet = 0;
	}
};
static PbrConfig gConfig;


// Python (ctypes) looks these up by name, so they must survive -fvisibility=hidden.
#define PBR_API extern "C" __attribute__((visibility("default")))

/// Which graphics instances belong to which body and link.
///
/// The GUI learns this implicitly, because changeVisualShape(specularColor=...) is
/// routed to the right instance by the physics server. A plugin renderer never sees
/// specular colours at all, so the headless path reports the mapping instead and
/// materials are addressed by (body, link) directly.
struct PbrInstanceKey
{
	int m_body;
	int m_link;
	int m_uid;
};
static b3AlignedObjectArray<PbrInstanceKey> gInstanceKeys;

/// Renderer the by-body material API talks to. One per loaded extension, with the same
/// lifetime as the renderer itself.
static PBRInstancingRenderer* gRenderer = 0;

static void pbr_apply_pending_materials(int graphicsUid, int bodyUniqueId, int linkIndex);

PBR_API void pbr_register_instance(int bodyUniqueId, int linkIndex, int graphicsUid)
{
	for (int i = 0; i < gInstanceKeys.size(); i++)
		if (gInstanceKeys[i].m_uid == graphicsUid)
		{
			gInstanceKeys[i].m_body = bodyUniqueId;
			gInstanceKeys[i].m_link = linkIndex;
			return;
		}
	PbrInstanceKey key;
	key.m_body = bodyUniqueId;
	key.m_link = linkIndex;
	key.m_uid = graphicsUid;
	gInstanceKeys.push_back(key);
	pbr_apply_pending_materials(graphicsUid, bodyUniqueId, linkIndex);
}

/// Materials addressed by (body, link) before their instance exists.
///
/// The headless converter creates graphics instances lazily, at the first render, so a
/// set_material() right after loadURDF() would otherwise be dropped. Recording the
/// request means ordering does not matter.
struct PbrPendingMaterial
{
	int m_body;
	int m_link;
	float m_a;
	float m_b;
	float m_marker;
};
static b3AlignedObjectArray<PbrPendingMaterial> gPendingMaterials;

PBR_API int pbr_instance_map_size() { return gInstanceKeys.size(); }

/// Extra material maps, per registered graphics shape.
///
/// Requests arrive from Python as a (slot, handle) pair and are stored per shape,
/// matching how PyBullet already treats its base-colour texture: one per registered
/// shape, bound per draw call. That keeps instancing intact - a texture array keyed
/// per instance would force every map to a common resolution.
#define PBR_TEX_NORMAL 0
#define PBR_TEX_ROUGHNESS 1
#define PBR_TEX_METALLIC 2
#define PBR_TEX_AO 3
#define PBR_TEX_SLOTS 4

struct PbrTextureRequest
{
	char m_path[512];
	int m_srgb;
	unsigned int m_glTexture;   // 0 until the render thread loads it
	int m_failed;
};
static b3AlignedObjectArray<PbrTextureRequest> gTextureRequests;

struct PbrShapeTextures
{
	int m_shape;
	int m_handle[PBR_TEX_SLOTS];
};
static b3AlignedObjectArray<PbrShapeTextures> gShapeTextures;

/// Records a texture to load. Returns a handle; the file is read on the render thread,
/// which is the only one with a GL context.
PBR_API int pbr_register_texture(const char* path, int srgb)
{
	if (!path || !path[0])
		return -1;
	for (int i = 0; i < gTextureRequests.size(); i++)
		if (strcmp(gTextureRequests[i].m_path, path) == 0 &&
			gTextureRequests[i].m_srgb == srgb)
			return i;

	PbrTextureRequest request;
	size_t n = strlen(path);
	if (n >= sizeof(request.m_path))
		n = sizeof(request.m_path) - 1;
	memcpy(request.m_path, path, n);
	request.m_path[n] = 0;
	request.m_srgb = srgb;
	request.m_glTexture = 0;
	request.m_failed = 0;
	gTextureRequests.push_back(request);
	return gTextureRequests.size() - 1;
}

PBR_API int pbr_texture_count() { return gTextureRequests.size(); }

static PbrShapeTextures* pbrShapeTextures(int shapeIndex, bool create)
{
	for (int i = 0; i < gShapeTextures.size(); i++)
		if (gShapeTextures[i].m_shape == shapeIndex)
			return &gShapeTextures[i];
	if (!create)
		return 0;
	PbrShapeTextures entry;
	entry.m_shape = shapeIndex;
	for (int i = 0; i < PBR_TEX_SLOTS; i++)
		entry.m_handle[i] = -1;
	gShapeTextures.push_back(entry);
	return &gShapeTextures[gShapeTextures.size() - 1];
}

PBR_API void pbr_set_exposure(float v) { gConfig.m_exposure = v; }
PBR_API float pbr_get_exposure() { return gConfig.m_exposure; }
PBR_API void pbr_set_tonemapper(int v) { gConfig.m_tonemapper = v; }
PBR_API int pbr_get_tonemapper() { return gConfig.m_tonemapper; }
PBR_API void pbr_set_diagnostic(int v) { gConfig.m_diagnostic = v; }
PBR_API int pbr_get_diagnostic() { return gConfig.m_diagnostic; }
PBR_API void pbr_set_sun_intensity(float v) { gConfig.m_sunIntensity = v; }
PBR_API float pbr_get_sun_intensity() { return gConfig.m_sunIntensity; }
PBR_API void pbr_set_ambient_intensity(float v) { gConfig.m_ambientIntensity = v; }
PBR_API float pbr_get_ambient_intensity() { return gConfig.m_ambientIntensity; }
PBR_API void pbr_set_default_material(float metallic, float roughness)
{
	gConfig.m_defaultMetallic = metallic;
	gConfig.m_defaultRoughness = roughness;
}
PBR_API float pbr_get_default_metallic() { return gConfig.m_defaultMetallic; }
PBR_API float pbr_get_default_roughness() { return gConfig.m_defaultRoughness; }
PBR_API void pbr_set_shadow_params(float bias, float slopeBias, float softness, float strength)
{
	gConfig.m_shadowBias = bias;
	gConfig.m_shadowSlopeBias = slopeBias;
	gConfig.m_shadowSoftness = softness;
	gConfig.m_shadowStrength = strength;
}
PBR_API float pbr_get_shadow_strength() { return gConfig.m_shadowStrength; }
PBR_API void pbr_set_ssao(int enabled, float radius, float strength, float bias)
{
	gConfig.m_ssaoEnabled = enabled;
	gConfig.m_ssaoRadius = radius;
	gConfig.m_ssaoStrength = strength;
	gConfig.m_ssaoBias = bias;
}
PBR_API int pbr_get_ssao_enabled() { return gConfig.m_ssaoEnabled; }
PBR_API float pbr_get_ssao_radius() { return gConfig.m_ssaoRadius; }
PBR_API float pbr_get_ssao_strength() { return gConfig.m_ssaoStrength; }
PBR_API float pbr_get_ssao_bias() { return gConfig.m_ssaoBias; }
PBR_API void pbr_set_environment(const char* path, float strength, int showBackground)
{
	// Only the render thread has a GL context, so this just records the request.
	gConfig.m_environmentPath[0] = 0;
	if (path && path[0])
	{
		size_t n = strlen(path);
		if (n >= sizeof(gConfig.m_environmentPath))
			n = sizeof(gConfig.m_environmentPath) - 1;
		memcpy(gConfig.m_environmentPath, path, n);
		gConfig.m_environmentPath[n] = 0;
	}
	gConfig.m_iblEnabled = strength > 0.f ? 1 : 0;
	gConfig.m_iblStrength = strength;
	gConfig.m_showBackground = showBackground;
	gConfig.m_environmentDirty = 1;
}
PBR_API int pbr_get_ibl_enabled() { return gConfig.m_iblEnabled; }
PBR_API float pbr_get_ibl_strength() { return gConfig.m_iblStrength; }
PBR_API int pbr_get_show_background() { return gConfig.m_showBackground; }
PBR_API void pbr_set_sun_color(float r, float g, float b)
{
	gConfig.m_sunColor[0] = r;
	gConfig.m_sunColor[1] = g;
	gConfig.m_sunColor[2] = b;
	gConfig.m_environmentDirty = 1;  // the sky's sun glow uses it too
}
PBR_API void pbr_get_sun_color(float* out)
{
	out[0] = gConfig.m_sunColor[0];
	out[1] = gConfig.m_sunColor[1];
	out[2] = gConfig.m_sunColor[2];
}
/// Sky appearance. Any change rebuilds the IBL textures, so the light the scene
/// receives always matches the sky that is drawn.
PBR_API void pbr_set_sky(const float* zenith, const float* horizon, const float* ground,
						 float glow, int airless)
{
	for (int i = 0; i < 3; i++)
	{
		gConfig.m_skyZenith[i] = zenith[i];
		gConfig.m_skyHorizon[i] = horizon[i];
		gConfig.m_skyGround[i] = ground[i];
	}
	gConfig.m_skyGlow = glow;
	gConfig.m_skyAirless = airless;
	gConfig.m_environmentDirty = 1;
}
PBR_API void pbr_get_sky(float* out)
{
	for (int i = 0; i < 3; i++)
	{
		out[i] = gConfig.m_skyZenith[i];
		out[3 + i] = gConfig.m_skyHorizon[i];
		out[6 + i] = gConfig.m_skyGround[i];
	}
	out[9] = gConfig.m_skyGlow;
	out[10] = (float)gConfig.m_skyAirless;
}
PBR_API const char* pbr_get_environment_path() { return gConfig.m_environmentPath; }
/// Adds or replaces a light. Slot >= 0 replaces that slot, -1 takes the next free
/// one. Returns the slot, or -1 when they are all taken.
PBR_API int pbr_set_light(int slot, int type, const float* position, const float* direction,
						  const float* color, float intensity, float range,
						  float innerDegrees, float outerDegrees, float radius, int castsShadow)
{
	if (slot < 0)
	{
		for (int i = 0; i < PBR_MAX_LIGHTS; i++)
			if (gConfig.m_lights[i].m_type == PBR_LIGHT_OFF)
			{
				slot = i;
				break;
			}
		if (slot < 0)
			return -1;
	}
	if (slot >= PBR_MAX_LIGHTS)
		return -1;

	PbrLight& light = gConfig.m_lights[slot];
	for (int i = 0; i < 3; i++)
	{
		light.m_position[i] = position[i];
		light.m_direction[i] = direction[i];
		light.m_color[i] = color[i];
	}
	light.m_intensity = intensity;
	light.m_range = range > 0.001f ? range : 0.001f;
	light.m_innerDegrees = innerDegrees;
	light.m_outerDegrees = outerDegrees;
	light.m_radius = radius < 0.f ? 0.f : radius;
	light.m_castsShadow = castsShadow;
	light.m_type = type;   // published last: the render thread reads type first

	if (slot >= gConfig.m_lightCount)
		gConfig.m_lightCount = slot + 1;
	return slot;
}

/// Moving a light is the common case, so it gets a cheap path of its own.
PBR_API void pbr_move_light(int slot, const float* position, const float* direction)
{
	if (slot < 0 || slot >= PBR_MAX_LIGHTS)
		return;
	for (int i = 0; i < 3; i++)
	{
		gConfig.m_lights[slot].m_position[i] = position[i];
		gConfig.m_lights[slot].m_direction[i] = direction[i];
	}
}

PBR_API void pbr_remove_light(int slot)
{
	if (slot < 0 || slot >= PBR_MAX_LIGHTS)
		return;
	gConfig.m_lights[slot].m_type = PBR_LIGHT_OFF;
	while (gConfig.m_lightCount > 0 &&
		   gConfig.m_lights[gConfig.m_lightCount - 1].m_type == PBR_LIGHT_OFF)
		gConfig.m_lightCount--;
}

PBR_API void pbr_clear_lights()
{
	for (int i = 0; i < PBR_MAX_LIGHTS; i++)
		gConfig.m_lights[i].m_type = PBR_LIGHT_OFF;
	gConfig.m_lightCount = 0;
}

PBR_API int pbr_get_light_count() { return gConfig.m_lightCount; }
PBR_API int pbr_get_max_lights() { return PBR_MAX_LIGHTS; }

/// Fills out[] with type, position, direction, colour, intensity, range, inner, outer,
/// radius - 15 floats. Returns the shadow flag.
PBR_API int pbr_get_light(int slot, float* out)
{
	if (slot < 0 || slot >= PBR_MAX_LIGHTS)
		return 0;
	const PbrLight& l = gConfig.m_lights[slot];
	out[0] = (float)l.m_type;
	for (int i = 0; i < 3; i++)
	{
		out[1 + i] = l.m_position[i];
		out[4 + i] = l.m_direction[i];
		out[7 + i] = l.m_color[i];
	}
	out[10] = l.m_intensity;
	out[11] = l.m_range;
	out[12] = l.m_innerDegrees;
	out[13] = l.m_outerDegrees;
	out[14] = l.m_radius;
	return l.m_castsShadow;
}

/// Sun direction, set directly rather than through configureDebugVisualizer's light
/// position - that path only exists in the GUI, so scenes would silently keep the
/// default sun when rendering headless.
PBR_API void pbr_set_normal_scale(float v) { gConfig.m_normalScale = v; }
PBR_API float pbr_get_normal_scale() { return gConfig.m_normalScale; }
PBR_API void pbr_set_sun_direction(float x, float y, float z)
{
	float length = sqrtf(x * x + y * y + z * z);
	if (length < 1e-6f)
		return;
	gConfig.m_sunDirection[0] = x / length;
	gConfig.m_sunDirection[1] = y / length;
	gConfig.m_sunDirection[2] = z / length;
	gConfig.m_sunDirectionSet = 1;
	gConfig.m_environmentDirty = 1;   // the sky's glow follows the sun
}
PBR_API void pbr_get_sun_direction(float* out)
{
	out[0] = gConfig.m_sunDirection[0];
	out[1] = gConfig.m_sunDirection[1];
	out[2] = gConfig.m_sunDirection[2];
}
PBR_API void pbr_set_sun_angular_diameter(float degrees)
{
	gConfig.m_sunAngularDiameter = degrees;
}
PBR_API float pbr_get_sun_angular_diameter() { return gConfig.m_sunAngularDiameter; }
PBR_API void pbr_set_light_shadow_bias(float metres) { gConfig.m_lightShadowBias = metres; }
PBR_API float pbr_get_light_shadow_bias() { return gConfig.m_lightShadowBias; }

PBR_API void pbr_set_antialiasing(int mode) { gConfig.m_antialiasing = mode; }
PBR_API int pbr_get_antialiasing() { return gConfig.m_antialiasing; }
PBR_API void pbr_set_auto_exposure(int enabled, float key, float minEv, float maxEv)
{
	gConfig.m_autoExposure = enabled;
	gConfig.m_exposureKey = key;
	gConfig.m_exposureMinEv = minEv;
	gConfig.m_exposureMaxEv = maxEv;
}
PBR_API int pbr_get_auto_exposure() { return gConfig.m_autoExposure; }
PBR_API float pbr_get_exposure_key() { return gConfig.m_exposureKey; }
PBR_API void pbr_set_sensor(float bloom, float bloomThreshold, float vignette,
							float noise, float motionBlur)
{
	gConfig.m_bloom = bloom;
	gConfig.m_bloomThreshold = bloomThreshold;
	gConfig.m_vignette = vignette;
	gConfig.m_noise = noise;
	gConfig.m_motionBlur = motionBlur;
}
PBR_API void pbr_get_sensor(float* out)
{
	out[0] = gConfig.m_bloom;
	out[1] = gConfig.m_bloomThreshold;
	out[2] = gConfig.m_vignette;
	out[3] = gConfig.m_noise;
	out[4] = gConfig.m_motionBlur;
}
PBR_API void pbr_set_white_balance(float r, float g, float b)
{
	gConfig.m_whiteBalance[0] = r;
	gConfig.m_whiteBalance[1] = g;
	gConfig.m_whiteBalance[2] = b;
}
PBR_API void pbr_get_white_balance(float* out)
{
	out[0] = gConfig.m_whiteBalance[0];
	out[1] = gConfig.m_whiteBalance[1];
	out[2] = gConfig.m_whiteBalance[2];
}
PBR_API void pbr_set_taa_blend(float v) { gConfig.m_taaBlend = v; }
PBR_API float pbr_get_taa_blend() { return gConfig.m_taaBlend; }
PBR_API float pbr_get_shadow_bias() { return gConfig.m_shadowBias; }
PBR_API float pbr_get_shadow_slope_bias() { return gConfig.m_shadowSlopeBias; }
PBR_API float pbr_get_shadow_softness() { return gConfig.m_shadowSoftness; }

// Markers pybullet_pbr puts in the third specular-colour slot so its payloads can be
// told apart from a plain Bullet specular colour: 2 carries metallic/roughness, 3
// carries emissive luminance. They are stored as separate flags, so setting one does
// not clear the other.
#define PBR_MATERIAL_MARKER 2.0f
#define PBR_EMISSIVE_MARKER 3.0f
#define PBR_TEXTURE_MARKER 4.0f
#define PBR_CLEAR_MARKER 5.0f
#define PBR_FLAG_MATERIAL 1
#define PBR_FLAG_EMISSIVE 2

#define PBR_SSAO_KERNEL_SIZE 24
// The light's orthographic depth range. Bias is configured in world units and scaled
// by this span, which is 299 units wide - biasing in raw depth units is ~300x too
// strong and silently erases every shadow.
#define PBR_SHADOW_NEAR 1.f
#define PBR_SHADOW_FAR 300.f

struct PBRRendererData
{
	GLuint m_forwardProgram;
	GLuint m_tonemapProgram;
	GLuint m_ssaoProgram;
	GLuint m_ssaoBlurProgram;
	GLuint m_fxaaProgram;
	GLuint m_bloomPrefilterProgram;
	GLuint m_bloomBlurProgram;
	GLuint m_shadowProgram;
	GLuint m_lightShadowProgram;
	GLuint m_captureProgram;
	GLuint m_irradianceProgram;
	GLuint m_prefilterProgram;
	GLuint m_brdfProgram;
	GLuint m_backgroundProgram;
	GLuint m_equirectTexture;
	GLuint m_taaProgram;
	GLuint m_prevInstanceVbo;
	GLuint m_fullscreenVao;
	GLuint m_noiseTexture;
	bool m_shaderLoadFailed;
	RenderTarget m_hdr;
	RenderTarget m_ao;
	RenderTarget m_aoBlur;
	RenderTarget m_ldr;
	RenderTarget m_bloomA;
	RenderTarget m_bloomB;
	RenderTarget m_taaHistory;
	RenderTarget m_taaResolve;
	RenderTarget m_shadow;
	RenderTarget m_lightShadowAtlas;
	Environment m_env;
	float m_shadowMatrix[16];
	bool m_shadowValid;
	float m_kernel[PBR_SSAO_KERNEL_SIZE * 3];

	// Renderer-side history for TAA. Bullet keeps only current transforms, so the
	// previous frame's instance positions/orientations are ours to hold - physics
	// state is never involved.
	b3AlignedObjectArray<float> m_prevInstanceData;   // positions then quaternions, vec4 each
	int m_prevInstanceCount;
	float m_prevViewProjection[16];
	bool m_historyValid;
	bool m_prevCameraValid;
	int m_frameIndex;
	float m_lightMatrices[PBR_MAX_LIGHTS * 16];

	// Per-instance materials, indexed by graphics instance uid and uploaded as a
	// texture the vertex shader fetches. Per *instance*, not per registered shape:
	// two bodies built from one collision shape share a shape but must not share a
	// material.
	b3AlignedObjectArray<float> m_materialByUid;      // vec4 per uid: metallic, roughness, marker, spare
	b3AlignedObjectArray<float> m_materialByInstance; // the same, reordered for drawing
	GLuint m_materialVbo;
	int m_materialInstanceCount;
	bool m_materialDirty;

	// forward pass uniforms
	GLint m_uMVP, m_uLightDirection, m_uCameraPosition,
		m_uAmbientIntensity, m_uSunColor, m_uDiagnostic, m_uDiffuse,
		m_uShadowMap, m_uShadowMatrix, m_uShadowEnabled, m_uShadowIntensity,
		m_uShadowTexelSize, m_uShadowBias, m_uShadowSlopeBias, m_uShadowSoftness,
		m_uSunPenumbraScale,
		m_uDefaultMaterial, m_uMaterialMaps, m_uNormalMap, m_uRoughnessMap,
		m_uMetallicMap, m_uAoMap, m_uNormalScale;
	// tonemap pass uniforms
	GLint m_uHdrBuffer, m_uExposure, m_uTonemapper, m_uAoTexture, m_uSsaoEnabled,
		m_uSsaoStrength, m_uAoDebug, m_uVelocityTexture, m_uAutoExposure, m_uExposureKey,
		m_uExposureLimits, m_uBloomStrength, m_uVignette, m_uNoise,
		m_uMotionBlur, m_uFrameSeed, m_uScreenSize, m_uWhiteBalance;
	// ssao pass uniforms
	GLint m_uSsaoDepth, m_uSsaoNoise, m_uSsaoScreenSize, m_uSsaoProjScale, m_uSsaoDepthParams,
		m_uSsaoProjMatrix, m_uSsaoRadius, m_uSsaoBias, m_uSsaoKernelSize,
		m_uSsaoKernel;
	// blur pass uniforms
	GLint m_uBlurAo, m_uBlurDepth, m_uBlurTexelSize, m_uBlurDepthThreshold, m_uBlurDepthParams;
	// fxaa pass uniforms
	GLint m_uBloomPreSource, m_uBloomPreTexel, m_uBloomPreThreshold, m_uBloomPreKnee;
	GLint m_uBloomBlurSource, m_uBloomBlurDirection;
	GLint m_uBloomTexture;
	GLint m_uFxaaScreen, m_uFxaaTexelSize, m_uFxaaSpanMax, m_uFxaaReduceMul, m_uFxaaReduceMin;
	// shadow pass uniforms
	GLint m_uShadowPassMVP;
	GLint m_uLightShadowMVP, m_uLightShadowPosition, m_uLightShadowRange;
	// punctual light uniforms in the forward pass
	GLint m_uLightCount, m_uLightPositionRange, m_uLightColorType, m_uLightDirCone,
		m_uLightConeTile, m_uLightMatrix, m_uLightShadowAtlas, m_uAtlasTileInfo;
	// ibl uniforms in the forward pass
	GLint m_uIrradianceMap, m_uPrefilterMap, m_uBrdfLut, m_uIblEnabled, m_uIblStrength,
		m_uPrefilterMips;
	// background pass uniforms
	GLint m_uBgMap, m_uBgRight, m_uBgUp, m_uBgForward, m_uBgProjScale, m_uBgStrength,
		m_uBgAirless, m_uBgUpAxis;
	// motion-vector uniforms in the forward pass
	GLint m_uViewProjection, m_uPrevViewProjection;
	// taa resolve uniforms
	GLint m_uTaaCurrent, m_uTaaHistory, m_uTaaVelocity, m_uTaaTexelSize, m_uTaaBlend,
		m_uTaaHistoryValid;

	PBRRendererData()
		: m_forwardProgram(0), m_tonemapProgram(0), m_ssaoProgram(0), m_ssaoBlurProgram(0),
		  m_fxaaProgram(0), m_bloomPrefilterProgram(0), m_bloomBlurProgram(0),
		  m_shadowProgram(0), m_lightShadowProgram(0),
		  m_captureProgram(0), m_irradianceProgram(0),
		  m_prefilterProgram(0), m_brdfProgram(0), m_backgroundProgram(0), m_equirectTexture(0),
		  m_taaProgram(0), m_prevInstanceVbo(0), m_fullscreenVao(0), m_noiseTexture(0),
		  m_shaderLoadFailed(false), m_shadowValid(false), m_prevInstanceCount(0),
		  m_historyValid(false), m_prevCameraValid(false), m_frameIndex(0), m_materialVbo(0),
		  m_materialInstanceCount(0), m_materialDirty(true)
	{
	}
};

static std::string shaderDir()
{
	const char* env = getenv("PYBULLET_PBR_SHADER_DIR");
	return std::string(env ? env : PBR_SHADER_DIR);
}

static std::string readFile(const std::string& path, bool* ok)
{
	FILE* f = fopen(path.c_str(), "rb");
	if (!f)
	{
		*ok = false;
		return std::string();
	}
	std::string out;
	char buf[4096];
	size_t n;
	while ((n = fread(buf, 1, sizeof(buf), f)) > 0)
		out.append(buf, n);
	fclose(f);
	*ok = true;
	return out;
}

static GLuint loadProgram(const std::string& dir, const char* vertName, const char* fragName)
{
	bool okV = false, okF = false;
	std::string vs = readFile(dir + "/" + vertName, &okV);
	std::string fs = readFile(dir + "/" + fragName, &okF);
	if (!okV || !okF)
	{
		b3Warning("pybullet_pbr: cannot read %s/%s + %s (set PYBULLET_PBR_SHADER_DIR)",
				  dir.c_str(), vertName, fragName);
		return 0;
	}
	return gltLoadShaderPair(vs.c_str(), fs.c_str());
}

// same convention as GLInstancingRenderer.cpp's file-static helper
static void pbrMatrixMul16(const float aIn[16], const float bIn[16], float result[16])
{
	for (int j = 0; j < 4; j++)
		for (int i = 0; i < 4; i++)
			result[j * 4 + i] = aIn[0 * 4 + i] * bIn[j * 4 + 0] + aIn[1 * 4 + i] * bIn[j * 4 + 1] +
								aIn[2 * 4 + i] * bIn[j * 4 + 2] + aIn[3 * 4 + i] * bIn[j * 4 + 3];
}

// Copied from GLInstancingRenderer.cpp: our light matrices must match the ones the
// base class used when it filled the shadow map, or the shadows land in the wrong place.
static void pbrCreateOrtho(float left, float right, float bottom, float top,
						   float zNear, float zFar, float result[16])
{
	for (int i = 0; i < 16; i++) result[i] = (i % 5) ? 0.f : 1.f;
	result[0] = 2.f / (right - left);
	result[5] = 2.f / (top - bottom);
	result[10] = -2.f / (zFar - zNear);
	result[12] = -(right + left) / (right - left);
	result[13] = -(top + bottom) / (top - bottom);
	result[14] = -(zFar + zNear) / (zFar - zNear);
}

static void pbrCreateLookAt(const b3Vector3& eye, const b3Vector3& center,
							const b3Vector3& up, float result[16])
{
	b3Vector3 f = (center - eye).normalized();
	b3Vector3 u = up.normalized();
	b3Vector3 s = (f.cross(u)).normalized();
	u = s.cross(f);

	result[0] = s.x;  result[4] = s.y;  result[8] = s.z;
	result[1] = u.x;  result[5] = u.y;  result[9] = u.z;
	result[2] = -f.x; result[6] = -f.y; result[10] = -f.z;
	result[3] = 0.f;  result[7] = 0.f;  result[11] = 0.f;
	result[12] = -s.dot(eye);
	result[13] = -u.dot(eye);
	result[14] = f.dot(eye);
	result[15] = 1.f;
}

/// The sun's direction: ours when set from Python, otherwise Bullet's light position.
static b3Vector3 sunDirection(const b3Vector3& bulletLightPos)
{
	if (gConfig.m_sunDirectionSet)
		return b3MakeVector3(gConfig.m_sunDirection[0], gConfig.m_sunDirection[1],
							 gConfig.m_sunDirection[2]);
	return bulletLightPos.normalized();
}

static float srgbToLinear(float c)
{
	return c <= 0.04045f ? c / 12.92f : powf((c + 0.055f) / 1.055f, 2.4f);
}

/// Cosine-ish hemisphere kernel plus a 4x4 rotation-noise texture. Both are fixed
/// at startup: the kernel is uploaded once per frame as a uniform array, the noise
/// tiles over the screen so neighbouring pixels use different rotations (the blur
/// pass then averages the resulting dither away).
void PBRInstancingRenderer::initSsaoKernel()
{
	if (m_pbr->m_noiseTexture)
		return;

	unsigned int seed = 1234567u;  // deterministic: renders must be reproducible
	struct Rnd
	{
		unsigned int s;
		float next() { s = s * 1664525u + 1013904223u; return (float)(s >> 8) / 16777216.f; }
	} rnd{seed};

	for (int i = 0; i < PBR_SSAO_KERNEL_SIZE; i++)
	{
		float x = rnd.next() * 2.f - 1.f;
		float y = rnd.next() * 2.f - 1.f;
		float z = rnd.next();  // hemisphere: +z only
		float len = sqrtf(x * x + y * y + z * z);
		if (len < 1e-5f) { x = 0; y = 0; z = 1; len = 1; }
		// pack samples towards the origin so nearby geometry dominates
		float scale = 0.1f + 0.9f * ((float)i / PBR_SSAO_KERNEL_SIZE) * ((float)i / PBR_SSAO_KERNEL_SIZE);
		m_pbr->m_kernel[i * 3 + 0] = x / len * scale;
		m_pbr->m_kernel[i * 3 + 1] = y / len * scale;
		m_pbr->m_kernel[i * 3 + 2] = z / len * scale;
	}

	float noise[4 * 4 * 3];
	for (int i = 0; i < 16; i++)
	{
		noise[i * 3 + 0] = rnd.next();
		noise[i * 3 + 1] = rnd.next();
		noise[i * 3 + 2] = 0.5f;  // rotation around the normal only
	}
	glGenTextures(1, &m_pbr->m_noiseTexture);
	glBindTexture(GL_TEXTURE_2D, m_pbr->m_noiseTexture);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB16F, 4, 4, 0, GL_RGB, GL_FLOAT, noise);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glBindTexture(GL_TEXTURE_2D, 0);
}

PBRInstancingRenderer::PBRInstancingRenderer(int maxObjectCapacity, int maxShapeCapacityInBytes)
	: GLInstancingRenderer(maxObjectCapacity, maxShapeCapacityInBytes),
	  m_pbr(new PBRRendererData())
{
	gRenderer = this;
	b3Printf("pybullet_pbr: PBRInstancingRenderer active");
}

PBRInstancingRenderer::~PBRInstancingRenderer()
{
	if (m_pbr->m_forwardProgram) glDeleteProgram(m_pbr->m_forwardProgram);
	if (m_pbr->m_tonemapProgram) glDeleteProgram(m_pbr->m_tonemapProgram);
	if (m_pbr->m_ssaoProgram) glDeleteProgram(m_pbr->m_ssaoProgram);
	if (m_pbr->m_ssaoBlurProgram) glDeleteProgram(m_pbr->m_ssaoBlurProgram);
	if (m_pbr->m_fxaaProgram) glDeleteProgram(m_pbr->m_fxaaProgram);
	if (m_pbr->m_bloomPrefilterProgram) glDeleteProgram(m_pbr->m_bloomPrefilterProgram);
	if (m_pbr->m_bloomBlurProgram) glDeleteProgram(m_pbr->m_bloomBlurProgram);
	if (m_pbr->m_shadowProgram) glDeleteProgram(m_pbr->m_shadowProgram);
	if (m_pbr->m_lightShadowProgram) glDeleteProgram(m_pbr->m_lightShadowProgram);
	if (m_pbr->m_captureProgram) glDeleteProgram(m_pbr->m_captureProgram);
	if (m_pbr->m_irradianceProgram) glDeleteProgram(m_pbr->m_irradianceProgram);
	if (m_pbr->m_prefilterProgram) glDeleteProgram(m_pbr->m_prefilterProgram);
	if (m_pbr->m_brdfProgram) glDeleteProgram(m_pbr->m_brdfProgram);
	if (m_pbr->m_backgroundProgram) glDeleteProgram(m_pbr->m_backgroundProgram);
	if (m_pbr->m_equirectTexture) glDeleteTextures(1, &m_pbr->m_equirectTexture);
	if (m_pbr->m_taaProgram) glDeleteProgram(m_pbr->m_taaProgram);
	if (m_pbr->m_prevInstanceVbo) glDeleteBuffers(1, &m_pbr->m_prevInstanceVbo);
	if (m_pbr->m_materialVbo) glDeleteBuffers(1, &m_pbr->m_materialVbo);
	for (int i = 0; i < gTextureRequests.size(); i++)
		if (gTextureRequests[i].m_glTexture)
		{
			glDeleteTextures(1, &gTextureRequests[i].m_glTexture);
			gTextureRequests[i].m_glTexture = 0;
			gTextureRequests[i].m_failed = 0;
		}
	if (m_pbr->m_noiseTexture) glDeleteTextures(1, &m_pbr->m_noiseTexture);
	if (m_pbr->m_fullscreenVao) glDeleteVertexArrays(1, &m_pbr->m_fullscreenVao);
	if (gRenderer == this)
		gRenderer = 0;
	delete m_pbr;
}

bool PBRInstancingRenderer::loadShaders()
{
	if (m_pbr->m_forwardProgram && m_pbr->m_tonemapProgram)
		return true;
	if (m_pbr->m_shaderLoadFailed)
		return false;

	std::string dir = shaderDir();
	m_pbr->m_forwardProgram = loadProgram(dir, "forward.vert", "forward.frag");
	m_pbr->m_tonemapProgram = loadProgram(dir, "fullscreen.vert", "tonemap.frag");
	m_pbr->m_ssaoProgram = loadProgram(dir, "fullscreen.vert", "ssao.frag");
	m_pbr->m_ssaoBlurProgram = loadProgram(dir, "fullscreen.vert", "ssao_blur.frag");
	m_pbr->m_fxaaProgram = loadProgram(dir, "fullscreen.vert", "fxaa.frag");
	m_pbr->m_shadowProgram = loadProgram(dir, "shadow.vert", "shadow.frag");
	m_pbr->m_bloomPrefilterProgram = loadProgram(dir, "fullscreen.vert", "bloom_prefilter.frag");
	m_pbr->m_bloomBlurProgram = loadProgram(dir, "fullscreen.vert", "bloom_blur.frag");
	m_pbr->m_lightShadowProgram = loadProgram(dir, "shadow.vert", "light_shadow.frag");
	m_pbr->m_captureProgram = loadProgram(dir, "fullscreen.vert", "env_capture.frag");
	m_pbr->m_irradianceProgram = loadProgram(dir, "fullscreen.vert", "env_irradiance.frag");
	m_pbr->m_prefilterProgram = loadProgram(dir, "fullscreen.vert", "env_prefilter.frag");
	m_pbr->m_brdfProgram = loadProgram(dir, "fullscreen.vert", "env_brdf.frag");
	m_pbr->m_backgroundProgram = loadProgram(dir, "fullscreen.vert", "background.frag");
	m_pbr->m_taaProgram = loadProgram(dir, "fullscreen.vert", "taa.frag");
	if (!m_pbr->m_forwardProgram || !m_pbr->m_tonemapProgram || !m_pbr->m_ssaoProgram ||
		!m_pbr->m_ssaoBlurProgram || !m_pbr->m_fxaaProgram || !m_pbr->m_shadowProgram ||
		!m_pbr->m_captureProgram || !m_pbr->m_irradianceProgram || !m_pbr->m_prefilterProgram ||
		!m_pbr->m_brdfProgram || !m_pbr->m_backgroundProgram || !m_pbr->m_taaProgram ||
		!m_pbr->m_lightShadowProgram || !m_pbr->m_bloomPrefilterProgram ||
		!m_pbr->m_bloomBlurProgram)
	{
		b3Warning("pybullet_pbr: shader setup failed, falling back to the stock renderer");
		m_pbr->m_shaderLoadFailed = true;
		return false;
	}

	GLuint p = m_pbr->m_forwardProgram;
	m_pbr->m_uMVP = glGetUniformLocation(p, "MVP");
	m_pbr->m_uLightDirection = glGetUniformLocation(p, "lightDirection");
	m_pbr->m_uCameraPosition = glGetUniformLocation(p, "cameraPosition");
	m_pbr->m_uSunColor = glGetUniformLocation(p, "sunColor");
	m_pbr->m_uAmbientIntensity = glGetUniformLocation(p, "ambientIntensity");
	m_pbr->m_uDiagnostic = glGetUniformLocation(p, "diagnosticMode");
	m_pbr->m_uDiffuse = glGetUniformLocation(p, "Diffuse");
	m_pbr->m_uDefaultMaterial = glGetUniformLocation(p, "defaultMaterial");
	m_pbr->m_uMaterialMaps = glGetUniformLocation(p, "materialMaps");
	m_pbr->m_uNormalMap = glGetUniformLocation(p, "normalMap");
	m_pbr->m_uRoughnessMap = glGetUniformLocation(p, "roughnessMap");
	m_pbr->m_uMetallicMap = glGetUniformLocation(p, "metallicMap");
	m_pbr->m_uAoMap = glGetUniformLocation(p, "aoMap");
	m_pbr->m_uNormalScale = glGetUniformLocation(p, "normalScale");
	m_pbr->m_uShadowMap = glGetUniformLocation(p, "shadowMap");
	m_pbr->m_uShadowMatrix = glGetUniformLocation(p, "shadowMatrix");
	m_pbr->m_uShadowEnabled = glGetUniformLocation(p, "shadowEnabled");
	m_pbr->m_uShadowIntensity = glGetUniformLocation(p, "shadowIntensity");
	m_pbr->m_uShadowTexelSize = glGetUniformLocation(p, "shadowTexelSize");
	m_pbr->m_uShadowBias = glGetUniformLocation(p, "shadowBias");
	m_pbr->m_uShadowSlopeBias = glGetUniformLocation(p, "shadowSlopeBias");
	m_pbr->m_uShadowSoftness = glGetUniformLocation(p, "shadowSoftness");
	m_pbr->m_uSunPenumbraScale = glGetUniformLocation(p, "sunPenumbraScale");
	m_pbr->m_uIrradianceMap = glGetUniformLocation(p, "irradianceMap");
	m_pbr->m_uPrefilterMap = glGetUniformLocation(p, "prefilterMap");
	m_pbr->m_uBrdfLut = glGetUniformLocation(p, "brdfLut");
	m_pbr->m_uIblEnabled = glGetUniformLocation(p, "iblEnabled");
	m_pbr->m_uIblStrength = glGetUniformLocation(p, "iblStrength");
	m_pbr->m_uPrefilterMips = glGetUniformLocation(p, "prefilterMips");
	m_pbr->m_uViewProjection = glGetUniformLocation(p, "viewProjection");
	m_pbr->m_uPrevViewProjection = glGetUniformLocation(p, "prevViewProjection");
	m_pbr->m_uLightCount = glGetUniformLocation(p, "lightCount");
	m_pbr->m_uLightPositionRange = glGetUniformLocation(p, "lightPositionRange");
	m_pbr->m_uLightColorType = glGetUniformLocation(p, "lightColorType");
	m_pbr->m_uLightDirCone = glGetUniformLocation(p, "lightDirCone");
	m_pbr->m_uLightConeTile = glGetUniformLocation(p, "lightConeTile");
	m_pbr->m_uLightMatrix = glGetUniformLocation(p, "lightMatrix");
	m_pbr->m_uLightShadowAtlas = glGetUniformLocation(p, "lightShadowAtlas");
	m_pbr->m_uAtlasTileInfo = glGetUniformLocation(p, "atlasTileInfo");

	p = m_pbr->m_tonemapProgram;
	m_pbr->m_uHdrBuffer = glGetUniformLocation(p, "hdrBuffer");
	m_pbr->m_uExposure = glGetUniformLocation(p, "exposure");
	m_pbr->m_uTonemapper = glGetUniformLocation(p, "tonemapper");
	m_pbr->m_uAoTexture = glGetUniformLocation(p, "aoTexture");
	m_pbr->m_uSsaoEnabled = glGetUniformLocation(p, "ssaoEnabled");
	m_pbr->m_uSsaoStrength = glGetUniformLocation(p, "ssaoStrength");
	m_pbr->m_uAoDebug = glGetUniformLocation(p, "aoDebug");
	m_pbr->m_uVelocityTexture = glGetUniformLocation(p, "velocityTexture");
	m_pbr->m_uAutoExposure = glGetUniformLocation(p, "autoExposure");
	m_pbr->m_uExposureKey = glGetUniformLocation(p, "exposureKey");
	m_pbr->m_uExposureLimits = glGetUniformLocation(p, "exposureLimits");
	m_pbr->m_uBloomStrength = glGetUniformLocation(p, "bloomStrength");
	m_pbr->m_uVignette = glGetUniformLocation(p, "vignetteStrength");
	m_pbr->m_uNoise = glGetUniformLocation(p, "noiseStrength");
	m_pbr->m_uMotionBlur = glGetUniformLocation(p, "motionBlur");
	m_pbr->m_uFrameSeed = glGetUniformLocation(p, "frameSeed");
	m_pbr->m_uScreenSize = glGetUniformLocation(p, "screenSize");
	m_pbr->m_uWhiteBalance = glGetUniformLocation(p, "whiteBalance");
	m_pbr->m_uBloomTexture = glGetUniformLocation(p, "bloomTexture");

	p = m_pbr->m_bloomPrefilterProgram;
	m_pbr->m_uBloomPreSource = glGetUniformLocation(p, "hdrBuffer");
	m_pbr->m_uBloomPreTexel = glGetUniformLocation(p, "texelSize");
	m_pbr->m_uBloomPreThreshold = glGetUniformLocation(p, "threshold");
	m_pbr->m_uBloomPreKnee = glGetUniformLocation(p, "knee");

	p = m_pbr->m_bloomBlurProgram;
	m_pbr->m_uBloomBlurSource = glGetUniformLocation(p, "source");
	m_pbr->m_uBloomBlurDirection = glGetUniformLocation(p, "direction");

	p = m_pbr->m_ssaoProgram;
	m_pbr->m_uSsaoDepth = glGetUniformLocation(p, "depthTexture");
	m_pbr->m_uSsaoNoise = glGetUniformLocation(p, "noiseTexture");
	m_pbr->m_uSsaoScreenSize = glGetUniformLocation(p, "screenSize");
	m_pbr->m_uSsaoProjScale = glGetUniformLocation(p, "projScale");
	m_pbr->m_uSsaoDepthParams = glGetUniformLocation(p, "depthParams");
	m_pbr->m_uSsaoProjMatrix = glGetUniformLocation(p, "projMatrix");
	m_pbr->m_uSsaoRadius = glGetUniformLocation(p, "radius");
	m_pbr->m_uSsaoBias = glGetUniformLocation(p, "bias");
	m_pbr->m_uSsaoKernelSize = glGetUniformLocation(p, "kernelSize");
	m_pbr->m_uSsaoKernel = glGetUniformLocation(p, "kernel");

	p = m_pbr->m_ssaoBlurProgram;
	m_pbr->m_uBlurAo = glGetUniformLocation(p, "aoTexture");
	m_pbr->m_uBlurDepth = glGetUniformLocation(p, "depthTexture");
	m_pbr->m_uBlurTexelSize = glGetUniformLocation(p, "texelSize");
	m_pbr->m_uBlurDepthThreshold = glGetUniformLocation(p, "depthThreshold");
	m_pbr->m_uBlurDepthParams = glGetUniformLocation(p, "depthParams");

	p = m_pbr->m_fxaaProgram;
	m_pbr->m_uFxaaScreen = glGetUniformLocation(p, "screenTexture");
	m_pbr->m_uFxaaTexelSize = glGetUniformLocation(p, "texelSize");
	m_pbr->m_uFxaaSpanMax = glGetUniformLocation(p, "spanMax");
	m_pbr->m_uFxaaReduceMul = glGetUniformLocation(p, "reduceMul");
	m_pbr->m_uFxaaReduceMin = glGetUniformLocation(p, "reduceMin");

	m_pbr->m_uShadowPassMVP = glGetUniformLocation(m_pbr->m_shadowProgram, "MVP");
	p = m_pbr->m_lightShadowProgram;
	m_pbr->m_uLightShadowMVP = glGetUniformLocation(p, "MVP");
	m_pbr->m_uLightShadowPosition = glGetUniformLocation(p, "lightPosition");
	m_pbr->m_uLightShadowRange = glGetUniformLocation(p, "lightRange");

	p = m_pbr->m_backgroundProgram;
	m_pbr->m_uBgMap = glGetUniformLocation(p, "environmentMap");
	m_pbr->m_uBgRight = glGetUniformLocation(p, "cameraRight");
	m_pbr->m_uBgUp = glGetUniformLocation(p, "cameraUp");
	m_pbr->m_uBgForward = glGetUniformLocation(p, "cameraForward");
	m_pbr->m_uBgProjScale = glGetUniformLocation(p, "projScale");
	m_pbr->m_uBgStrength = glGetUniformLocation(p, "strength");
	m_pbr->m_uBgAirless = glGetUniformLocation(p, "airless");
	m_pbr->m_uBgUpAxis = glGetUniformLocation(p, "upAxis");

	p = m_pbr->m_taaProgram;
	m_pbr->m_uTaaCurrent = glGetUniformLocation(p, "currentTexture");
	m_pbr->m_uTaaHistory = glGetUniformLocation(p, "historyTexture");
	m_pbr->m_uTaaVelocity = glGetUniformLocation(p, "velocityTexture");
	m_pbr->m_uTaaTexelSize = glGetUniformLocation(p, "texelSize");
	m_pbr->m_uTaaBlend = glGetUniformLocation(p, "blend");
	m_pbr->m_uTaaHistoryValid = glGetUniformLocation(p, "historyValid");

	glGenBuffers(1, &m_pbr->m_prevInstanceVbo);
	glGenBuffers(1, &m_pbr->m_materialVbo);

	glGenVertexArrays(1, &m_pbr->m_fullscreenVao);
	initSsaoKernel();
	return true;
}

/// Loads a requested texture the first time it is needed and returns its GL name.
/// A file that cannot be read is reported once and then treated as absent.
unsigned int PBRInstancingRenderer::resolveTexture(int handle)
{
	if (handle < 0 || handle >= gTextureRequests.size())
		return 0;
	PbrTextureRequest& request = gTextureRequests[handle];
	if (request.m_glTexture || request.m_failed)
		return request.m_glTexture;

	int width = 0, height = 0, channels = 0;
	unsigned char* pixels = stbi_load(request.m_path, &width, &height, &channels, 4);
	if (!pixels)
	{
		b3Warning("pybullet_pbr: cannot read texture '%s'", request.m_path);
		request.m_failed = 1;
		return 0;
	}

	GLuint tex = 0;
	glGenTextures(1, &tex);
	glBindTexture(GL_TEXTURE_2D, tex);
	// Colour maps are authored in sRGB; normal, roughness, metallic and AO maps carry
	// measurements, not colour, and must stay linear or the values come out wrong.
	glTexImage2D(GL_TEXTURE_2D, 0, request.m_srgb ? GL_SRGB8_ALPHA8 : GL_RGBA8, width,
				 height, 0, GL_RGBA, GL_UNSIGNED_BYTE, pixels);
	glGenerateMipmap(GL_TEXTURE_2D);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glBindTexture(GL_TEXTURE_2D, 0);
	stbi_image_free(pixels);

	request.m_glTexture = tex;
	b3Printf("pybullet_pbr: texture '%s' (%dx%d)", request.m_path, width, height);
	return tex;
}

/// Binds this shape's extra maps and tells the shader which ones exist.
void PBRInstancingRenderer::bindShapeMaterialTextures(int shapeIndex)
{
	PbrShapeTextures* entry = pbrShapeTextures(shapeIndex, false);
	int present = 0;
	static const GLenum units[PBR_TEX_SLOTS] = {GL_TEXTURE7, GL_TEXTURE8, GL_TEXTURE9,
												GL_TEXTURE10};
	for (int slot = 0; slot < PBR_TEX_SLOTS; slot++)
	{
		unsigned int tex = entry ? resolveTexture(entry->m_handle[slot]) : 0;
		glActiveTexture(units[slot]);
		glBindTexture(GL_TEXTURE_2D, tex);
		if (tex)
			present |= (1 << slot);
	}
	glActiveTexture(GL_TEXTURE0);
	glUniform1i(m_pbr->m_uMaterialMaps, present);
}

void PBRInstancingRenderer::bindShapeTexture(struct b3GraphicsInstance* gfxObj)
{
	glActiveTexture(GL_TEXTURE0);
	if ((gfxObj->m_flags & B3_INSTANCE_TEXTURE) && gfxObj->m_textureIndex >= 0)
	{
		const InternalTextureHandle& tex = m_data->m_textureHandles[gfxObj->m_textureIndex];
		glBindTexture(GL_TEXTURE_2D, tex.m_glTexture);
		if (tex.m_enableFiltering)
		{
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_BASE_LEVEL, 0);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		}
		else
		{
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
		}
	}
	else
	{
		glBindTexture(GL_TEXTURE_2D, m_data->m_defaultTexturehandle);
	}
}

void PBRInstancingRenderer::recordInstanceMaterial(int instanceUid, float metallic,
												   float roughness, float marker)
{
	if (getenv("PBR_DEBUG_MATERIAL"))
		b3Printf("pybullet_pbr: material uid=%d metallic=%.2f roughness=%.2f marker=%.2f",
				 instanceUid, metallic, roughness, marker);
	if (instanceUid < 0 || instanceUid > 1000000)
		return;
	int needed = (instanceUid + 1) * 4;
	if (m_pbr->m_materialByUid.size() < needed)
	{
		int old = m_pbr->m_materialByUid.size();
		m_pbr->m_materialByUid.resize(needed);
		for (int i = old; i < needed; i++)
			m_pbr->m_materialByUid[i] = 0.f;
	}
	int flags = (int)(m_pbr->m_materialByUid[instanceUid * 4 + 2] + 0.5f);
	if (fabsf(marker - PBR_CLEAR_MARKER) < 1e-3f)
	{
		for (int c = 0; c < 4; c++)
			m_pbr->m_materialByUid[instanceUid * 4 + c] = 0.f;
		m_pbr->m_materialDirty = true;
		return;
	}
	if (fabsf(marker - PBR_MATERIAL_MARKER) < 1e-3f)
	{
		m_pbr->m_materialByUid[instanceUid * 4 + 0] = metallic;
		m_pbr->m_materialByUid[instanceUid * 4 + 1] = roughness;
		flags |= PBR_FLAG_MATERIAL;
	}
	else if (fabsf(marker - PBR_EMISSIVE_MARKER) < 1e-3f)
	{
		m_pbr->m_materialByUid[instanceUid * 4 + 3] = metallic;  // emissive luminance
		flags |= PBR_FLAG_EMISSIVE;
		if (metallic <= 0.f)
			flags &= ~PBR_FLAG_EMISSIVE;
	}
	else if (fabsf(marker - PBR_TEXTURE_MARKER) < 1e-3f)
	{
		// (slot, handle) for an extra material map. Stored against the instance's
		// registered shape, which is where textures live.
		int slot = (int)(metallic + 0.5f);
		int handle = (int)(roughness + 0.5f);
		if (slot >= 0 && slot < PBR_TEX_SLOTS)
		{
			b3PublicGraphicsInstance* pg =
				instanceUid < m_data->m_publicGraphicsInstances.getNumHandles()
					? m_data->m_publicGraphicsInstances.getHandle(instanceUid) : 0;
			if (pg)
			{
				PbrShapeTextures* entry = pbrShapeTextures(pg->m_shapeIndex, true);
				entry->m_handle[slot] = handle;
			}
		}
		return;
	}
	else
	{
		return;  // a plain Bullet specular colour, not one of ours
	}
	m_pbr->m_materialByUid[instanceUid * 4 + 2] = (float)flags;
	m_pbr->m_materialDirty = true;
}

static void pbr_apply_pending_materials(int graphicsUid, int bodyUniqueId, int linkIndex)
{
	if (!gRenderer)
		return;
	for (int i = 0; i < gPendingMaterials.size(); i++)
	{
		const PbrPendingMaterial& pending = gPendingMaterials[i];
		if (pending.m_body != bodyUniqueId)
			continue;
		if (pending.m_link != -2 && pending.m_link != linkIndex)
			continue;
		gRenderer->recordInstanceMaterial(graphicsUid, pending.m_a, pending.m_b,
										  pending.m_marker);
	}
}

PBR_API int pbr_set_material_by_body(int bodyUniqueId, int linkIndex, float a, float b,
									 float marker)
{
	// Clearing also has to drop any request that has not been applied yet, or the old
	// material would come back when the instance is registered.
	if (fabsf(marker - PBR_CLEAR_MARKER) < 1e-3f)
	{
		for (int i = gPendingMaterials.size() - 1; i >= 0; i--)
		{
			const PbrPendingMaterial& pending = gPendingMaterials[i];
			if (pending.m_body == bodyUniqueId &&
				(linkIndex == -2 || pending.m_link == linkIndex))
				gPendingMaterials.removeAtIndex(i);
		}
	}

	// Remember it whether or not an instance exists yet, replacing any earlier request
	// for the same body, link and payload kind.
	bool replaced = false;
	for (int i = 0; i < gPendingMaterials.size(); i++)
	{
		PbrPendingMaterial& pending = gPendingMaterials[i];
		if (pending.m_body == bodyUniqueId && pending.m_link == linkIndex &&
			fabsf(pending.m_marker - marker) < 1e-3f)
		{
			pending.m_a = a;
			pending.m_b = b;
			replaced = true;
			break;
		}
	}
	if (!replaced)
	{
		PbrPendingMaterial pending;
		pending.m_body = bodyUniqueId;
		pending.m_link = linkIndex;
		pending.m_a = a;
		pending.m_b = b;
		pending.m_marker = marker;
		gPendingMaterials.push_back(pending);
	}

	if (!gRenderer)
		return 0;
	int applied = 0;
	for (int i = 0; i < gInstanceKeys.size(); i++)
	{
		const PbrInstanceKey& key = gInstanceKeys[i];
		if (key.m_body != bodyUniqueId)
			continue;
		if (linkIndex != -2 && key.m_link != linkIndex)   // -2 means every link
			continue;
		gRenderer->recordInstanceMaterial(key.m_uid, a, b, marker);
		applied++;
	}
	return applied;
}

void PBRInstancingRenderer::writeSingleInstanceSpecularColorToCPU(const float* specular,
																  int srcIndex2)
{
	// [metallic, roughness, PBR_MATERIAL_MARKER] means "this is a PBR material";
	// anything else is a plain specular colour and keeps Bullet's meaning.
	recordInstanceMaterial(srcIndex2, specular[0], specular[1], specular[2]);
	GLInstancingRenderer::writeSingleInstanceSpecularColorToCPU(specular, srcIndex2);
}

void PBRInstancingRenderer::writeSingleInstanceSpecularColorToCPU(const double* specular,
																  int srcIndex2)
{
	recordInstanceMaterial(srcIndex2, (float)specular[0], (float)specular[1],
						   (float)specular[2]);
	GLInstancingRenderer::writeSingleInstanceSpecularColorToCPU(specular, srcIndex2);
}

/// Reorders the per-uid materials into draw order and uploads them as an instanced
/// vertex attribute.
///
/// Materials arrive keyed by graphics instance uid, but instances are drawn in
/// internal-index order, and Bullet renumbers those whenever instances are added or
/// removed. The pool maps one to the other. (Indexing a table by uid inside the shader
/// would be neater, but the uid Bullet packs into `instance_scale_obUid.w` does not
/// survive in this build - the same reason pybullet's own segmentation mask comes back
/// empty here.)
void PBRInstancingRenderer::uploadMaterialTable()
{
	int totalNumInstances = totalInstanceCount();
	if (!m_pbr->m_materialDirty && m_pbr->m_materialInstanceCount == totalNumInstances)
		return;

	m_pbr->m_materialByInstance.resize(totalNumInstances * 4);
	for (int i = 0; i < totalNumInstances * 4; i++)
		m_pbr->m_materialByInstance[i] = 0.f;

	int uidCount = m_pbr->m_materialByUid.size() / 4;
	int poolSize = m_data->m_publicGraphicsInstances.getNumHandles();
	for (int uid = 0; uid < uidCount; uid++)
	{
		if (m_pbr->m_materialByUid[uid * 4 + 2] == 0.f)
			continue;  // neither a material nor an emissive was ever set
		if (uid >= poolSize)
			continue;
		b3PublicGraphicsInstance* pg = m_data->m_publicGraphicsInstances.getHandle(uid);
		if (!pg)
			continue;
		int index = pg->m_internalInstanceIndex;
		if (index < 0 || index >= totalNumInstances)
			continue;
		for (int c = 0; c < 4; c++)
			m_pbr->m_materialByInstance[index * 4 + c] = m_pbr->m_materialByUid[uid * 4 + c];
	}

	glBindBuffer(GL_ARRAY_BUFFER, m_pbr->m_materialVbo);
	glBufferData(GL_ARRAY_BUFFER, totalNumInstances * 4 * sizeof(float),
				 totalNumInstances ? &m_pbr->m_materialByInstance[0] : 0, GL_DYNAMIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	m_pbr->m_materialInstanceCount = totalNumInstances;
	m_pbr->m_materialDirty = false;
}

int PBRInstancingRenderer::totalInstanceCount() const
{
	int total = 0;
	for (int i = 0; i < m_graphicsInstances.size(); i++)
		total += m_graphicsInstances[i]->m_numGraphicsInstances;
	return total;
}

void PBRInstancingRenderer::drawInstances()
{
	int totalNumInstances = totalInstanceCount();
	if (!totalNumInstances)
		return;

	const int POSITION_BUFFER_SIZE = totalNumInstances * 4 * sizeof(float);
	const int ORIENTATION_BUFFER_SIZE = POSITION_BUFFER_SIZE;
	const int COLOR_BUFFER_SIZE = POSITION_BUFFER_SIZE;
	const int shapeBytes = m_data->m_maxShapeCapacityInBytes;
	const int vertexStride = 9 * sizeof(float);

	glBindBuffer(GL_ARRAY_BUFFER, m_data->m_vbo);

	int instanceOffset = 0;
	for (int obj = 0; obj < m_graphicsInstances.size(); obj++)
	{
		b3GraphicsInstance* gfxObj = m_graphicsInstances[obj];
		if (!gfxObj->m_numGraphicsInstances)
			continue;

		// Points and point sprites are drawn by the base renderer, which has the
		// point-size handling this pass does not.
		if (gfxObj->m_primitiveType != B3_GL_TRIANGLES)
		{
			instanceOffset += gfxObj->m_numGraphicsInstances;
			continue;
		}

		bindShapeTexture(gfxObj);
		bindShapeMaterialTextures(obj);
		glBindVertexArray(gfxObj->m_cube_vao);

		size_t vertexBase = (size_t)gfxObj->m_vertexArrayOffset * vertexStride;
		size_t instBase = (size_t)shapeBytes + (size_t)instanceOffset * 4 * sizeof(float);

		glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, vertexStride, (GLvoid*)vertexBase);
		glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 0, (GLvoid*)instBase);
		glVertexAttribPointer(2, 4, GL_FLOAT, GL_FALSE, 0, (GLvoid*)(instBase + POSITION_BUFFER_SIZE));
		glVertexAttribPointer(3, 2, GL_FLOAT, GL_FALSE, vertexStride, (GLvoid*)(vertexBase + 7 * sizeof(float)));
		glVertexAttribPointer(4, 3, GL_FLOAT, GL_FALSE, vertexStride, (GLvoid*)(vertexBase + 4 * sizeof(float)));
		glVertexAttribPointer(5, 4, GL_FLOAT, GL_FALSE, 0, (GLvoid*)(instBase + POSITION_BUFFER_SIZE + ORIENTATION_BUFFER_SIZE));
		glVertexAttribPointer(6, 4, GL_FLOAT, GL_FALSE, 0, (GLvoid*)(instBase + POSITION_BUFFER_SIZE + ORIENTATION_BUFFER_SIZE + COLOR_BUFFER_SIZE));

		const int divisors[7] = {0, 1, 1, 0, 0, 1, 1};
		for (int a = 0; a <= 6; a++)
		{
			glEnableVertexAttribArray(a);
			glVertexAttribDivisor(a, divisors[a]);
		}

		if (m_pbr->m_materialInstanceCount == totalNumInstances && m_pbr->m_materialVbo)
		{
			glBindBuffer(GL_ARRAY_BUFFER, m_pbr->m_materialVbo);
			glVertexAttribPointer(9, 4, GL_FLOAT, GL_FALSE, 0,
								  (GLvoid*)((size_t)instanceOffset * 4 * sizeof(float)));
			glEnableVertexAttribArray(9);
			glVertexAttribDivisor(9, 1);
			glBindBuffer(GL_ARRAY_BUFFER, m_data->m_vbo);
		}

		// Previous-frame transforms live in our own buffer; glVertexAttribPointer binds
		// whatever is on GL_ARRAY_BUFFER right now, so the two buffers can be mixed
		// inside one VAO.
		if (m_pbr->m_prevInstanceCount == totalNumInstances && m_pbr->m_prevInstanceVbo)
		{
			glBindBuffer(GL_ARRAY_BUFFER, m_pbr->m_prevInstanceVbo);
			size_t prevBase = (size_t)instanceOffset * 4 * sizeof(float);
			glVertexAttribPointer(7, 4, GL_FLOAT, GL_FALSE, 0, (GLvoid*)prevBase);
			glVertexAttribPointer(8, 4, GL_FLOAT, GL_FALSE, 0,
								  (GLvoid*)(prevBase + POSITION_BUFFER_SIZE));
			glEnableVertexAttribArray(7);
			glEnableVertexAttribArray(8);
			glVertexAttribDivisor(7, 1);
			glVertexAttribDivisor(8, 1);
			glBindBuffer(GL_ARRAY_BUFFER, m_data->m_vbo);
		}

		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, gfxObj->m_index_vbo);
		glDrawElementsInstanced(GL_TRIANGLES, gfxObj->m_numIndices, GL_UNSIGNED_INT, 0,
								gfxObj->m_numGraphicsInstances);

		instanceOffset += gfxObj->m_numGraphicsInstances;
	}
	glBindVertexArray(0);
}

/// Drives Environment::build(): binds the right program for each stage and draws one
/// fullscreen triangle per cube face.
struct EnvFaceDrawer
{
	PBRRendererData* d;
	GLuint program;
	GLint faceLocation;
	int upAxis;
	float sunDir[3];

	void use(GLuint p)
	{
		program = p;
		glUseProgram(p);
		faceLocation = glGetUniformLocation(p, "face");
	}

	void beginCapture(GLuint equirect)
	{
		use(d->m_captureProgram);
		glUniform1i(glGetUniformLocation(program, "hasTexture"), equirect ? 1 : 0);
		glUniform1i(glGetUniformLocation(program, "upAxis"), upAxis);
		glUniform3f(glGetUniformLocation(program, "sunDirection"), sunDir[0], sunDir[1], sunDir[2]);
		glUniform3fv(glGetUniformLocation(program, "zenithColor"), 1, gConfig.m_skyZenith);
		glUniform3fv(glGetUniformLocation(program, "horizonColor"), 1, gConfig.m_skyHorizon);
		glUniform3fv(glGetUniformLocation(program, "groundColor"), 1, gConfig.m_skyGround);
		glUniform3fv(glGetUniformLocation(program, "sunColor"), 1, gConfig.m_sunColor);
		glUniform1f(glGetUniformLocation(program, "sunGlow"), gConfig.m_skyGlow);
		glUniform1i(glGetUniformLocation(program, "airless"), gConfig.m_skyAirless);
		if (equirect)
		{
			glActiveTexture(GL_TEXTURE0);
			glBindTexture(GL_TEXTURE_2D, equirect);
			glUniform1i(glGetUniformLocation(program, "equirect"), 0);
		}
	}

	void beginIrradiance(GLuint cubemap)
	{
		use(d->m_irradianceProgram);
		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_CUBE_MAP, cubemap);
		glUniform1i(glGetUniformLocation(program, "environmentMap"), 0);
	}

	void beginPrefilter(GLuint cubemap, float resolution)
	{
		use(d->m_prefilterProgram);
		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_CUBE_MAP, cubemap);
		glUniform1i(glGetUniformLocation(program, "environmentMap"), 0);
		glUniform1f(glGetUniformLocation(program, "resolution"), resolution);
	}

	void setRoughness(float r)
	{
		glUniform1f(glGetUniformLocation(program, "roughness"), r);
	}

	void beginBrdf() { use(d->m_brdfProgram); }

	void face(int f)
	{
		if (faceLocation >= 0)
			glUniform1i(faceLocation, f);
		glDrawArrays(GL_TRIANGLES, 0, 3);
	}
};

/// Loads an equirectangular .hdr/.exr-style float image, if one was requested.
/// Returns 0 for "use the procedural sky", which is also the fallback when a file
/// cannot be read - a missing file must not leave the scene unlit.
GLuint PBRInstancingRenderer::loadEquirect(const char* path)
{
	if (m_pbr->m_equirectTexture)
	{
		glDeleteTextures(1, &m_pbr->m_equirectTexture);
		m_pbr->m_equirectTexture = 0;
	}
	if (!path || !path[0])
		return 0;

	int width = 0, height = 0, channels = 0;
	float* pixels = stbi_loadf(path, &width, &height, &channels, 3);
	if (!pixels)
	{
		b3Warning("pybullet_pbr: cannot read environment '%s', using the default sky", path);
		return 0;
	}

	glGenTextures(1, &m_pbr->m_equirectTexture);
	glBindTexture(GL_TEXTURE_2D, m_pbr->m_equirectTexture);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB16F, width, height, 0, GL_RGB, GL_FLOAT, pixels);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glBindTexture(GL_TEXTURE_2D, 0);
	stbi_image_free(pixels);
	b3Printf("pybullet_pbr: environment '%s' (%dx%d)", path, width, height);
	return m_pbr->m_equirectTexture;
}

/// Rebuilds the IBL textures when Python has asked for a different environment.
/// One-time cost (a fraction of a second), never per frame.
void PBRInstancingRenderer::updateEnvironment()
{
	if (!gConfig.m_environmentDirty)
		return;
	gConfig.m_environmentDirty = 0;

	GLint prevFbo = 0, viewport[4];
	glGetIntegerv(GL_DRAW_FRAMEBUFFER_BINDING, &prevFbo);
	glGetIntegerv(GL_VIEWPORT, viewport);

	GLuint equirect = loadEquirect(gConfig.m_environmentPath);
	b3Vector3 lightDir = sunDirection(m_data->m_lightPos);

	EnvFaceDrawer drawer;
	drawer.d = m_pbr;
	drawer.program = 0;
	drawer.faceLocation = -1;
	drawer.upAxis = m_upAxis;
	drawer.sunDir[0] = lightDir.x;
	drawer.sunDir[1] = lightDir.y;
	drawer.sunDir[2] = lightDir.z;

	glDisable(GL_DEPTH_TEST);
	glDisable(GL_CULL_FACE);
	glDepthMask(GL_FALSE);
	glBindVertexArray(m_pbr->m_fullscreenVao);
	m_pbr->m_env.build(gConfig.m_environmentPath, equirect, drawer);
	glBindVertexArray(0);
	glUseProgram(0);

	glBindFramebuffer(GL_FRAMEBUFFER, prevFbo);
	glViewport(viewport[0], viewport[1], viewport[2], viewport[3]);
	glDepthMask(GL_TRUE);
	glEnable(GL_DEPTH_TEST);
}

/// Uploads last frame's instance transforms and then records this frame's, so the
/// forward pass can produce motion vectors. This is the renderer's own history:
/// Bullet keeps only current transforms and physics state is never involved.
///
/// When the instance count changes (a body was added or removed) the mapping from
/// index to object is not trustworthy, so history is dropped for one frame rather than
/// reprojecting onto the wrong geometry.
void PBRInstancingRenderer::updatePreviousTransforms(int totalNumInstances)
{
	const int floatsPerInstance = 8;  // vec4 position + vec4 quaternion
	int needed = totalNumInstances * floatsPerInstance;

	bool countChanged = (m_pbr->m_prevInstanceCount != totalNumInstances);
	if (countChanged)
	{
		m_pbr->m_prevInstanceData.resize(needed);
		m_pbr->m_historyValid = false;
	}

	// If we have no usable history, seed it with the current transforms: zero motion
	// beats wrong motion.
	if (countChanged || (int)m_pbr->m_prevInstanceData.size() < needed)
	{
		m_pbr->m_prevInstanceData.resize(needed);
		for (int i = 0; i < totalNumInstances; i++)
			for (int c = 0; c < 4; c++)
			{
				m_pbr->m_prevInstanceData[i * 4 + c] = m_data->m_instance_positions_ptr[i * 4 + c];
				m_pbr->m_prevInstanceData[totalNumInstances * 4 + i * 4 + c] =
					m_data->m_instance_quaternion_ptr[i * 4 + c];
			}
	}

	glBindBuffer(GL_ARRAY_BUFFER, m_pbr->m_prevInstanceVbo);
	glBufferData(GL_ARRAY_BUFFER, needed * sizeof(float),
				 needed ? &m_pbr->m_prevInstanceData[0] : 0, GL_DYNAMIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	m_pbr->m_prevInstanceCount = totalNumInstances;
}

/// Copies this frame's transforms into the history, after they have been drawn.
void PBRInstancingRenderer::recordCurrentTransforms(int totalNumInstances)
{
	int needed = totalNumInstances * 8;
	m_pbr->m_prevInstanceData.resize(needed);
	for (int i = 0; i < totalNumInstances * 4; i++)
	{
		m_pbr->m_prevInstanceData[i] = m_data->m_instance_positions_ptr[i];
		m_pbr->m_prevInstanceData[totalNumInstances * 4 + i] = m_data->m_instance_quaternion_ptr[i];
	}
}

/// Halton(2,3), the usual low-discrepancy sequence for sub-pixel jitter.
static float halton(int index, int base)
{
	float result = 0.f, f = 1.f / (float)base;
	int i = index;
	while (i > 0)
	{
		result += f * (float)(i % base);
		i /= base;
		f /= (float)base;
	}
	return result;
}

static void pbrCreatePerspective(float fovyRadians, float aspect, float zNear, float zFar,
								 float result[16])
{
	float f = 1.f / tanf(fovyRadians * 0.5f);
	for (int i = 0; i < 16; i++) result[i] = 0.f;
	result[0] = f / aspect;
	result[5] = f;
	result[10] = (zFar + zNear) / (zNear - zFar);
	result[11] = -1.f;
	result[14] = (2.f * zFar * zNear) / (zNear - zFar);
}

#define PBR_LIGHT_ATLAS_SIZE 4096
#define PBR_LIGHT_TILE_SIZE 1024
#define PBR_LIGHT_TILES_PER_ROW (PBR_LIGHT_ATLAS_SIZE / PBR_LIGHT_TILE_SIZE)

/// Renders one tile of the light shadow atlas.
void PBRInstancingRenderer::renderLightShadowTile(const float viewProjection[16],
												  const float lightPosition[3], float range,
												  int tile)
{
	int x = (tile % PBR_LIGHT_TILES_PER_ROW) * PBR_LIGHT_TILE_SIZE;
	int y = (tile / PBR_LIGHT_TILES_PER_ROW) * PBR_LIGHT_TILE_SIZE;
	glViewport(x, y, PBR_LIGHT_TILE_SIZE, PBR_LIGHT_TILE_SIZE);
	glScissor(x, y, PBR_LIGHT_TILE_SIZE, PBR_LIGHT_TILE_SIZE);
	glEnable(GL_SCISSOR_TEST);
	glClearColor(1.f, 1.f, 1.f, 1.f);   // "nothing here", i.e. maximum distance
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glDisable(GL_SCISSOR_TEST);

	glUniformMatrix4fv(m_pbr->m_uLightShadowMVP, 1, false, viewProjection);
	glUniform3fv(m_pbr->m_uLightShadowPosition, 1, lightPosition);
	glUniform1f(m_pbr->m_uLightShadowRange, range);
	glViewport(x, y, PBR_LIGHT_TILE_SIZE, PBR_LIGHT_TILE_SIZE);
	drawInstances();
}

/// Fills the atlas for every shadow-casting point and spot light. A spot takes one
/// tile, a point takes six (one per cube face).
void PBRInstancingRenderer::renderLightShadows()
{
	for (int i = 0; i < PBR_MAX_LIGHTS; i++)
		gConfig.m_lights[i].m_shadowTile = -1;

	int active = 0;
	for (int i = 0; i < gConfig.m_lightCount; i++)
		if (gConfig.m_lights[i].m_type != PBR_LIGHT_OFF && gConfig.m_lights[i].m_castsShadow)
			active++;
	if (!active)
		return;

	if (!m_pbr->m_lightShadowAtlas.resize(PBR_LIGHT_ATLAS_SIZE, PBR_LIGHT_ATLAS_SIZE,
										  GL_R16F, true))
	{
		b3Warning("pybullet_pbr: no light shadow atlas; lights will not cast shadows");
		return;
	}

	GLint prevFbo = 0, viewport[4];
	glGetIntegerv(GL_DRAW_FRAMEBUFFER_BINDING, &prevFbo);
	glGetIntegerv(GL_VIEWPORT, viewport);
	glBindFramebuffer(GL_FRAMEBUFFER, m_pbr->m_lightShadowAtlas.fbo());
	glEnable(GL_DEPTH_TEST);
	glDepthMask(GL_TRUE);
	glEnable(GL_CULL_FACE);
	glCullFace(GL_BACK);
	glUseProgram(m_pbr->m_lightShadowProgram);

	// The six cube directions, in the order the fragment shader's cubeFace() expects.
	static const float faceDir[6][3] = {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0},
										{0, -1, 0}, {0, 0, 1}, {0, 0, -1}};
	static const float faceUp[6][3] = {{0, -1, 0}, {0, -1, 0}, {0, 0, 1},
									   {0, 0, -1}, {0, -1, 0}, {0, -1, 0}};

	int nextTile = 0;
	for (int i = 0; i < gConfig.m_lightCount; i++)
	{
		PbrLight& light = gConfig.m_lights[i];
		if (light.m_type == PBR_LIGHT_OFF || !light.m_castsShadow)
			continue;

		int needed = (light.m_type == PBR_LIGHT_POINT) ? 6 : 1;
		if (nextTile + needed > PBR_MAX_SHADOW_TILES)
		{
			b3Warning("pybullet_pbr: out of shadow atlas tiles, light %d casts none", i);
			continue;
		}

		b3Vector3 position = b3MakeVector3(light.m_position[0], light.m_position[1],
										   light.m_position[2]);
		float projection[16], view[16], viewProjection[16];
		float nearPlane = 0.05f;

		if (light.m_type == PBR_LIGHT_POINT)
		{
			pbrCreatePerspective(1.5707963f, 1.f, nearPlane, light.m_range, projection);
			for (int f = 0; f < 6; f++)
			{
				b3Vector3 target = position + b3MakeVector3(faceDir[f][0], faceDir[f][1],
														   faceDir[f][2]);
				b3Vector3 up = b3MakeVector3(faceUp[f][0], faceUp[f][1], faceUp[f][2]);
				pbrCreateLookAt(position, target, up, view);
				pbrMatrixMul16(projection, view, viewProjection);
				renderLightShadowTile(viewProjection, light.m_position, light.m_range,
									  nextTile + f);
			}
		}
		else
		{
			// m_outerDegrees is the full cone angle, so the frustum's field of view is
			// that angle, with a little margin so the cone edge is covered.
			float fovy = light.m_outerDegrees * 0.0174532925f * 1.05f;
			if (fovy > 3.0f) fovy = 3.0f;
			pbrCreatePerspective(fovy, 1.f, nearPlane, light.m_range, projection);

			b3Vector3 direction = b3MakeVector3(light.m_direction[0], light.m_direction[1],
												light.m_direction[2]);
			if (direction.length2() < 1e-8f)
				direction = b3MakeVector3(0, 0, -1);
			direction.normalize();
			b3Vector3 up, side;
			b3PlaneSpace1(direction, up, side);
			pbrCreateLookAt(position, position + direction, up, view);
			pbrMatrixMul16(projection, view, viewProjection);
			renderLightShadowTile(viewProjection, light.m_position, light.m_range, nextTile);

			for (int k = 0; k < 16; k++)
				m_pbr->m_lightMatrices[i * 16 + k] = viewProjection[k];
		}

		light.m_shadowTile = nextTile;
		nextTile += needed;
	}

	glUseProgram(0);
	glBindFramebuffer(GL_FRAMEBUFFER, prevFbo);
	glViewport(viewport[0], viewport[1], viewport[2], viewport[3]);
}

/// Packs the light array into the forward pass's uniforms.
void PBRInstancingRenderer::uploadLightUniforms()
{
	float positionRange[PBR_MAX_LIGHTS * 4] = {0};
	float colorType[PBR_MAX_LIGHTS * 4] = {0};
	float dirCone[PBR_MAX_LIGHTS * 4] = {0};
	float coneTile[PBR_MAX_LIGHTS * 4] = {0};

	int count = gConfig.m_lightCount;
	if (count > PBR_MAX_LIGHTS)
		count = PBR_MAX_LIGHTS;

	for (int i = 0; i < count; i++)
	{
		const PbrLight& light = gConfig.m_lights[i];
		for (int c = 0; c < 3; c++)
		{
			positionRange[i * 4 + c] = light.m_position[c];
			colorType[i * 4 + c] = light.m_color[c] * light.m_intensity;
			dirCone[i * 4 + c] = light.m_direction[c];
		}
		positionRange[i * 4 + 3] = light.m_range;
		colorType[i * 4 + 3] = (float)light.m_type;
		// Cone angles are full angles; shading compares against the angle from the beam
		// axis, which is half of that.
		float outer = light.m_outerDegrees * 0.0087266462f;
		float inner = light.m_innerDegrees * 0.0087266462f;
		if (inner > outer)
			inner = outer;
		dirCone[i * 4 + 3] = cosf(outer);
		coneTile[i * 4 + 0] = cosf(inner);
		coneTile[i * 4 + 1] = (float)light.m_shadowTile;
		coneTile[i * 4 + 2] = gConfig.m_lightShadowBias;
		coneTile[i * 4 + 3] = light.m_radius;
	}

	glUniform1i(m_pbr->m_uLightCount, count);
	glUniform4fv(m_pbr->m_uLightPositionRange, PBR_MAX_LIGHTS, positionRange);
	glUniform4fv(m_pbr->m_uLightColorType, PBR_MAX_LIGHTS, colorType);
	glUniform4fv(m_pbr->m_uLightDirCone, PBR_MAX_LIGHTS, dirCone);
	glUniform4fv(m_pbr->m_uLightConeTile, PBR_MAX_LIGHTS, coneTile);
	glUniformMatrix4fv(m_pbr->m_uLightMatrix, PBR_MAX_LIGHTS, false, m_pbr->m_lightMatrices);

	glActiveTexture(GL_TEXTURE6);
	glBindTexture(GL_TEXTURE_2D, m_pbr->m_lightShadowAtlas.colorTexture());
	glUniform1i(m_pbr->m_uLightShadowAtlas, 6);
	glUniform2f(m_pbr->m_uAtlasTileInfo, (float)PBR_LIGHT_TILES_PER_ROW,
				1.f / (float)PBR_LIGHT_TILES_PER_ROW);
	glActiveTexture(GL_TEXTURE0);
}

/// Renders the shadow map ourselves instead of delegating to the base class.
///
/// Bullet's depth pass culls *front* faces, so a solid object's recorded occluder
/// depth is its far side; the ground right under a sphere then tests as lit and the
/// object looks like it is floating. We render front faces with normal back-face
/// culling and lean on the slope-scaled bias instead, which puts the contact shadow
/// where the contact is. Resolution and world size still come from PyBullet
/// (configureDebugVisualizer(shadowMapResolution=..., shadowMapWorldSize=...)).
void PBRInstancingRenderer::renderShadowMap()
{
	m_pbr->m_shadowValid = false;
	int size = m_data->m_shadowMapWidth;
	if (size <= 0)
		return;
	if (!m_pbr->m_shadow.resize(size, m_data->m_shadowMapHeight, 0, true))
	{
		b3Warning("pybullet_pbr: could not create a %dx%d shadow map", size, size);
		return;
	}

	float center[3] = {0, 0, 0};
	m_data->m_activeCamera->getCameraTargetPosition(center);
	b3Vector3 target = b3MakeVector3(center[0], center[1], center[2]);
	b3Vector3 lightDir = sunDirection(m_data->m_lightPos);
	b3Vector3 up, lightFwd;
	b3PlaneSpace1(lightDir, up, lightFwd);

	float worldSize = m_data->m_shadowMapWorldSize;
	// Snap the frustum centre to whole shadow-map texels in the light's plane. A follow
	// camera moves the target continuously; sub-texel slides re-rasterise every static
	// shadow edge each frame (shimmer). Snapped, the texel grid stays fixed in the world.
	float texel = 2.f * worldSize / (float)size;
	if (texel > 0.f)
	{
		float u = target.dot(up), v = target.dot(lightFwd);
		target += up * (floorf(u / texel + 0.5f) * texel - u) +
				  lightFwd * (floorf(v / texel + 0.5f) * texel - v);
	}
	float lightProjection[16], lightView[16], lightViewProjection[16];
	pbrCreateOrtho(-worldSize, worldSize, -worldSize, worldSize,
				   PBR_SHADOW_NEAR, PBR_SHADOW_FAR, lightProjection);
	pbrCreateLookAt(lightDir * 60.f + target, target, up, lightView);
	pbrMatrixMul16(lightProjection, lightView, lightViewProjection);

	// clip space [-1,1] -> shadow map [0,1]
	float bias[16] = {0.5f, 0, 0, 0, 0, 0.5f, 0, 0, 0, 0, 0.5f, 0, 0.5f, 0.5f, 0.5f, 1.f};
	pbrMatrixMul16(bias, lightViewProjection, m_pbr->m_shadowMatrix);

	GLint prevFbo = 0;
	GLint viewport[4];
	glGetIntegerv(GL_DRAW_FRAMEBUFFER_BINDING, &prevFbo);
	glGetIntegerv(GL_VIEWPORT, viewport);

	glBindFramebuffer(GL_FRAMEBUFFER, m_pbr->m_shadow.fbo());
	glViewport(0, 0, m_pbr->m_shadow.width(), m_pbr->m_shadow.height());
	glClear(GL_DEPTH_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST);
	glDepthMask(GL_TRUE);
	glEnable(GL_CULL_FACE);
	glCullFace(GL_BACK);

	glUseProgram(m_pbr->m_shadowProgram);
	glUniformMatrix4fv(m_pbr->m_uShadowPassMVP, 1, false, lightViewProjection);
	drawInstances();
	glUseProgram(0);

	glBindFramebuffer(GL_FRAMEBUFFER, prevFbo);
	glViewport(viewport[0], viewport[1], viewport[2], viewport[3]);
	m_pbr->m_shadowValid = true;
}

/// Two fullscreen passes between the scene and the tone mapper: occlusion from the
/// depth buffer, then a depth-aware blur to remove the sampling noise.
void PBRInstancingRenderer::renderSsao(const float projectionMatrix[16], const float viewMatrix[16])
{
	int w = m_pbr->m_hdr.width(), h = m_pbr->m_hdr.height();
	if (!m_pbr->m_ao.resize(w, h, GL_R8, false) || !m_pbr->m_aoBlur.resize(w, h, GL_R8, false))
	{
		gConfig.m_ssaoEnabled = 0;
		b3Warning("pybullet_pbr: SSAO buffers unavailable, disabling SSAO");
		return;
	}

	glDisable(GL_DEPTH_TEST);
	glDepthMask(GL_FALSE);
	glDisable(GL_CULL_FACE);
	glBindVertexArray(m_pbr->m_fullscreenVao);

	// occlusion
	glBindFramebuffer(GL_FRAMEBUFFER, m_pbr->m_ao.fbo());
	glViewport(0, 0, w, h);
	glUseProgram(m_pbr->m_ssaoProgram);
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, m_pbr->m_hdr.depthTexture());
	glUniform1i(m_pbr->m_uSsaoDepth, 0);
	glActiveTexture(GL_TEXTURE1);
	glBindTexture(GL_TEXTURE_2D, m_pbr->m_noiseTexture);
	glUniform1i(m_pbr->m_uSsaoNoise, 1);
	glUniform2f(m_pbr->m_uSsaoScreenSize, (float)w, (float)h);
	glUniform2f(m_pbr->m_uSsaoProjScale, projectionMatrix[0], projectionMatrix[5]);
	glUniform2f(m_pbr->m_uSsaoDepthParams, projectionMatrix[10], projectionMatrix[14]);
	glUniformMatrix4fv(m_pbr->m_uSsaoProjMatrix, 1, false, projectionMatrix);
	glUniform1f(m_pbr->m_uSsaoRadius, gConfig.m_ssaoRadius);
	glUniform1f(m_pbr->m_uSsaoBias, gConfig.m_ssaoBias);
	glUniform1i(m_pbr->m_uSsaoKernelSize, PBR_SSAO_KERNEL_SIZE);
	glUniform3fv(m_pbr->m_uSsaoKernel, PBR_SSAO_KERNEL_SIZE, m_pbr->m_kernel);
	glDrawArrays(GL_TRIANGLES, 0, 3);

	// depth-aware blur
	glBindFramebuffer(GL_FRAMEBUFFER, m_pbr->m_aoBlur.fbo());
	glUseProgram(m_pbr->m_ssaoBlurProgram);
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, m_pbr->m_ao.colorTexture());
	glUniform1i(m_pbr->m_uBlurAo, 0);
	glActiveTexture(GL_TEXTURE1);
	glBindTexture(GL_TEXTURE_2D, m_pbr->m_hdr.depthTexture());
	glUniform1i(m_pbr->m_uBlurDepth, 1);
	glUniform2f(m_pbr->m_uBlurTexelSize, 1.f / (float)w, 1.f / (float)h);
	glUniform2f(m_pbr->m_uBlurDepthParams, projectionMatrix[10], projectionMatrix[14]);
	glUniform1f(m_pbr->m_uBlurDepthThreshold, 0.05f);  // world units
	glDrawArrays(GL_TRIANGLES, 0, 3);

	glBindVertexArray(0);
	glActiveTexture(GL_TEXTURE0);
}

/// Bright-pass at quarter resolution, then a separable Gaussian, ping-ponging between
/// two targets. Cheap because it all happens at a quarter of the frame's size.
bool PBRInstancingRenderer::renderBloom(const int viewport[4])
{
	int width = viewport[2] / 4;
	int height = viewport[3] / 4;
	if (width < 4 || height < 4)
		return false;
	if (!m_pbr->m_bloomA.resize(width, height, GL_RGBA16F, false) ||
		!m_pbr->m_bloomB.resize(width, height, GL_RGBA16F, false))
		return false;

	glDisable(GL_DEPTH_TEST);
	glDepthMask(GL_FALSE);
	glDisable(GL_CULL_FACE);
	glBindVertexArray(m_pbr->m_fullscreenVao);
	glViewport(0, 0, width, height);

	glBindFramebuffer(GL_FRAMEBUFFER, m_pbr->m_bloomA.fbo());
	glUseProgram(m_pbr->m_bloomPrefilterProgram);
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, m_pbr->m_hdr.colorTexture());
	glUniform1i(m_pbr->m_uBloomPreSource, 0);
	glUniform2f(m_pbr->m_uBloomPreTexel, 1.f / (float)viewport[2], 1.f / (float)viewport[3]);
	glUniform1f(m_pbr->m_uBloomPreThreshold, gConfig.m_bloomThreshold);
	glUniform1f(m_pbr->m_uBloomPreKnee, gConfig.m_bloomThreshold * 0.5f + 1e-3f);
	glDrawArrays(GL_TRIANGLES, 0, 3);

	glUseProgram(m_pbr->m_bloomBlurProgram);
	for (int pass = 0; pass < 2; pass++)   // horizontal, then vertical
	{
		RenderTarget& from = pass == 0 ? m_pbr->m_bloomA : m_pbr->m_bloomB;
		RenderTarget& to = pass == 0 ? m_pbr->m_bloomB : m_pbr->m_bloomA;
		glBindFramebuffer(GL_FRAMEBUFFER, to.fbo());
		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_2D, from.colorTexture());
		glUniform1i(m_pbr->m_uBloomBlurSource, 0);
		glUniform2f(m_pbr->m_uBloomBlurDirection,
					pass == 0 ? 1.f / (float)width : 0.f,
					pass == 0 ? 0.f : 1.f / (float)height);
		glDrawArrays(GL_TRIANGLES, 0, 3);
	}

	glBindVertexArray(0);
	glDepthMask(GL_TRUE);
	glEnable(GL_DEPTH_TEST);
	return true;
}

/// Graphics instance ids are reused after a reset, so anything keyed by one has to go
/// with it - otherwise a new body inherits the material of whatever held its id before.
void PBRInstancingRenderer::removeAllInstances()
{
	gInstanceKeys.clear();
	gPendingMaterials.clear();
	gShapeTextures.clear();
	m_pbr->m_materialByUid.clear();
	m_pbr->m_materialDirty = true;
	m_pbr->m_historyValid = false;
	m_pbr->m_prevInstanceCount = -1;
	GLInstancingRenderer::removeAllInstances();
}

void PBRInstancingRenderer::renderSceneInternal(int orgRenderMode)
{
	// The colour pass and the shadow pass are ours; every other render mode
	// (segmentation mask, projective texture, reflections) stays with the base class.
	bool ours = (orgRenderMode == B3_DEFAULT_RENDERMODE ||
				 orgRenderMode == B3_USE_SHADOWMAP_RENDERMODE ||
				 orgRenderMode == B3_CREATE_SHADOWMAP_RENDERMODE);
	if (!ours || !loadShaders())
	{
		GLInstancingRenderer::renderSceneInternal(orgRenderMode);
		return;
	}

	init();

	updateEnvironment();

	// With COV_ENABLE_SHADOWS off there is no CREATE_SHADOWMAP pass, but movable
	// lights still cast shadows, so their atlas is filled here instead.
	if (orgRenderMode == B3_DEFAULT_RENDERMODE)
		renderLightShadows();

	if (orgRenderMode == B3_CREATE_SHADOWMAP_RENDERMODE)
	{
		renderShadowMap();
		renderLightShadows();
		return;
	}

	GLint viewport[4];
	glGetIntegerv(GL_VIEWPORT, viewport);
	GLint prevFbo = 0;
	glGetIntegerv(GL_DRAW_FRAMEBUFFER_BINDING, &prevFbo);

	// Motion vectors are needed by temporal anti-aliasing and by motion blur alike.
	bool needVelocity = gConfig.m_antialiasing == 2 || gConfig.m_motionBlur > 0.f;
	if (!m_pbr->m_hdr.resize(viewport[2], viewport[3], GL_RGBA16F, true,
							 needVelocity ? GL_RG16F : 0))
	{
		b3Warning("pybullet_pbr: HDR framebuffer unavailable, using the stock renderer");
		m_pbr->m_shaderLoadFailed = true;
		GLInstancingRenderer::renderSceneInternal(orgRenderMode);
		return;
	}

	float projectionMatrix[16], viewMatrix[16], mvp[16], viewProjection[16];
	m_data->m_activeCamera->getCameraProjectionMatrix(projectionMatrix);
	m_data->m_activeCamera->getCameraViewMatrix(viewMatrix);
	pbrMatrixMul16(projectionMatrix, viewMatrix, viewProjection);

	bool taa = gConfig.m_antialiasing == 2;
	// The previous-frame transforms have to be maintained whenever velocity is used:
	// skipping that upkeep leaves stale data behind, which shows up as a still object
	// smearing as though it were moving.
	int totalNumInstances = totalInstanceCount();
	if (!taa)
		m_pbr->m_frameIndex = (m_pbr->m_frameIndex + 1) & 1023;
	if (needVelocity)
		updatePreviousTransforms(totalNumInstances);
	else
		m_pbr->m_prevInstanceCount = -1;   // stop drawInstances binding a stale buffer
	if (taa)
	{
		// Sub-pixel jitter: each frame samples a slightly different point inside the
		// pixel, and the temporal accumulation of those samples is the anti-aliasing.
		// Only rasterisation is jittered - motion vectors use the clean matrices.
		m_pbr->m_frameIndex = (m_pbr->m_frameIndex + 1) & 1023;
		float jx = (halton(m_pbr->m_frameIndex + 1, 2) - 0.5f) * 2.f / (float)viewport[2];
		float jy = (halton(m_pbr->m_frameIndex + 1, 3) - 0.5f) * 2.f / (float)viewport[3];
		float jittered[16];
		for (int i = 0; i < 16; i++) jittered[i] = projectionMatrix[i];
		jittered[8] += jx;
		jittered[9] += jy;
		pbrMatrixMul16(jittered, viewMatrix, mvp);
	}
	else
	{
		for (int i = 0; i < 16; i++) mvp[i] = viewProjection[i];
		m_pbr->m_historyValid = false;
	}
	float camPos[3] = {0, 0, 0};
	m_data->m_activeCamera->getCameraPosition(camPos);
	b3Vector3 lightDir = sunDirection(m_data->m_lightPos);

	// --- scene pass, into the HDR target -----------------------------------
	glBindFramebuffer(GL_FRAMEBUFFER, m_pbr->m_hdr.fbo());
	glViewport(0, 0, m_pbr->m_hdr.width(), m_pbr->m_hdr.height());

	GLfloat bg[4];
	glGetFloatv(GL_COLOR_CLEAR_VALUE, bg);  // the app's background colour, in sRGB
	glClearColor(srgbToLinear(bg[0]), srgbToLinear(bg[1]), srgbToLinear(bg[2]), 1.f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glClearColor(bg[0], bg[1], bg[2], bg[3]);  // leave the state as we found it

	bool ibl = gConfig.m_iblEnabled != 0 && m_pbr->m_env.ready();

	// The sky the scene is lit by should be the sky you see behind it.
	if (ibl && gConfig.m_showBackground)
	{
		glDisable(GL_DEPTH_TEST);
		glDepthMask(GL_FALSE);
		glDisable(GL_CULL_FACE);
		glUseProgram(m_pbr->m_backgroundProgram);
		// Camera basis straight out of the view matrix: its rotation part is the
		// world->view rotation, so its transpose gives the axes in world space. Taken
		// from the accessors instead, the up vector does not always come back as the
		// camera's own up and the sky ends up with no vertical gradient at all.
		float right[3] = {viewMatrix[0], viewMatrix[4], viewMatrix[8]};
		float up[3] = {viewMatrix[1], viewMatrix[5], viewMatrix[9]};
		float forward[3] = {-viewMatrix[2], -viewMatrix[6], -viewMatrix[10]};
		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_CUBE_MAP, m_pbr->m_env.cubemap());
		glUniform1i(m_pbr->m_uBgMap, 0);
		glUniform3fv(m_pbr->m_uBgRight, 1, right);
		glUniform3fv(m_pbr->m_uBgUp, 1, up);
		glUniform3fv(m_pbr->m_uBgForward, 1, forward);
		glUniform2f(m_pbr->m_uBgProjScale, projectionMatrix[0], projectionMatrix[5]);
		glUniform1f(m_pbr->m_uBgStrength, gConfig.m_iblStrength);
		glUniform1i(m_pbr->m_uBgAirless, gConfig.m_skyAirless);
		glUniform1i(m_pbr->m_uBgUpAxis, m_upAxis);
		glBindVertexArray(m_pbr->m_fullscreenVao);
		glDrawArrays(GL_TRIANGLES, 0, 3);
		glBindVertexArray(0);
		glDepthMask(GL_TRUE);
	}

	glEnable(GL_DEPTH_TEST);
	glDepthMask(GL_TRUE);
	glEnable(GL_CULL_FACE);
	glCullFace(GL_BACK);

	glUseProgram(m_pbr->m_forwardProgram);
	glUniformMatrix4fv(m_pbr->m_uMVP, 1, false, mvp);
	glUniformMatrix4fv(m_pbr->m_uViewProjection, 1, false, viewProjection);
	glUniformMatrix4fv(m_pbr->m_uPrevViewProjection, 1, false,
					   m_pbr->m_prevCameraValid ? m_pbr->m_prevViewProjection : viewProjection);
	glUniform3f(m_pbr->m_uLightDirection, lightDir.x, lightDir.y, lightDir.z);
	glUniform3f(m_pbr->m_uCameraPosition, camPos[0], camPos[1], camPos[2]);
	glUniform3f(m_pbr->m_uSunColor, gConfig.m_sunColor[0] * gConfig.m_sunIntensity,
				gConfig.m_sunColor[1] * gConfig.m_sunIntensity,
				gConfig.m_sunColor[2] * gConfig.m_sunIntensity);
	glUniform1f(m_pbr->m_uAmbientIntensity, gConfig.m_ambientIntensity);
	glUniform1i(m_pbr->m_uDiagnostic, gConfig.m_diagnostic == 1 ? 1 : 0);
	glUniform1i(m_pbr->m_uDiffuse, 0);
	uploadMaterialTable();
	uploadLightUniforms();
	glUniform2f(m_pbr->m_uDefaultMaterial, gConfig.m_defaultMetallic, gConfig.m_defaultRoughness);
	glUniform1i(m_pbr->m_uNormalMap, 7);
	glUniform1i(m_pbr->m_uRoughnessMap, 8);
	glUniform1i(m_pbr->m_uMetallicMap, 9);
	glUniform1i(m_pbr->m_uAoMap, 10);
	glUniform1f(m_pbr->m_uNormalScale, gConfig.m_normalScale);
	glUniform1i(m_pbr->m_uIblEnabled, ibl ? 1 : 0);
	glUniform1f(m_pbr->m_uIblStrength, gConfig.m_iblStrength);
	glUniform1f(m_pbr->m_uPrefilterMips, (float)(Environment::prefilterMips() - 1));

	// Assigned even when IBL is off: sampler uniforms default to unit 0, and a program
	// with a sampler2D and a samplerCube on the same unit is invalid to draw with, so
	// every draw call would fail and the scene would render empty.
	glUniform1i(m_pbr->m_uIrradianceMap, 2);
	glUniform1i(m_pbr->m_uPrefilterMap, 3);
	glUniform1i(m_pbr->m_uBrdfLut, 4);
	if (ibl)
	{
		glActiveTexture(GL_TEXTURE2);
		glBindTexture(GL_TEXTURE_CUBE_MAP, m_pbr->m_env.irradiance());
		glActiveTexture(GL_TEXTURE3);
		glBindTexture(GL_TEXTURE_CUBE_MAP, m_pbr->m_env.prefiltered());
		glActiveTexture(GL_TEXTURE4);
		glBindTexture(GL_TEXTURE_2D, m_pbr->m_env.brdfLut());
		glActiveTexture(GL_TEXTURE0);
	}

	// The base class has just filled its shadow map (renderScene() runs the
	// B3_CREATE_SHADOWMAP_RENDERMODE pass first); rebuild the same light matrices
	// and sample it. B3_DEFAULT_RENDERMODE means shadows are off.
	bool shadows = (orgRenderMode == B3_USE_SHADOWMAP_RENDERMODE) && m_pbr->m_shadowValid;
	glUniform1i(m_pbr->m_uShadowEnabled, shadows ? 1 : 0);
	if (shadows)
	{
		glActiveTexture(GL_TEXTURE1);
		glBindTexture(GL_TEXTURE_2D, m_pbr->m_shadow.depthTexture());
		glUniform1i(m_pbr->m_uShadowMap, 1);
		glUniformMatrix4fv(m_pbr->m_uShadowMatrix, 1, false, m_pbr->m_shadowMatrix);
		// Bullet's own shadowMapIntensity defaults to 0.3 because its renderer had no
		// ambient term to fall back on; ours does, so shadows default to removing all
		// direct sun and pybullet_pbr.set_shadows(strength=) is the control.
		glUniform1f(m_pbr->m_uShadowIntensity, gConfig.m_shadowStrength);
		glUniform2f(m_pbr->m_uShadowTexelSize,
					1.f / (float)m_data->m_shadowMapWidth,
					1.f / (float)m_data->m_shadowMapHeight);
		// The light's ortho depth range is linear, so a world-unit bias converts by a
		// constant. Biasing in raw depth units would be ~300x too strong and erase
		// every shadow.
		float depthPerWorldUnit = 1.f / (PBR_SHADOW_FAR - PBR_SHADOW_NEAR);
		glUniform1f(m_pbr->m_uShadowBias, gConfig.m_shadowBias * depthPerWorldUnit);
		glUniform1f(m_pbr->m_uShadowSlopeBias, gConfig.m_shadowSlopeBias * depthPerWorldUnit);
		glUniform1f(m_pbr->m_uShadowSoftness, gConfig.m_shadowSoftness);

		// Penumbra width = gap * tan(angular radius). Depth is normalised over the
		// light frustum's span and the filter radius is in texels, so convert once:
		// (span in metres) * tan(radius) / (metres per texel).
		float angularRadius = gConfig.m_sunAngularDiameter * 0.5f * 0.0174532925f;
		float metresPerTexel = 2.f * m_data->m_shadowMapWorldSize /
							   (float)(m_data->m_shadowMapWidth > 0 ? m_data->m_shadowMapWidth : 1);
		float penumbraScale = (PBR_SHADOW_FAR - PBR_SHADOW_NEAR) * tanf(angularRadius) /
							  metresPerTexel;
		glUniform1f(m_pbr->m_uSunPenumbraScale,
					gConfig.m_sunAngularDiameter > 0.f ? penumbraScale : 0.f);
		glActiveTexture(GL_TEXTURE0);
	}

	drawInstances();

	if (needVelocity)
	{
		recordCurrentTransforms(totalNumInstances);
		for (int i = 0; i < 16; i++) m_pbr->m_prevViewProjection[i] = viewProjection[i];
		m_pbr->m_prevCameraValid = true;
	}
	else
	{
		m_pbr->m_prevCameraValid = false;
	}

	bool ssao = gConfig.m_ssaoEnabled != 0 && gConfig.m_ssaoStrength > 0.f;
	if (ssao)
		renderSsao(projectionMatrix, viewMatrix);

	// Auto-exposure meters the frame's average off the smallest mip. Built here rather
	// than in the tone-mapping block: generating mips binds the texture on the active
	// unit, which would displace what the tone mapper is about to read from it.
	if (gConfig.m_autoExposure)
		m_pbr->m_hdr.generateColorMips();

	bool bloom = gConfig.m_bloom > 0.f && renderBloom(viewport);

	// --- resolve: depth back for the overlay, then tone map ------------------
	glBindFramebuffer(GL_READ_FRAMEBUFFER, m_pbr->m_hdr.fbo());
	glBindFramebuffer(GL_DRAW_FRAMEBUFFER, prevFbo);
	glBlitFramebuffer(0, 0, m_pbr->m_hdr.width(), m_pbr->m_hdr.height(),
					  viewport[0], viewport[1], viewport[0] + viewport[2], viewport[1] + viewport[3],
					  GL_DEPTH_BUFFER_BIT, GL_NEAREST);

	// With AA on the tone mapper writes to an LDR buffer that FXAA then resolves onto
	// the real target; without it, straight to the target and one pass less.
	bool fxaa = gConfig.m_antialiasing == 1 &&
				m_pbr->m_ldr.resize(viewport[2], viewport[3], GL_RGBA8, false);
	// A viewport size change reallocates the history, so whatever it held is gone.
	// This happens for real when getCameraImage() is interleaved with GUI frames: the
	// two render at different sizes, and TAA then has nothing to accumulate.
	int historyWidth = m_pbr->m_taaHistory.width();
	int historyHeight = m_pbr->m_taaHistory.height();
	bool taaResolve = taa &&
					  m_pbr->m_ldr.resize(viewport[2], viewport[3], GL_RGBA8, false) &&
					  m_pbr->m_taaHistory.resize(viewport[2], viewport[3], GL_RGBA8, false) &&
					  m_pbr->m_taaResolve.resize(viewport[2], viewport[3], GL_RGBA8, false);
	if (historyWidth != viewport[2] || historyHeight != viewport[3])
		m_pbr->m_historyValid = false;
	if (fxaa || taaResolve)
	{
		glBindFramebuffer(GL_FRAMEBUFFER, m_pbr->m_ldr.fbo());
		glViewport(0, 0, m_pbr->m_ldr.width(), m_pbr->m_ldr.height());
	}
	else
	{
		glBindFramebuffer(GL_FRAMEBUFFER, prevFbo);
		glViewport(viewport[0], viewport[1], viewport[2], viewport[3]);
	}
	glDisable(GL_DEPTH_TEST);
	glDepthMask(GL_FALSE);
	glDisable(GL_CULL_FACE);

	glUseProgram(m_pbr->m_tonemapProgram);
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, m_pbr->m_hdr.colorTexture());
	glUniform1i(m_pbr->m_uHdrBuffer, 0);
	glUniform1f(m_pbr->m_uExposure, gConfig.m_exposure);
	glUniform1i(m_pbr->m_uTonemapper, gConfig.m_tonemapper);
	glUniform1i(m_pbr->m_uSsaoEnabled, ssao ? 1 : 0);
	glUniform1f(m_pbr->m_uSsaoStrength, gConfig.m_ssaoStrength);
	glUniform1i(m_pbr->m_uAoDebug, gConfig.m_diagnostic == 2 ? 1 : 0);
	glUniform1i(m_pbr->m_uAutoExposure, gConfig.m_autoExposure);
	glUniform1f(m_pbr->m_uExposureKey, gConfig.m_exposureKey);
	glUniform2f(m_pbr->m_uExposureLimits, gConfig.m_exposureMinEv, gConfig.m_exposureMaxEv);
	glUniform1f(m_pbr->m_uBloomStrength, bloom ? gConfig.m_bloom : 0.f);
	if (bloom)
	{
		glActiveTexture(GL_TEXTURE12);
		glBindTexture(GL_TEXTURE_2D, m_pbr->m_bloomA.colorTexture());
		glUniform1i(m_pbr->m_uBloomTexture, 12);
		glActiveTexture(GL_TEXTURE0);
	}
	glUniform1f(m_pbr->m_uVignette, gConfig.m_vignette);
	glUniform1f(m_pbr->m_uNoise, gConfig.m_noise);
	glUniform1f(m_pbr->m_uMotionBlur, needVelocity ? gConfig.m_motionBlur : 0.f);
	glUniform1f(m_pbr->m_uFrameSeed, (float)(m_pbr->m_frameIndex & 255) / 255.f);
	glUniform2f(m_pbr->m_uScreenSize, (float)viewport[2], (float)viewport[3]);
	glUniform3fv(m_pbr->m_uWhiteBalance, 1, gConfig.m_whiteBalance);
	if (needVelocity)
	{
		glActiveTexture(GL_TEXTURE11);
		glBindTexture(GL_TEXTURE_2D, m_pbr->m_hdr.secondColorTexture());
		glUniform1i(m_pbr->m_uVelocityTexture, 11);
		glActiveTexture(GL_TEXTURE0);
	}
	if (ssao)
	{
		glActiveTexture(GL_TEXTURE1);
		glBindTexture(GL_TEXTURE_2D, m_pbr->m_aoBlur.colorTexture());
		glUniform1i(m_pbr->m_uAoTexture, 1);
		glActiveTexture(GL_TEXTURE0);
	}
	glBindVertexArray(m_pbr->m_fullscreenVao);
	glDrawArrays(GL_TRIANGLES, 0, 3);

	if (taaResolve)
	{
		// Reproject the history onto this frame, clamped to the local colour range,
		// into a buffer we keep as the next frame's history.
		glBindFramebuffer(GL_FRAMEBUFFER, m_pbr->m_taaResolve.fbo());
		glViewport(0, 0, m_pbr->m_taaResolve.width(), m_pbr->m_taaResolve.height());
		glUseProgram(m_pbr->m_taaProgram);
		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_2D, m_pbr->m_ldr.colorTexture());
		glUniform1i(m_pbr->m_uTaaCurrent, 0);
		glActiveTexture(GL_TEXTURE1);
		glBindTexture(GL_TEXTURE_2D, m_pbr->m_taaHistory.colorTexture());
		glUniform1i(m_pbr->m_uTaaHistory, 1);
		glActiveTexture(GL_TEXTURE2);
		glBindTexture(GL_TEXTURE_2D, m_pbr->m_hdr.secondColorTexture());
		glUniform1i(m_pbr->m_uTaaVelocity, 2);
		glActiveTexture(GL_TEXTURE0);
		glUniform2f(m_pbr->m_uTaaTexelSize, 1.f / (float)viewport[2], 1.f / (float)viewport[3]);
		glUniform1f(m_pbr->m_uTaaBlend, gConfig.m_taaBlend);
		glUniform1i(m_pbr->m_uTaaHistoryValid, m_pbr->m_historyValid ? 1 : 0);
		glDrawArrays(GL_TRIANGLES, 0, 3);

		// Show the resolved frame, and keep it as history.
		glBindFramebuffer(GL_READ_FRAMEBUFFER, m_pbr->m_taaResolve.fbo());
		glBindFramebuffer(GL_DRAW_FRAMEBUFFER, prevFbo);
		glBlitFramebuffer(0, 0, viewport[2], viewport[3],
						  viewport[0], viewport[1], viewport[0] + viewport[2],
						  viewport[1] + viewport[3], GL_COLOR_BUFFER_BIT, GL_NEAREST);
		glBindFramebuffer(GL_DRAW_FRAMEBUFFER, m_pbr->m_taaHistory.fbo());
		glBlitFramebuffer(0, 0, viewport[2], viewport[3], 0, 0, viewport[2], viewport[3],
						  GL_COLOR_BUFFER_BIT, GL_NEAREST);
		glBindFramebuffer(GL_FRAMEBUFFER, prevFbo);
		glViewport(viewport[0], viewport[1], viewport[2], viewport[3]);
		m_pbr->m_historyValid = true;
	}
	else if (fxaa)
	{
		glBindFramebuffer(GL_FRAMEBUFFER, prevFbo);
		glViewport(viewport[0], viewport[1], viewport[2], viewport[3]);
		glUseProgram(m_pbr->m_fxaaProgram);
		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_2D, m_pbr->m_ldr.colorTexture());
		glUniform1i(m_pbr->m_uFxaaScreen, 0);
		glUniform2f(m_pbr->m_uFxaaTexelSize, 1.f / (float)m_pbr->m_ldr.width(),
					1.f / (float)m_pbr->m_ldr.height());
		glUniform1f(m_pbr->m_uFxaaSpanMax, 8.f);
		glUniform1f(m_pbr->m_uFxaaReduceMul, 1.f / 8.f);
		glUniform1f(m_pbr->m_uFxaaReduceMin, 1.f / 128.f);
		glDrawArrays(GL_TRIANGLES, 0, 3);
	}
	glBindVertexArray(0);

	glDepthMask(GL_TRUE);
	glEnable(GL_DEPTH_TEST);
	glUseProgram(0);
}

/// Factory picked up by the weak hook in SimpleOpenGL3App.cpp.
GLInstancingRenderer* b3CreateInstancingRenderer(int maxObjects, int maxShapeBytes)
{
	if (getenv("PYBULLET_PBR_DISABLE"))
		return 0;
	return new PBRInstancingRenderer(maxObjects, maxShapeBytes);
}
