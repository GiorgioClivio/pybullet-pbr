#version 330 core
// Forward scene pass: metallic/roughness Cook-Torrance, linear space throughout.
// The HDR result is tone mapped in a separate pass.

in vec3 worldNormal;
in vec3 worldPos;
in vec2 texcoord;
in vec4 instanceColor;
in vec4 shadowCoord;
in vec4 clipCurrent;
in vec4 clipPrevious;

uniform sampler2D Diffuse;
uniform sampler2D normalMap;
uniform sampler2D roughnessMap;
uniform sampler2D metallicMap;
uniform sampler2D aoMap;
uniform int materialMaps;      // bit per slot: 1 normal, 2 roughness, 4 metallic, 8 AO
uniform float normalScale;
uniform vec3 lightDirection;      // world space, points towards the light
uniform vec3 cameraPosition;
uniform vec3 sunColor;   // colour premultiplied by intensity
uniform float ambientIntensity;
uniform int diagnosticMode;       // 1 = show world-space normals as RGB
flat in vec3 material;   // metallic, roughness, emissive luminance (cd/m2)

uniform samplerCube irradianceMap;
uniform samplerCube prefilterMap;
uniform sampler2D brdfLut;
uniform int iblEnabled;
uniform float iblStrength;
uniform float prefilterMips;

// --- movable point / spot lights -----------------------------------------
#define PBR_MAX_LIGHTS 8
uniform int lightCount;
uniform vec4 lightPositionRange[PBR_MAX_LIGHTS];   // xyz position, w range
uniform vec4 lightColorType[PBR_MAX_LIGHTS];       // rgb colour*intensity, w type
uniform vec4 lightDirCone[PBR_MAX_LIGHTS];         // xyz direction, w cos(outer)
uniform vec4 lightConeTile[PBR_MAX_LIGHTS];        // x cos(inner), y tile, z bias, w radius
uniform mat4 lightMatrix[PBR_MAX_LIGHTS];          // spot only
uniform sampler2D lightShadowAtlas;
uniform vec2 atlasTileInfo;                        // tiles per row, tile size in uv

uniform sampler2D shadowMap;
uniform int shadowEnabled;
uniform float shadowIntensity;   // Bullet's setShadowMapIntensity(), 0 = no shadowing
uniform vec2 shadowTexelSize;
uniform float shadowBias;
uniform float shadowSlopeBias;
uniform float shadowSoftness;    // PCF radius in texels
uniform float sunPenumbraScale;  // tan(sun angular radius) / shadow-map world size

layout (location = 0) out vec4 fragColor;
layout (location = 1) out vec2 outVelocity;

const float PI = 3.14159265359;

vec3 srgbToLinear(vec3 c)
{
	return mix(c / 12.92, pow((c + 0.055) / 1.055, vec3(2.4)), step(vec3(0.04045), c));
}

/// Tangent frame from screen-space derivatives, rather than a stored tangent attribute.
///
/// Bullet's vertex format has no tangent, and adding one would mean shadowing its whole
/// mesh registration path. Deriving the frame per pixel needs none of that and copes
/// with missing or degenerate UVs by construction - where the UV gradient vanishes the
/// frame collapses and the geometric normal is used unchanged. The cost is that it is a
/// per-pixel frame rather than a smoothly interpolated per-vertex one, which shows only
/// on very low-poly geometry with strong normal maps.
mat3 cotangentFrame(vec3 N, vec3 position, vec2 uv)
{
	vec3 dp1 = dFdx(position);
	vec3 dp2 = dFdy(position);
	vec2 duv1 = dFdx(uv);
	vec2 duv2 = dFdy(uv);

	vec3 dp2perp = cross(dp2, N);
	vec3 dp1perp = cross(N, dp1);
	vec3 T = dp2perp * duv1.x + dp1perp * duv2.x;
	vec3 B = dp2perp * duv1.y + dp1perp * duv2.y;

	// Without usable UVs both tangents collapse; inversesqrt(0) would give infinities,
	// so fall back to a frame that leaves the geometric normal untouched.
	float lengthSquared = max(dot(T, T), dot(B, B));
	if (lengthSquared < 1e-12)
		return mat3(vec3(0.0), vec3(0.0), N);
	return mat3(T * inversesqrt(lengthSquared), B * inversesqrt(lengthSquared), N);
}

// GGX / Trowbridge-Reitz normal distribution
float distributionGGX(float NdotH, float a)
{
	float a2 = a * a;
	float d = NdotH * NdotH * (a2 - 1.0) + 1.0;
	return a2 / max(PI * d * d, 1e-7);
}

// Smith masking-shadowing with Schlick-GGX, direct-lighting k
float geometrySmith(float NdotV, float NdotL, float a)
{
	float k = (a + 1.0) * (a + 1.0) / 8.0;
	float gv = NdotV / (NdotV * (1.0 - k) + k);
	float gl = NdotL / (NdotL * (1.0 - k) + k);
	return gv * gl;
}

vec3 fresnelSchlick(float cosTheta, vec3 F0)
{
	return F0 + (1.0 - F0) * pow(clamp(1.0 - cosTheta, 0.0, 1.0), 5.0);
}

// Rough surfaces should not get a mirror-bright rim, so the Fresnel used for the
// environment is capped by the roughness.
vec3 fresnelSchlickRoughness(float cosTheta, vec3 F0, float rough)
{
	return F0 + (max(vec3(1.0 - rough), F0) - F0) * pow(clamp(1.0 - cosTheta, 0.0, 1.0), 5.0);
}

/// Sun shadowing with a contact-hardening penumbra.
///
/// The sun is not a point: it subtends about half a degree from Earth and the Moon,
/// a third from Mars. So a shadow edge is sharp where the caster touches the ground
/// and softens with distance from it - which is a real depth cue, and on airless
/// bodies often the only one. A short blocker search estimates how far the occluder
/// is, and the filter widens to match.
float sunVisibility(float NdotL)
{
	if (shadowEnabled == 0 || shadowIntensity <= 0.0)
		return 1.0;

	vec3 coord = shadowCoord.xyz / shadowCoord.w;
	if (coord.x < 0.0 || coord.x > 1.0 || coord.y < 0.0 || coord.y > 1.0 || coord.z > 1.0)
		return 1.0;  // outside the shadow map: unshadowed, no hard border

	// Slope-scaled bias: grazing light needs more of it, or the surface shadows itself.
	float slope = sqrt(max(1.0 - NdotL * NdotL, 0.0)) / max(NdotL, 0.05);
	float bias = shadowBias + shadowSlopeBias * min(slope, 4.0);

	float radius = shadowSoftness;
	if (sunPenumbraScale > 0.0)
	{
		float blockerDepth = 0.0;
		float blockers = 0.0;
		float searchRadius = max(shadowSoftness, 2.0) * 3.0;
		for (int y = -1; y <= 1; y++)
			for (int x = -1; x <= 1; x++)
			{
				float d = texture(shadowMap,
								  coord.xy + vec2(x, y) * shadowTexelSize * searchRadius).r;
				if (d < coord.z - bias)
				{
					blockerDepth += d;
					blockers += 1.0;
				}
			}
		if (blockers < 0.5)
			return 1.0;   // nothing in the way at all
		blockerDepth /= blockers;
		// Depth here is linear in the light's ortho frustum, so the gap between
		// receiver and blocker converts straight to a penumbra width.
		float gap = max(coord.z - blockerDepth, 0.0);
		radius = max(shadowSoftness, gap * sunPenumbraScale);
	}

	float sum = 0.0;
	for (int y = -2; y <= 2; y++)
		for (int x = -2; x <= 2; x++)
		{
			vec2 offset = vec2(x, y) * shadowTexelSize * radius * 0.5;
			float stored = texture(shadowMap, coord.xy + offset).r;
			sum += (coord.z - bias) <= stored ? 1.0 : 0.0;
		}
	float visible = sum / 25.0;
	return mix(1.0, visible, shadowIntensity);
}

vec2 tileUv(float tile, vec2 local)
{
	float perRow = atlasTileInfo.x;
	float size = atlasTileInfo.y;
	vec2 origin = vec2(mod(tile, perRow), floor(tile / perRow)) * size;
	// stay a texel inside the tile so filtering cannot pull in a neighbour
	return origin + clamp(local, 0.002, 0.998) * size;
}

/// Cube-face lookup for a point light: pick the major axis of the light vector and
/// build that face's uv, the same mapping the six shadow passes rasterised with.
void cubeFace(vec3 v, out float face, out vec2 uv)
{
	vec3 a = abs(v);
	if (a.x >= a.y && a.x >= a.z)
	{
		face = v.x > 0.0 ? 0.0 : 1.0;
		uv = vec2(v.x > 0.0 ? -v.z : v.z, -v.y) / a.x;
	}
	else if (a.y >= a.z)
	{
		face = v.y > 0.0 ? 2.0 : 3.0;
		uv = vec2(v.x, v.y > 0.0 ? v.z : -v.z) / a.y;
	}
	else
	{
		face = v.z > 0.0 ? 4.0 : 5.0;
		uv = vec2(v.z > 0.0 ? v.x : -v.x, -v.y) / a.z;
	}
	uv = uv * 0.5 + 0.5;
}

float lightVisibility(int i, vec3 toFragment, float distance, float range)
{
	float tile = lightConeTile[i].y;
	if (tile < 0.0)
		return 1.0;
	float bias = lightConeTile[i].z;

	// An emitter of finite size casts a penumbra. A shadow-map tile spans about 2*d
	// of world at distance d, so a radius of r metres is r/(2d) in tile uv - which
	// also makes the penumbra widen with distance, as it should.
	float penumbra = lightConeTile[i].w / max(2.0 * distance, 1e-3);

	vec2 uv;
	if (lightColorType[i].w > 1.5)   // spot: one tile, projected by the light matrix
	{
		vec4 clip = lightMatrix[i] * vec4(worldPos, 1.0);
		if (clip.w <= 0.0)
			return 1.0;
		vec3 ndc = clip.xyz / clip.w;
		if (any(lessThan(ndc.xy, vec2(-1.0))) || any(greaterThan(ndc.xy, vec2(1.0))))
			return 1.0;
		uv = tileUv(tile, ndc.xy * 0.5 + 0.5);
	}
	else                              // point: six tiles, selected by cube face
	{
		float face;
		vec2 local;
		cubeFace(toFragment, face, local);
		uv = tileUv(tile + face, local);
	}

	if (penumbra <= 0.0001)
	{
		float stored = texture(lightShadowAtlas, uv).r * range;
		return distance - bias <= stored ? 1.0 : 0.0;
	}

	// 3x3 PCF across the penumbra width.
	float sum = 0.0;
	float step = penumbra * atlasTileInfo.y;
	for (int y = -1; y <= 1; y++)
		for (int x = -1; x <= 1; x++)
		{
			float stored = texture(lightShadowAtlas, uv + vec2(x, y) * step).r * range;
			sum += distance - bias <= stored ? 1.0 : 0.0;
		}
	return sum / 9.0;
}

/// Point and spot contribution, with inverse-square falloff windowed to the light's
/// range so it reaches zero smoothly instead of being cut off.
vec3 punctualLights(vec3 N, vec3 V, vec3 albedo, float metallic, float roughness,
					vec3 F0, float alpha)
{
	vec3 total = vec3(0.0);
	for (int i = 0; i < lightCount && i < PBR_MAX_LIGHTS; i++)
	{
		float type = lightColorType[i].w;
		if (type < 0.5)
			continue;

		vec3 lightPos = lightPositionRange[i].xyz;
		float range = lightPositionRange[i].w;
		vec3 toFragment = worldPos - lightPos;
		float distance = length(toFragment);
		if (distance > range || distance < 1e-5)
			continue;

		vec3 L = -toFragment / distance;
		float NdotL = max(dot(N, L), 0.0);
		if (NdotL <= 0.0)
			continue;

		float windowed = clamp(1.0 - pow(distance / range, 4.0), 0.0, 1.0);
		float attenuation = windowed * windowed / (distance * distance + 0.01);

		if (type > 1.5)   // spot cone
		{
			float cosTheta = dot(normalize(lightDirCone[i].xyz), -L);
			float cosOuter = lightDirCone[i].w;
			float cosInner = lightConeTile[i].x;
			attenuation *= smoothstep(cosOuter, cosInner, cosTheta);
			if (attenuation <= 0.0)
				continue;
		}

		float visibility = lightVisibility(i, toFragment, distance, range);
		if (visibility <= 0.0)
			continue;

		vec3 H = normalize(V + L);
		float NdotV = max(dot(N, V), 1e-4);
		float NdotH = max(dot(N, H), 0.0);
		vec3 F = fresnelSchlick(max(dot(H, V), 0.0), F0);
		// A light of finite size spreads its highlight: widen the lobe by the angle
		// the emitter subtends, so a big soft lamp cannot leave a pinpoint glint.
		float sphereAlpha = clamp(alpha + lightConeTile[i].w / max(2.0 * distance, 1e-3),
								  0.0, 1.0);
		float D = distributionGGX(NdotH, sphereAlpha);
		float G = geometrySmith(NdotV, NdotL, sphereAlpha);
		vec3 specular = (D * G * F) / max(4.0 * NdotV * NdotL, 1e-4);
		vec3 kD = (vec3(1.0) - F) * (1.0 - metallic);

		total += (kD * albedo / PI + specular) * lightColorType[i].rgb * NdotL *
				 attenuation * visibility;
	}
	return total;
}

void main(void)
{
	if (instanceColor.w == 0.0)
		discard;

	// Screen-space motion since last frame, in uv units, for temporal reprojection.
	outVelocity = (clipCurrent.xy / clipCurrent.w - clipPrevious.xy / clipPrevious.w) * 0.5;

	vec3 N = normalize(worldNormal);
	if (diagnosticMode == 1)
	{
		fragColor = vec4(N * 0.5 + 0.5, 0.0);
		return;
	}

	float metallic = material.x;
	float roughness = material.y;

	vec3 albedo = srgbToLinear(instanceColor.rgb) * srgbToLinear(texture(Diffuse, texcoord).rgb);

	// Material maps. Roughness, metallic and AO are data, not colour, so they are
	// sampled linearly and multiply the per-instance values rather than replacing them
	// - a map darkens or roughens what the material already says it is.
	float ambientOcclusion = 1.0;
	if (materialMaps != 0)
	{
		if ((materialMaps & 2) != 0)
			roughness *= texture(roughnessMap, texcoord).g;
		if ((materialMaps & 4) != 0)
			metallic *= texture(metallicMap, texcoord).b;
		if ((materialMaps & 8) != 0)
			ambientOcclusion = texture(aoMap, texcoord).r;
		if ((materialMaps & 1) != 0)
		{
			vec3 sampled = texture(normalMap, texcoord).xyz * 2.0 - 1.0;
			sampled.xy *= normalScale;
			vec3 perturbed = cotangentFrame(N, worldPos, texcoord) * sampled;
			if (dot(perturbed, perturbed) > 1e-12)
				N = normalize(perturbed);
		}
	}
	float a = max(roughness, 0.03);
	a = a * a;  // perceptual roughness -> GGX alpha

	vec3 V = normalize(cameraPosition - worldPos);
	vec3 L = normalize(lightDirection);
	vec3 H = normalize(V + L);
	float NdotV = max(dot(N, V), 1e-4);
	float NdotL = max(dot(N, L), 0.0);
	float NdotH = max(dot(N, H), 0.0);

	vec3 F0 = mix(vec3(0.04), albedo, metallic);
	vec3 F = fresnelSchlick(max(dot(H, V), 0.0), F0);
	float D = distributionGGX(NdotH, a);
	float G = geometrySmith(NdotV, NdotL, a);

	vec3 specular = (D * G * F) / max(4.0 * NdotV * NdotL, 1e-4);
	vec3 kD = (vec3(1.0) - F) * (1.0 - metallic);
	vec3 direct = (kD * albedo / PI + specular) * sunColor * NdotL * sunVisibility(NdotL);
	direct += punctualLights(N, V, albedo, metallic, roughness, F0, a);

	vec3 ambient;
	if (iblEnabled == 1)
	{
		vec3 F_env = fresnelSchlickRoughness(NdotV, F0, roughness);
		vec3 kD_env = (vec3(1.0) - F_env) * (1.0 - metallic);
		vec3 irradiance = texture(irradianceMap, N).rgb;
		vec3 diffuseIBL = irradiance * albedo;

		vec3 R = reflect(-V, N);
		vec3 prefiltered = textureLod(prefilterMap, R, roughness * prefilterMips).rgb;
		vec2 ab = texture(brdfLut, vec2(NdotV, roughness)).rg;
		vec3 specularIBL = prefiltered * (F_env * ab.x + ab.y);

		ambient = (kD_env * diffuseIBL + specularIBL) * iblStrength * ambientOcclusion;
	}
	else
	{
		ambient = ambientIntensity * albedo * (1.0 - metallic * 0.5) * ambientOcclusion;
	}

	// Emissive surfaces - LEDs, screens, markers - radiate their own luminance in
	// cd/m2, independent of any light in the scene. It is not ambient, so ambient
	// occlusion must not touch it either.
	vec3 emissive = albedo * material.z;

	// Alpha carries the share of this pixel that came from ambient light, so the
	// composite can apply ambient occlusion without darkening direct sunlight.
	vec3 total = direct + ambient + emissive;
	float totalLuma = dot(total, vec3(0.2126, 0.7152, 0.0722));
	float ambientLuma = dot(ambient, vec3(0.2126, 0.7152, 0.0722));
	fragColor = vec4(total, totalLuma > 0.0 ? ambientLuma / totalLuma : 1.0);
}
