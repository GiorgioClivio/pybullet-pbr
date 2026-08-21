#version 330 core
// Fills one face of the environment cubemap, either from an equirectangular HDR
// image or from a procedural sky. Rendered with a fullscreen triangle per face:
// no cube mesh, no per-face view matrices.
in vec2 uv;

uniform int face;
uniform int hasTexture;
uniform sampler2D equirect;
uniform int upAxis;          // 1 = Y up, 2 = Z up (PyBullet's default)
uniform vec3 sunDirection;   // world space, towards the sun
uniform vec3 zenithColor;
uniform vec3 horizonColor;
uniform vec3 groundColor;
uniform vec3 sunColor;
uniform float sunGlow;
uniform int airless;     // 1 = no atmosphere: no sky light, only ground bounce

out vec4 fragColor;

const float PI = 3.14159265359;

vec3 faceDirection(int f, vec2 texcoord)
{
	vec2 t = texcoord * 2.0 - 1.0;
	if (f == 0) return vec3(1.0, -t.y, -t.x);
	if (f == 1) return vec3(-1.0, -t.y, t.x);
	if (f == 2) return vec3(t.x, 1.0, t.y);
	if (f == 3) return vec3(t.x, -1.0, -t.y);
	if (f == 4) return vec3(t.x, -t.y, 1.0);
	return vec3(-t.x, -t.y, -1.0);
}

float upComponent(vec3 d)
{
	return upAxis == 1 ? d.y : d.z;
}

void main(void)
{
	vec3 dir = normalize(faceDirection(face, uv));

	if (hasTexture == 1)
	{
		// equirectangular lookup, with the world's up axis as the polar axis
		float up = upComponent(dir);
		vec3 flat_ = upAxis == 1 ? vec3(dir.x, 0.0, dir.z) : vec3(dir.x, dir.y, 0.0);
		float azimuth = atan(upAxis == 1 ? flat_.z : flat_.y, flat_.x);
		vec2 texcoord = vec2(azimuth / (2.0 * PI) + 0.5, acos(clamp(up, -1.0, 1.0)) / PI);
		fragColor = vec4(texture(equirect, texcoord).rgb, 1.0);
		return;
	}

	// Procedural sky: a bright band at the horizon fading to a deeper zenith, a dim
	// bounce colour below, and a soft sun disc so glossy materials have something to
	// reflect.
	float up = upComponent(dir);
	vec3 sky;
	if (airless == 1)
	{
		// No atmosphere: the sky is black and the only indirect light is what the
		// ground bounces back. This is what makes lunar and Martian-night scenes read
		// correctly - shadows go nearly black instead of filling with blue.
		sky = up >= 0.0 ? zenithColor : mix(groundColor, groundColor * 0.5, pow(-up, 0.5));
	}
	else if (up >= 0.0)
		sky = mix(horizonColor, zenithColor, pow(up, 0.55));
	else
		sky = mix(horizonColor, groundColor, pow(-up, 0.35));

	// Glow around the sun, but no disc: the directional light already provides the
	// sun's specular highlight, and a pow(cos, 800) disc is badly undersampled at
	// cubemap resolution anyway - it shows up as a blocky white smear.
	float sun = max(dot(dir, normalize(sunDirection)), 0.0);
	sky += sunColor * sunGlow * pow(sun, 7.0);

	fragColor = vec4(sky, 1.0);
}
