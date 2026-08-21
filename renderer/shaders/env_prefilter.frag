#version 330 core
// GGX-importance-sampled environment, one mip per roughness level: the specular half
// of image-based lighting.
in vec2 uv;

uniform int face;
uniform samplerCube environmentMap;
uniform float roughness;
uniform float resolution;   // of the source cubemap face, for mip selection

out vec4 fragColor;

const float PI = 3.14159265359;
const uint SAMPLE_COUNT = 128u;

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

float radicalInverse(uint bits)
{
	bits = (bits << 16u) | (bits >> 16u);
	bits = ((bits & 0x55555555u) << 1u) | ((bits & 0xAAAAAAAAu) >> 1u);
	bits = ((bits & 0x33333333u) << 2u) | ((bits & 0xCCCCCCCCu) >> 2u);
	bits = ((bits & 0x0F0F0F0Fu) << 4u) | ((bits & 0xF0F0F0F0u) >> 4u);
	bits = ((bits & 0x00FF00FFu) << 8u) | ((bits & 0xFF00FF00u) >> 8u);
	return float(bits) * 2.3283064365386963e-10;
}

vec3 importanceSampleGGX(vec2 Xi, vec3 N, float a)
{
	float phi = 2.0 * PI * Xi.x;
	float cosTheta = sqrt((1.0 - Xi.y) / (1.0 + (a * a - 1.0) * Xi.y));
	float sinTheta = sqrt(1.0 - cosTheta * cosTheta);

	vec3 H = vec3(cos(phi) * sinTheta, sin(phi) * sinTheta, cosTheta);
	vec3 up = abs(N.z) < 0.999 ? vec3(0.0, 0.0, 1.0) : vec3(1.0, 0.0, 0.0);
	vec3 tangent = normalize(cross(up, N));
	vec3 bitangent = cross(N, tangent);
	return normalize(tangent * H.x + bitangent * H.y + N * H.z);
}

void main(void)
{
	vec3 N = normalize(faceDirection(face, uv));
	vec3 R = N, V = N;   // the usual isotropic approximation
	float a = roughness * roughness;

	vec3 colour = vec3(0.0);
	float totalWeight = 0.0;
	for (uint i = 0u; i < SAMPLE_COUNT; i++)
	{
		vec2 Xi = vec2(float(i) / float(SAMPLE_COUNT), radicalInverse(i));
		vec3 H = importanceSampleGGX(Xi, N, a);
		vec3 L = normalize(2.0 * dot(V, H) * H - V);

		float NdotL = max(dot(N, L), 0.0);
		if (NdotL <= 0.0)
			continue;

		// Sample a mip matched to the solid angle of this sample, or sparse kernels
		// turn a bright sun into fireflies.
		float NdotH = max(dot(N, H), 0.0);
		float D = (a * a) / max(PI * pow(NdotH * NdotH * (a * a - 1.0) + 1.0, 2.0), 1e-7);
		float pdf = D * NdotH / (4.0 * max(dot(H, V), 1e-4)) + 1e-4;
		float saTexel = 4.0 * PI / (6.0 * resolution * resolution);
		float saSample = 1.0 / (float(SAMPLE_COUNT) * pdf);
		float mip = roughness == 0.0 ? 0.0 : 0.5 * log2(saSample / saTexel);

		colour += textureLod(environmentMap, L, mip).rgb * NdotL;
		totalWeight += NdotL;
	}
	fragColor = vec4(colour / max(totalWeight, 1e-4), 1.0);
}
