#version 330 core
// Screen-space ambient occlusion from the depth buffer alone: view-space position
// comes from depth, the normal from its screen-space derivatives. One less render
// target than a normal buffer, and plenty for an occlusion term.
in vec2 uv;

uniform sampler2D depthTexture;
uniform sampler2D noiseTexture;
uniform vec2 screenSize;
uniform vec2 projScale;      // projection[0][0], projection[1][1]
uniform vec2 depthParams;    // projection[2][2], projection[3][2]
uniform mat4 projMatrix;
uniform float radius;
uniform float bias;
uniform int kernelSize;
uniform vec3 kernel[32];

out float fragColor;

// Inverse of a symmetric perspective projection. PyBullet's GUI camera is always
// symmetric; an off-centre projection would need the full inverse matrix.
vec3 viewPosition(vec2 texcoord, float depth)
{
	float ndcZ = depth * 2.0 - 1.0;
	// ndcZ = (P22*viewZ + P32) / -viewZ  =>  viewZ = -P32 / (ndcZ + P22); negative in front.
	float viewZ = -depthParams.y / (ndcZ + depthParams.x);
	vec2 ndc = texcoord * 2.0 - 1.0;
	return vec3(ndc.x * -viewZ / projScale.x, ndc.y * -viewZ / projScale.y, viewZ);
}

void main(void)
{
	float depth = texture(depthTexture, uv).r;
	if (depth >= 1.0)
	{
		fragColor = 1.0;   // background: nothing to occlude
		return;
	}

	vec3 P = viewPosition(uv, depth);
	vec3 N = normalize(cross(dFdx(P), dFdy(P)));
	if (N.z < 0.0)
		N = -N;

	vec3 randomVec = texture(noiseTexture, uv * screenSize / 4.0).xyz * 2.0 - 1.0;
	vec3 T = normalize(randomVec - N * dot(randomVec, N));
	mat3 TBN = mat3(T, cross(N, T), N);

	float occlusion = 0.0;
	for (int i = 0; i < kernelSize; i++)
	{
		vec3 samplePos = P + TBN * kernel[i] * radius;

		vec4 clip = projMatrix * vec4(samplePos, 1.0);
		vec2 sampleUV = (clip.xy / clip.w) * 0.5 + 0.5;
		if (sampleUV.x < 0.0 || sampleUV.x > 1.0 || sampleUV.y < 0.0 || sampleUV.y > 1.0)
			continue;

		float sampleDepth = texture(depthTexture, sampleUV).r;
		if (sampleDepth >= 1.0)
			continue;
		float sceneZ = viewPosition(sampleUV, sampleDepth).z;

		// Only count occluders in front of the sample, and fade out distant ones so a
		// foreground object cannot smear a halo over the background.
		float rangeCheck = smoothstep(0.0, 1.0, radius / abs(P.z - sceneZ));
		if (sceneZ >= samplePos.z + bias)
			occlusion += rangeCheck;
	}
	fragColor = 1.0 - occlusion / float(kernelSize);
}
