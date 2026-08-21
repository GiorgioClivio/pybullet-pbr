#version 330 core
// Depth-aware 4x4 blur: averages the AO noise but refuses to bleed across an edge,
// which is what stops the halos SSAO is notorious for. The depth comparison has to
// happen in linear view depth - raw depth-buffer differences are meaningless at
// distance, and thresholding them lets the silhouette smear onto the background.
in vec2 uv;

uniform sampler2D aoTexture;
uniform sampler2D depthTexture;
uniform vec2 texelSize;
uniform vec2 depthParams;      // projection[2][2], projection[3][2]
uniform float depthThreshold;  // world units

out float fragColor;

float viewDepth(float depth)
{
	float ndcZ = depth * 2.0 - 1.0;
	return -depthParams.y / (ndcZ + depthParams.x);
}

void main(void)
{
	float centerRaw = texture(depthTexture, uv).r;
	if (centerRaw >= 1.0)
	{
		fragColor = 1.0;   // background: never occluded, never blurred into
		return;
	}
	float centerZ = viewDepth(centerRaw);

	float sum = 0.0;
	float weight = 0.0;
	for (int y = -2; y < 2; y++)
		for (int x = -2; x < 2; x++)
		{
			vec2 offset = vec2(x, y) * texelSize;
			float raw = texture(depthTexture, uv + offset).r;
			if (raw >= 1.0)
				continue;
			float w = abs(viewDepth(raw) - centerZ) < depthThreshold ? 1.0 : 0.0;
			sum += texture(aoTexture, uv + offset).r * w;
			weight += w;
		}
	fragColor = weight > 0.0 ? sum / weight : texture(aoTexture, uv).r;
}
