#version 330 core
// Temporal anti-aliasing on the tone-mapped image.
//
// The history is reprojected with per-pixel motion vectors, then clamped to the
// colour range of the current 3x3 neighbourhood. That clamp is what stops ghost
// trails: anything the history holds that the current frame cannot justify gets
// pulled back into range.
in vec2 uv;

uniform sampler2D currentTexture;
uniform sampler2D historyTexture;
uniform sampler2D velocityTexture;
uniform vec2 texelSize;
uniform float blend;        // weight of the history, 0 = no temporal accumulation
uniform int historyValid;

out vec4 fragColor;

void main(void)
{
	vec3 current = texture(currentTexture, uv).rgb;
	if (historyValid == 0)
	{
		fragColor = vec4(current, 1.0);
		return;
	}

	vec2 velocity = texture(velocityTexture, uv).rg;
	vec2 prevUv = uv - velocity;
	if (prevUv.x < 0.0 || prevUv.x > 1.0 || prevUv.y < 0.0 || prevUv.y > 1.0)
	{
		fragColor = vec4(current, 1.0);   // came from off-screen: nothing to reuse
		return;
	}

	vec3 minColour = current;
	vec3 maxColour = current;
	for (int y = -1; y <= 1; y++)
		for (int x = -1; x <= 1; x++)
		{
			vec3 c = texture(currentTexture, uv + vec2(x, y) * texelSize).rgb;
			minColour = min(minColour, c);
			maxColour = max(maxColour, c);
		}

	vec3 history = clamp(texture(historyTexture, prevUv).rgb, minColour, maxColour);

	// Fast movement gets less history: reprojection is least trustworthy exactly
	// where the motion vectors are longest.
	float speed = length(velocity / texelSize);
	float weight = blend * clamp(1.0 - speed * 0.02, 0.0, 1.0);
	fragColor = vec4(mix(current, history, weight), 1.0);
}
