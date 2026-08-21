#version 330 core
// Isolates what is bright enough to bloom, at quarter resolution.
//
// The threshold has to be applied *before* blurring: blur first and a bright source
// is averaged with its dark surroundings, so it stops passing the threshold and the
// glow ends up shaped like the averaging window rather than like the source.
in vec2 uv;

uniform sampler2D hdrBuffer;
uniform vec2 texelSize;    // of the source
uniform float threshold;   // cd/m2
uniform float knee;        // width of the soft shoulder around the threshold

out vec3 fragColor;

void main(void)
{
	// 4-tap box downsample, which is what makes this quarter-res step cheap
	vec3 c = texture(hdrBuffer, uv + vec2(-1.0, -1.0) * texelSize).rgb;
	c += texture(hdrBuffer, uv + vec2(1.0, -1.0) * texelSize).rgb;
	c += texture(hdrBuffer, uv + vec2(-1.0, 1.0) * texelSize).rgb;
	c += texture(hdrBuffer, uv + vec2(1.0, 1.0) * texelSize).rgb;
	c *= 0.25;

	float luminance = dot(c, vec3(0.2126, 0.7152, 0.0722));
	float soft = clamp((luminance - threshold + knee) / max(2.0 * knee, 1e-4), 0.0, 1.0);
	float contribution = max(luminance - threshold, 0.0);
	contribution = max(contribution, soft * soft * knee);
	fragColor = c * (contribution / max(luminance, 1e-5));
}
