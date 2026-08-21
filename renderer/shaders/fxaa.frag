#version 330 core
// FXAA 3 (Lottes' compact form), run on the tone-mapped sRGB image where luma
// differences mean what the eye sees. No temporal state - that arrives with TAA.
in vec2 uv;

uniform sampler2D screenTexture;
uniform vec2 texelSize;
uniform float spanMax;     // longest blur, in pixels
uniform float reduceMul;   // how much local brightness reduces the blur
uniform float reduceMin;

out vec4 fragColor;

float luma(vec3 c)
{
	return dot(c, vec3(0.299, 0.587, 0.114));
}

void main(void)
{
	vec3 rgbNW = texture(screenTexture, uv + vec2(-1.0, -1.0) * texelSize).rgb;
	vec3 rgbNE = texture(screenTexture, uv + vec2(1.0, -1.0) * texelSize).rgb;
	vec3 rgbSW = texture(screenTexture, uv + vec2(-1.0, 1.0) * texelSize).rgb;
	vec3 rgbSE = texture(screenTexture, uv + vec2(1.0, 1.0) * texelSize).rgb;
	vec3 rgbM = texture(screenTexture, uv).rgb;

	float lumaNW = luma(rgbNW), lumaNE = luma(rgbNE);
	float lumaSW = luma(rgbSW), lumaSE = luma(rgbSE);
	float lumaM = luma(rgbM);
	float lumaMin = min(lumaM, min(min(lumaNW, lumaNE), min(lumaSW, lumaSE)));
	float lumaMax = max(lumaM, max(max(lumaNW, lumaNE), max(lumaSW, lumaSE)));

	// Edge direction from the luma gradient across the 2x2 neighbourhood.
	vec2 dir = vec2(-((lumaNW + lumaNE) - (lumaSW + lumaSE)),
					((lumaNW + lumaSW) - (lumaNE + lumaSE)));

	float dirReduce = max((lumaNW + lumaNE + lumaSW + lumaSE) * 0.25 * reduceMul, reduceMin);
	float rcpDirMin = 1.0 / (min(abs(dir.x), abs(dir.y)) + dirReduce);
	dir = clamp(dir * rcpDirMin, vec2(-spanMax), vec2(spanMax)) * texelSize;

	vec3 rgbA = 0.5 * (texture(screenTexture, uv + dir * (1.0 / 3.0 - 0.5)).rgb +
					   texture(screenTexture, uv + dir * (2.0 / 3.0 - 0.5)).rgb);
	vec3 rgbB = rgbA * 0.5 + 0.25 * (texture(screenTexture, uv + dir * -0.5).rgb +
									 texture(screenTexture, uv + dir * 0.5).rgb);

	// The wider tap set is better unless it wandered off the edge entirely.
	float lumaB = luma(rgbB);
	fragColor = vec4((lumaB < lumaMin || lumaB > lumaMax) ? rgbA : rgbB, 1.0);
}
