#version 330 core
// One axis of a separable Gaussian. Run twice, horizontally then vertically, which
// costs 2N taps instead of N*N and gives a round halo.
in vec2 uv;

uniform sampler2D source;
uniform vec2 direction;   // texel step along the axis being blurred

out vec3 fragColor;

void main(void)
{
	const float weights[5] = float[](0.227027, 0.194595, 0.121622, 0.054054, 0.016216);
	vec3 result = texture(source, uv).rgb * weights[0];
	for (int i = 1; i < 5; i++)
	{
		vec2 offset = direction * float(i);
		result += texture(source, uv + offset).rgb * weights[i];
		result += texture(source, uv - offset).rgb * weights[i];
	}
	fragColor = result;
}
