#version 330 core
in vec3 worldPos;

uniform vec3 lightPosition;
uniform float lightRange;

out float fragColor;

void main(void)
{
	// Distance to the light, normalised. Linear, so the comparison bias in the
	// forward pass is a plain distance in metres.
	fragColor = clamp(length(worldPos - lightPosition) / lightRange, 0.0, 1.0);
}
