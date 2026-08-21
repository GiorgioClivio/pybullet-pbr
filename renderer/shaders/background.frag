#version 330 core
// Draws the environment behind the scene, so the sky the objects are lit by is the
// sky you actually see. The view ray is built from the camera basis - no matrix
// inverse needed.
in vec2 uv;

uniform samplerCube environmentMap;
uniform vec3 cameraRight;
uniform vec3 cameraUp;
uniform vec3 cameraForward;
uniform vec2 projScale;      // projection[0][0], projection[1][1]
uniform float strength;
uniform int airless;    // on an airless body there is no sky below the horizon
uniform int upAxis;

layout (location = 0) out vec4 fragColor;
layout (location = 1) out vec2 outVelocity;

void main(void)
{
	vec2 ndc = uv * 2.0 - 1.0;
	vec3 dir = normalize(cameraForward + cameraRight * (ndc.x / projScale.x) +
						 cameraUp * (ndc.y / projScale.y));
	vec3 sky = texture(environmentMap, dir).rgb * strength;

	// The lower hemisphere of the environment carries ground bounce, which is light
	// the scene should receive but not something you can see past the horizon. On an
	// airless body that would otherwise show up as a bright band where the terrain
	// ends, so fade it out for the background only.
	if (airless == 1)
	{
		float up = upAxis == 1 ? dir.y : dir.z;
		sky *= smoothstep(-0.02, 0.02, up);
	}

	// alpha 0: the sky is not ambient light, so ambient occlusion must not touch it
	fragColor = vec4(sky, 0.0);
	outVelocity = vec2(0.0);   // the sky does not move relative to the camera's rotation
}
