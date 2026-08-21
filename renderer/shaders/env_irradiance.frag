#version 330 core
// Cosine-weighted convolution of the environment: the diffuse half of image-based
// lighting. Cheap because the target is only a few pixels per face.
in vec2 uv;

uniform int face;
uniform samplerCube environmentMap;

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

void main(void)
{
	vec3 N = normalize(faceDirection(face, uv));
	vec3 up = abs(N.z) < 0.999 ? vec3(0.0, 0.0, 1.0) : vec3(1.0, 0.0, 0.0);
	vec3 right = normalize(cross(up, N));
	up = cross(N, right);

	vec3 irradiance = vec3(0.0);
	float samples = 0.0;
	const float step = 0.025;
	for (float phi = 0.0; phi < 2.0 * PI; phi += step * 4.0)
		for (float theta = 0.0; theta < 0.5 * PI; theta += step)
		{
			vec3 tangentSample = vec3(sin(theta) * cos(phi), sin(theta) * sin(phi), cos(theta));
			vec3 sampleVec = tangentSample.x * right + tangentSample.y * up + tangentSample.z * N;
			irradiance += texture(environmentMap, sampleVec).rgb * cos(theta) * sin(theta);
			samples += 1.0;
		}
	fragColor = vec4(PI * irradiance / samples, 1.0);
}
