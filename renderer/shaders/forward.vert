#version 330 core
// Scene pass. Same instanced vertex layout as Bullet's stock renderer.
layout (location = 0) in vec4 position;
layout (location = 1) in vec4 instance_position;
layout (location = 2) in vec4 instance_quaternion;
layout (location = 3) in vec2 uvcoords;
layout (location = 4) in vec3 vertexnormal;
layout (location = 5) in vec4 instance_color;
layout (location = 6) in vec4 instance_scale_obUid;
// Kept renderer-side (see PBRInstancingRenderer::updatePreviousTransforms), not by Bullet.
layout (location = 7) in vec4 prev_instance_position;
layout (location = 8) in vec4 prev_instance_quaternion;
// Per-instance material, also renderer-side: metallic, roughness, marker, spare.
layout (location = 9) in vec4 instance_material;

uniform mat4 MVP;            // jittered, for rasterisation
uniform mat4 viewProjection;  // unjittered, for motion vectors
uniform mat4 prevViewProjection;
uniform mat4 shadowMatrix;
uniform vec2 defaultMaterial;      // metallic, roughness

out vec3 worldNormal;
out vec3 worldPos;
out vec2 texcoord;
out vec4 instanceColor;
out vec4 shadowCoord;
out vec4 clipCurrent;
out vec4 clipPrevious;
flat out vec3 material;   // metallic, roughness, emissive luminance

vec4 quatMul(in vec4 q1, in vec4 q2)
{
	vec3 im = q1.w * q2.xyz + q1.xyz * q2.w + cross(q1.xyz, q2.xyz);
	vec4 dt = q1 * q2;
	float re = dot(dt, vec4(-1.0, -1.0, -1.0, 1.0));
	return vec4(im, re);
}

vec4 quatRotate3(in vec3 p, in vec4 q)
{
	vec4 temp = quatMul(q, vec4(p, 0.0));
	return quatMul(temp, vec4(-q.x, -q.y, -q.z, q.w));
}

void main(void)
{
	// Flags say which of metallic/roughness and emissive were actually set; anything
	// not set falls back to the configured default.
	int materialFlags = int(instance_material.z + 0.5);
	material.xy = (materialFlags & 1) != 0 ? clamp(instance_material.xy, 0.0, 1.0)
										  : defaultMaterial;
	material.z = (materialFlags & 2) != 0 ? max(instance_material.w, 0.0) : 0.0;

	vec4 q = instance_quaternion;
	// Normals transform by the inverse scale, so a non-uniformly scaled shape shades
	// correctly instead of skewing its lighting.
	vec3 scale = max(abs(instance_scale_obUid.xyz), vec3(1e-6));
	worldNormal = quatRotate3(vertexnormal / scale, q).xyz;
	instanceColor = instance_color;
	texcoord = uvcoords;
	vec4 localcoord = quatRotate3(position.xyz * instance_scale_obUid.xyz, q);
	worldPos = (instance_position + localcoord).xyz;
	shadowCoord = shadowMatrix * vec4(worldPos, 1);
	gl_Position = MVP * vec4(worldPos, 1);

	vec3 prevLocal = quatRotate3(position.xyz * instance_scale_obUid.xyz,
								 prev_instance_quaternion).xyz;
	clipCurrent = viewProjection * vec4(worldPos, 1);
	clipPrevious = prevViewProjection * vec4((prev_instance_position.xyz + prevLocal), 1);
}
