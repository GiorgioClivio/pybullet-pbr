#version 330 core
// Fullscreen triangle; no vertex buffer needed.
out vec2 uv;
void main(void)
{
	vec2 p = vec2((gl_VertexID << 1) & 2, gl_VertexID & 2);
	uv = p;
	gl_Position = vec4(p * 2.0 - 1.0, 0.0, 1.0);
}
