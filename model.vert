#version 330
layout(location = 0) in vec3 pos;
layout(location = 1) in vec3 norm;
smooth out vec3 vertex_normal;
uniform mat4 projectionMatrix;
uniform mat4 modelToCameraMatrix;
uniform mat3 normalRotMatrix;
void main()
{
	vec4 full_pos = vec4(pos, 1.0);
	gl_Position = projectionMatrix * (modelToCameraMatrix * full_pos);
	vertex_normal = normalRotMatrix*norm;
}
