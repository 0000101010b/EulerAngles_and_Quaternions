#version 330 core
layout (location = 0) in vec3 position;
layout (location = 1) in vec3 normal;
layout (location = 2) in vec2 texCoords;

out VS_OUT {
    vec2 texCoords;
	vec3 Normal;
	vec3 Pos;
} vs_out;

uniform mat4 projection;
uniform mat4 view;
uniform mat4 model;

void main()
{
	vs_out.Normal=normalize(mat3(model) * normal);
	vs_out.Pos= mat3(model) * position;
    gl_Position = projection * view * model * vec4(position, 1.0f); 
	vs_out.texCoords = texCoords;
}
