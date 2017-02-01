#version 330 core
layout (location = 0) in vec3 position;
layout (location = 1) in vec3 normal;
layout (location = 2) in vec2 texCoords;

out vec2 TexCoords;
out vec3 world_normal;
out vec3 world_pos;

uniform mat4 projection;
uniform mat4 view;
uniform mat4 model;

void main()
{
	TexCoords = texCoords;
	world_normal=normalize(mat3(model) * normal);
	world_pos= mat3(model) * position;
    gl_Position = projection * view * model * vec4(position, 1.0f); 

}