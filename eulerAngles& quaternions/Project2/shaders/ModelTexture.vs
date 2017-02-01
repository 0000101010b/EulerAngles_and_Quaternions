#version 330 core
layout (location = 0) in vec3 position;
layout (location = 1) in vec3 normal;
layout (location = 2) in vec2 texCoords;

out vec3 Normal;
out vec2 TexCoords;


uniform mat4 projection;
uniform mat4 view;
uniform mat4 model;


void main(void)
{
	mat3 normalMatrix = transpose(inverse(mat3(model)));
    Normal = normalize(normalMatrix * normal);
	TexCoords=texCoords;
	gl_Position = projection * view * model * vec4(position, 1.0f);
}
