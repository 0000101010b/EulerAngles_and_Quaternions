#version 330 core
layout (location = 0) in vec3 position;
layout(location = 1) in vec3 normal;
layout(location = 2) in vec2 texCoords;

out vec4 ParticleColor;
out vec3 Normal;
out vec3 FragPos;

uniform mat4 projection;
uniform mat4 view;
uniform mat4 model;

uniform vec3 offset;
uniform vec4 color;
uniform float scale;


void main()
{
	Normal=normal;
	FragPos=vec3(model * vec4(position, 1.0f));
    ParticleColor = color;
    gl_Position = projection * view * model * vec4(position*scale +offset, 1.0f); 
}