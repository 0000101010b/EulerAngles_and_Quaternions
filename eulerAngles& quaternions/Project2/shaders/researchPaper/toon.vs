//Phong Reflection Model Vertex Shader
#version 330
 
layout(location = 0) in vec3 position;
layout(location = 1) in vec3 normal;
layout(location = 1) in vec2 texCoords;
 
uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;
 
//send them to fragment shader
out vec3 Pos;
out vec3 Normal;
out vec2 TexCoords;

 
void main()
{
	Pos = mat3(model) * position;
	Normal = normalize(mat3(model) * normal);
	TexCoords=texCoords;
	gl_Position = projection*view*model*vec4(position,1);
}