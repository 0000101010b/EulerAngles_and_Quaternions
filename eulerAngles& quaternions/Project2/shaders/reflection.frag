#version 330 core
in vec3 Normal;
in vec3 Position;
out vec4 color;
in vec2 TexCoords;

uniform vec3 cameraPos;
uniform sampler2D texture_diffuse1;
uniform samplerCube skybox;

void main()
{       
	vec4 diffuse_color = texture(texture_diffuse1, TexCoords);
    vec3 I = normalize(Position - cameraPos);
    vec3 R = reflect(I, normalize(Normal));
    vec4 reflect_color;
    reflect_color = texture(skybox, R);
	color = diffuse_color + reflect_color;
}