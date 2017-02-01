#version 330 core

in vec2 TexCoords;
in vec3 world_normal;
in vec3 world_pos;

out vec4 color;

uniform sampler2D texture_diffuse1;

uniform vec3 viewPos;



void main()
{
	vec3 V = normalize(viewPos - world_pos);
	float edgeDetection = (dot(V, world_normal) > 0.3) ? 1 : 0;
	
	float greyscale=dot(texture(texture_diffuse1, TexCoords).xyz, vec3(0.299, 0.587, 0.114));
	
	color.xyz =  ((vec3(greyscale)*10.2 )*edgeDetection) ;
	color.a=0.0;
	
}