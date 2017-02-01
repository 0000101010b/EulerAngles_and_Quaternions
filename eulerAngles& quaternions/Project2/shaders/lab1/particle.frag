#version 330 core
out vec4 color;

in vec4 ParticleColor;
in vec3 FragPos;
in vec3 Normal;


uniform vec3 lightPos;

void main()
{

	   // Diffuse 
    vec3 norm = normalize(Normal);
    vec3 lightDir = normalize(lightPos - FragPos);
	
    float diff = max(dot(lightDir,norm), 0.0);
    vec3 diffuse = diff * ParticleColor.xyz;
	
	color =vec4(diffuse,ParticleColor.a)*1.5f;
}  