#version 330 core
layout (location = 0) in vec3 position;
layout (location = 1) in vec3 normal;
layout (location = 2) in vec2 texCoords;


out vec3 Normal;
out vec3 Position;
out vec3 Reflect;
out vec3 Refract;
out vec3 RefractR,RefractG,RefractB;
out vec2 TexCoords;

out float Ratio;


uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

uniform bool shake;
uniform float time;

uniform float EtaR;
uniform float EtaG;
uniform float EtaB;
uniform float FresnelPower;

float rand(vec2 n)
{
  return 0.5 + 0.5 * 
     fract(sin(dot(n.xy, vec2(12.9898, 78.233)))* 43758.5453);
}



//const float Eta = 0.66; // Ratio of indices of refraction



void main()
{
	float F=((1.0- EtaG)*(1.0- EtaG))/((1.0+EtaG)*(1.0+EtaG));

    vec3 FragPos = vec3(time*model* view * vec4(position,1.0f));
	vec2 seed=FragPos.xy;
	vec2 seed2=FragPos.zy;
	vec2 seed3=FragPos.xz;
	
	
	float r=rand(seed);
	float r2=rand(seed2);
	float r3=rand(seed3);
	
	vec3 randPos;
	if(shake)
	{
		randPos=vec3(r,r2,r3);
	}else{
		randPos=vec3(0,0,0);
	}

	
	vec4 ecPosition= view*vec4(position+randPos, 1.0f);
	vec3 ecPosition3=ecPosition.xyz/ecPosition.w;

	vec3 i=normalize(ecPosition3);
	vec3 n=normalize(model * view * vec4(normal,1.0f)).xyz;

	Ratio = F + (1.0 - F) * pow((1.0 - dot(-i, n)), FresnelPower);
	
	RefractG = refract(i, n, EtaG);
	RefractB = refract(i, n,EtaG);
	RefractR = refract(i, n,EtaR);
	
	Reflect = reflect(i, n);
	
    gl_Position = projection * view * model * vec4(position+randPos, 1.0f);
    

	//Normal = mat3(transpose(inverse(model))) * normal;
    Position = vec3(model * vec4(position+randPos, 1.0f));
	TexCoords=texCoords;
}  