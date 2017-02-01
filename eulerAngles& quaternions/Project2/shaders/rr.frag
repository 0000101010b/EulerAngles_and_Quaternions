#version 330 core
in vec3 Normal;
in vec3 Position;

in vec3 Reflect;

in vec3 RefractR;
in vec3 RefractB;
in vec3 RefractG;
in vec2 TexCoords;

in float Ratio; 

out vec4 FragColor;

uniform vec3 cameraPos;
uniform samplerCube skybox;
uniform sampler2D texture_diffuse1;

void main()
{             

    //float ratio = 1.00 / 1.52;
  //  vec3 I = normalize(Position - cameraPos);
//	vec3 R = refract(I, normalize(Normal), ratio);

	vec4 diffuse_color = texture(texture_diffuse1, TexCoords);	
	
	vec3 refractColor;
	refractColor.r = vec3(texture(skybox, RefractR)).r;
	refractColor.g =vec3(texture(skybox, RefractG)).g;
	refractColor.b =vec3(texture(skybox, RefractB)).b;
	
	vec3 reflectColor = vec3(texture(skybox, Reflect));
	vec3 color = mix(refractColor, reflectColor, Ratio);
	FragColor = vec4(color, 1.0)+diffuse_color;
	
	
	
	
  //  vec4 temp = texture(skybox, R);
   // I = normalize(Position - cameraPos);
    //R = reflect(I, normalize(Normal));
    //FragColorcolor = texture(skybox, R)+temp;
}