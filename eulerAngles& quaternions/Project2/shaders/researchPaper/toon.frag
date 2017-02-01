//Fragment shader contour detection
//Blinn-Phong with same color for RGB
#version 330
layout(location = 0) out vec4 FragColor;
 
uniform vec3 lightPos;
uniform vec3 viewPos;

uniform int material_shininess;
uniform float material_dif;
uniform float material_spec;

uniform sampler2D texture_diffuse1;

in vec3 Pos;
in vec3 Normal;
in vec2 TexCoords;

void main()
{ 
    vec3 color = vec3(0.10,0.0,0.60);
    int levels = 5;
    float scaleFactor = 1.0 / levels;
	vec3 diffuseColor = vec3(0.30,0.80,0.10);
 
	//light
	vec3 Light = normalize( lightPos - Pos);
	//view
	vec3 View = normalize( viewPos - Pos);
	 
	float diffuse = max(0, dot(Light,Normal));
	diffuseColor = diffuseColor * material_dif * floor(diffuse * levels) * scaleFactor;
	
	//angle for reflect light to view 
	vec3 reflectDir = normalize(Light + View);
	 
	 float specular = 0.0;
	 
	 if( dot(Light,Normal) > 0.0)
	 {
		specular = material_spec * pow( max(0, dot( reflectDir, Normal)), material_shininess);
	 }
	 
	 //limit specular
	 float specMask = (pow(dot(reflectDir, Normal), material_shininess) > 0.4) ? 1 : 0;
	 
	 float edgeDetection = (dot(View, Normal) > 0.2) ? 1 : 0;
	 
	 color = edgeDetection * (color + diffuseColor + specular * specMask);
	 float greyscale= dot(texture(texture_diffuse1, TexCoords).xyz, vec3(0.299, 0.587, 0.114));
	 
	 FragColor = vec4(color,1);
}