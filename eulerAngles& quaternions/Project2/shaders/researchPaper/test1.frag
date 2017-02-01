#version 330 core
out vec4 color;

in float gDist;
in vec2 gSpine;


uniform sampler2D texture_diffuse1;
uniform vec3 viewPos;

const vec3 LineColor = vec3(0, 0, 0);

void main()
{
    float alpha = 1.0;
    float d = abs(gDist);
    float tipLength = 2.0 * fwidth(d);
    if (d > 0.001 - tipLength)
        alpha = 1.0 - (d - 0.001 + tipLength) / tipLength;
		
    color = vec4(LineColor, alpha);
}