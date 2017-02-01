#version 330 core
layout (triangles_adjacency) in;
layout (triangle_strip, max_vertices = 12) out;

in VS_OUT {
	vec3 Pos;
	vec3 Normal;
	vec2 texCoords;
} gs_in[];

out vec2 TexCoords; 
out vec3 Normal;
out	vec3 Pos;

uniform int mode;
uniform float time;

bool IsFront(vec3 A, vec3 B, vec3 C);


vec4 explode(vec4 position, vec3 normal)
{
    float magnitude = 2.0f;
    vec3 direction = normal * ((sin(time) + 1.0f) / 2.0f) * magnitude; 
    return position + vec4(direction, 0.0f);
}


vec3 GetNormal()
{
    vec3 a = vec3(gl_in[0].gl_Position) - vec3(gl_in[1].gl_Position);
    vec3 b = vec3(gl_in[2].gl_Position) - vec3(gl_in[1].gl_Position);
    return normalize(cross(a, b));
}


void main() {    
    vec3 normal = GetNormal();
	
    gl_Position = gl_in[0].gl_Position;
    TexCoords = gs_in[0].texCoords;
	Pos=gs_in[0].Pos;
	Normal=gs_in[1].Normal;
    EmitVertex();
    /*
	gl_Position = explode(gl_in[1].gl_Position, normal);
    TexCoords = gs_in[1].texCoords;
	Pos=gs_in[1].Pos;
	Normal=gs_in[1].Normal;
    EmitVertex();
    
	gl_Position = explode(gl_in[2].gl_Position, normal);
    TexCoords = gs_in[2].texCoords;
	Pos=gs_in[2].Pos;
	Normal=gs_in[1].Normal;
    EmitVertex();
	*/
    EndPrimitive();
}