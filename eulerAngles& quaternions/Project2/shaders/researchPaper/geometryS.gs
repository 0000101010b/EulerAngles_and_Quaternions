#version 330 core
layout (triangles) in;
layout (triangle_strip, max_vertices = 3) out;

in VS_OUT {
    vec2 texCoords;
	vec3 World_normal;
	vec3 World_pos;
} gs_in[];

out vec2 TexCoords; 
out vec3 world_normal;
out	vec3 world_pos;


void main()
{
    gl_Position = gl_in[0].gl_Position;
    TexCoords = gs_in[0].texCoords;
	world_normal= gs_in[0].World_normal;
	world_pos=gs_in[0].World_pos;
    EmitVertex();
	
    gl_Position = gl_in[2].gl_Position;
    TexCoords = gs_in[2].texCoords;
	world_normal= gs_in[2].World_normal;
	world_pos=gs_in[2].World_pos;
    EmitVertex();
	
    gl_Position = gl_in[1].gl_Position;
    TexCoords = gs_in[1].texCoords;
	world_normal= gs_in[1].World_normal;
	world_pos=gs_in[1].World_pos;
    EmitVertex();
	
    EndPrimitive();
}