#version 330 core
layout (triangles_adjacency) in;
layout (triangle_strip, max_vertices = 12) out;

in VS_OUT {
    vec2 texCoords;
	vec3 World_normal;
	vec3 World_pos;
} gs_in[];

out float gDist;
out vec2 gSpine;


uniform int mode;
uniform float time;

bool IsFront(vec3 A, vec3 B, vec3 C);

/*
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
*/
bool IsFront(vec3 A, vec3 B, vec3 C)
{
    float area = (A.x * B.y - B.x * A.y) + (B.x * C.y - C.x * B.y) + (C.x * A.y - A.x * C.y);
    return area > 0;
}

bool IsFront(vec2 A, vec2 B, vec2 C)
{
    return 0 < (A.x * B.y - B.x * A.y) + (B.x * C.y - C.x * B.y) + (C.x * A.y - A.x * C.y);
}

void EmitEdge(vec2 P0, vec2 P1)
{
	float OverhangLength=0.1;
	float HalfWidth=1;
	
	vec2 E = OverhangLength * (P1 - P0);
	vec2 V = normalize(E);
    vec2 N = vec2(-V.y, V.x) * HalfWidth;
	
	gSpine = (P0 + 1.0) * 0.5;
    gDist = +HalfWidth;
	gl_Position = vec4(P0 - N - E, 0, 1); EmitVertex();
    gDist = -HalfWidth;
    gl_Position = vec4(P0 + N - E, 0, 1); EmitVertex();
    gSpine = (P1 + 1.0) * 0.5;
    gDist = +HalfWidth;
    gl_Position = vec4(P1 - N + E, 0, 1); EmitVertex();
    gDist = -HalfWidth;
    gl_Position = vec4(P1 + N + E, 0, 1); EmitVertex();
    EndPrimitive();


}

void EmitEdge(vec3 P0, vec3 P1)
{
	float OverhangLength=1.0;
	float HalfWidth=.001;
	
    vec3  E = OverhangLength * vec3(P1.xy - P0.xy, 0);
    vec2  V = normalize(E.xy);
    vec3  N = vec3(-V.y, V.x, 0) * HalfWidth;
    vec3  S = -N;
    float D = HalfWidth;

    //gSpine = P0;
    gl_Position = vec4(P0 + S - E, 1); gDist = +D; EmitVertex();
 	gl_Position = vec4(P0 + N - E, 1); gDist = -D; EmitVertex();
    //gSpine = P1;
 	gl_Position = vec4(P1 + S + E, 1); gDist = +D; EmitVertex();
    gl_Position = vec4(P1 + N + E, 1); gDist = -D; EmitVertex();
    EndPrimitive();
}




void main() {
	

    vec3 v0 = gl_in[0].gl_Position.xyz / gl_in[0].gl_Position.w;
    vec3 v1 = gl_in[1].gl_Position.xyz / gl_in[1].gl_Position.w;
    vec3 v2 = gl_in[2].gl_Position.xyz / gl_in[2].gl_Position.w;
    vec3 v3 = gl_in[3].gl_Position.xyz / gl_in[3].gl_Position.w;
    vec3 v4 = gl_in[4].gl_Position.xyz / gl_in[4].gl_Position.w;
    vec3 v5 = gl_in[5].gl_Position.xyz / gl_in[5].gl_Position.w;
    
	
	if(mode!=1)
	{
		EmitEdge(v0, v2);
		EmitEdge(v2, v4);
		EmitEdge(v4, v0);
		
	}else{
		if (IsFront(v0, v2, v4)) {
			if (!IsFront(v0, v1, v2)) EmitEdge(v0, v2);
			if (!IsFront(v2, v3, v4)) EmitEdge(v2, v4);
			if (!IsFront(v0, v4, v5)) EmitEdge(v4, v0);
		}
	}
}