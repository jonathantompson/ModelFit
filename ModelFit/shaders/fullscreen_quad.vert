#version 150

in vec3 v_position;

out vec2 f_texture;

void main(){
	gl_Position =  vec4(v_position, 1);
	// f_texture = (v_position.xy + vec2(1,1)) * 0.5;
	f_texture = v_position.xy * 0.5 + 0.5;
}

