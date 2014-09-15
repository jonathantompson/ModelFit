#version 150

in vec3 v_position;

out vec2 f_texture;

void main(){
	gl_Position =  vec4(v_position, 1);
	// FLIP the vertical texture coordinates!
	f_texture = (vec2(v_position.x, -v_position.y) + vec2(1,1)) * 0.5;
}

