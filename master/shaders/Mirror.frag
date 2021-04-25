#version 330


uniform vec3 u_cam_pos;

uniform samplerCube u_texture_cubemap;

in vec4 v_position;
in vec4 v_normal;
in vec4 v_tangent;

out vec4 out_color;

void main() {
  vec3 w_o = normalize(u_cam_pos - v_position.xyz);
  vec3 w_i = 2 * dot(v_normal.xyz, w_o) * v_normal.xyz - w_o;
  out_color = texture(u_texture_cubemap, w_i);;
}
