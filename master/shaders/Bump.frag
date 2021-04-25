#version 330

uniform vec3 u_cam_pos;
uniform vec3 u_light_pos;
uniform vec3 u_light_intensity;

uniform vec4 u_color;

uniform sampler2D u_texture_2;
uniform vec2 u_texture_2_size;

uniform float u_normal_scaling;
uniform float u_height_scaling;

in vec4 v_position;
in vec4 v_normal;
in vec4 v_tangent;
in vec2 v_uv;

out vec4 out_color;

float h(vec2 uv) {
  // You may want to use this helper function...
  return texture(u_texture_2, uv)[0];
}

void main() {
  // YOUR CODE HERE
  
  // (Placeholder code. You will want to replace it.)
  mat3 tbn = mat3(v_tangent.xyz, cross(v_normal.xyz, v_tangent.xyz), v_normal.xyz);
  float k_k = u_height_scaling * u_normal_scaling;
  float du = (h(vec2(v_uv[0] + 1.0 / u_texture_2_size[0], v_uv[1])) - h(v_uv)) * k_k;
  float dv = (h(vec2(v_uv[0], v_uv[1] + 1.0 / u_texture_2_size[1])) - h(v_uv)) * k_k;
  vec3 n0 = vec3(-1.0 * du, -1.0 * dv, 1.0);
  vec3 nd = tbn * n0;
  
  // replace n v_normal with nd for phong 
  float ka = .1;
  float kd = 0.5;
  float ks = 0.8;
  vec3 Ia = vec3(1, 1, 1);
  int p = 50;
  vec3 r = u_light_pos - v_position.xyz;
  vec3 I_R = u_light_intensity / dot(r, r);
  vec3 h = normalize(u_light_pos + u_cam_pos - 2. * v_position.xyz);
  vec3 a = ka * Ia;
  vec3 d = kd * I_R * max(0., dot(nd, normalize(r)));
  vec3 s = ks * I_R * pow(max(0., dot(nd, h)), p);
  out_color.rgb = a + d + s;
  out_color.a = 1;
}

