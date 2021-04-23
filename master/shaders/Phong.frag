#version 330

uniform vec4 u_color;
uniform vec3 u_cam_pos;
uniform vec3 u_light_pos;
uniform vec3 u_light_intensity;

in vec4 v_position;
in vec4 v_normal;
in vec2 v_uv;

out vec4 out_color;

void main() {
  float ka = .3;
  float kd = 0.5;
  float ks = 10;
  vec3 Ia = vec3(1, 1, 1);
  int p = 80;
  vec3 r = u_light_pos - v_position.xyz;
  vec3 I_R = u_light_intensity / dot(r, r);
  vec3 h = normalize(u_light_pos + u_cam_pos - 2. * v_position.xyz);
  vec3 a = ka * Ia;
  vec3 d = kd * I_R * max(0., dot(v_normal.xyz, normalize(r)));
  vec3 s = ks * I_R * pow(max(0., dot(v_normal.xyz, h)), p);
  out_color.rgb = a + d + s;
  out_color.a = 1;
}

