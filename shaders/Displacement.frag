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
  return texture(u_texture_2, uv).x;;
}

void main() {
  // YOUR CODE HERE
  
  mat3 tbn;
  vec3 b = cross(v_normal.xyz, v_tangent.xyz);
  tbn[0] = v_tangent.xyz;
  tbn[1] = b;
  tbn[2] = v_normal.xyz;

  float kh = u_height_scaling;
  float kn = u_normal_scaling;
  float u = v_uv.x;
  float v = v_uv.y;

  float wid = u_texture_2_size.x;
  float hei = u_texture_2_size.y;

  float du = (h(vec2(u + 1.0/wid, v)) - h(v_uv)) * kh * kn;
  float dv = (h(vec2(u, v + 1.0/hei)) - h(v_uv)) * kh * kn;


  vec3 no = vec3(-du, -dv, 1);
  vec3 nd = (tbn * normalize(no));
  

  float r = distance(u_light_pos, v_position.xyz);
  vec3 illumination = (u_light_intensity/(r*r));
  vec3 incoming = normalize(u_light_pos - v_position.xyz);
  vec3 outgoing = normalize(u_cam_pos - v_position.xyz);


  float ka = 0.05;
  float kd = 1.0;
  float ks = 0.6;

  vec3 Ia = vec3(1.0, 1.0, 1.0);
  vec3 term1 = ka * Ia;
  vec3 term2 = kd * illumination * max(0.0, dot(nd, incoming));
  vec3 h = normalize(incoming + outgoing);
  float p = 100.0;
  vec3 term3 = ks * illumination * pow(max(0.0, dot(nd, h)), p);




  // (Placeholder code. You will want to replace it.)
  out_color = vec4(term1 + term2 + term3, 1.0) * u_color;
  out_color.a = 1.0;
}

