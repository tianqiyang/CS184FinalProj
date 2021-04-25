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
  // YOUR CODE HERE
  float r = distance(u_light_pos, v_position.xyz);
  vec3 illumination = (u_light_intensity/(r*r));
  vec3 incoming = normalize(u_light_pos - v_position.xyz);
  vec3 outgoing = normalize(u_cam_pos - v_position.xyz);


  float ka = 0.3;
  float kd = 1.0;
  float ks = 2.0;

  vec3 Ia = vec3(1.0, 1.0, 1.0);
  vec3 term1 = ka * Ia;
  
  vec3 term2 = kd * illumination * max(0.0, dot(v_normal.xyz, incoming));


  vec3 h = normalize(incoming + outgoing);
  float p = 100.0;
  vec3 term3 = ks * illumination * pow(max(0.0, dot(v_normal.xyz, h)), p);




  // (Placeholder code. You will want to replace it.)
  out_color = vec4(term1 + term2 + term3, 1.0) * u_color;
  out_color.a = 1.0;
}

