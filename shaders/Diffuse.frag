#version 330

// The camera's position in world-space
uniform vec3 u_cam_pos;

// Color
uniform vec4 u_color;

// Properties of the single point light
uniform vec3 u_light_pos;
uniform vec3 u_light_intensity;

// We also get the uniform texture we want to use.
uniform sampler2D u_texture_1;

// These are the inputs which are the outputs of the vertex shader.
in vec4 v_position;
in vec4 v_normal;

// This is where the final pixel color is output.
// Here, we are only interested in the first 3 dimensions (xyz).
// The 4th entry in this vector is for "alpha blending" which we
// do not require you to know about. For now, just set the alpha
// to 1.
out vec4 out_color;

void main() {
  // YOUR CODE HERE
  float kd = 1.0;
  float r = distance(u_light_pos, v_position.xyz);
  vec3 illumination = (u_light_intensity/pow(r, 2));
  vec3 incoming = normalize(u_light_pos - v_position.xyz);
  vec3 outgoing = normalize(u_cam_pos - v_position.xyz);


  vec3 term2 = u_color.xyz * illumination * max(0.0, dot(v_normal.xyz, incoming));
  out_color = vec4(term2, 1.0);

}
