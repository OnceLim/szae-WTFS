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
  vec3 ambient = 0.1*vec3(1.0, 1.0, 1.0);

  vec3 l = u_light_pos - vec3(v_position);
  float r_square = dot(l,l);
  vec3 diffuse = vec3(u_color) * (u_light_intensity / r_square) * max(0, dot(normalize(vec3(v_normal)), normalize(l)));

  float k_s = 1;
  vec3 v = normalize(u_cam_pos - v_position.xyz);
  vec3 h = (v + normalize(l)) / length(v + normalize(l));
  vec3 spec = k_s * u_light_intensity / dot(l, l) * pow(max(0, dot(normalize(vec3(v_normal)), normalize(h))), 25);

  out_color = vec4(ambient+diffuse+spec, 1.0);
}

