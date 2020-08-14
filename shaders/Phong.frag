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
  
  // (Placeholder code. You will want to replace it.)

    vec3 diffuseCoefficient = vec3(1.0, 1.0, 1.0);
    vec3 diffuseCoefficient1 = vec3(.2, .2, .2);
    float radius = length(u_light_pos - v_position.xyz);

    vec3 second = diffuseCoefficient * u_light_intensity * max(0.0, dot(v_normal, v_position))/(radius * radius);


    vec3 h = normalize(v_position.xyz + u_light_pos);

    vec3 third = diffuseCoefficient1 * u_light_intensity * pow(max(0.0, dot(v_normal.xyz, h)), 90) / (radius * radius);
    out_color = u_color + vec4(second,0.0) + vec4(third,0.0);
//  out_color = (vec4(1, 1, 1, 0) + v_normal) / 2;
    out_color.a = 1;
}

