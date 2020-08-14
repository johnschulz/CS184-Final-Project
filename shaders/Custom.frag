#version 330

// (Every uniform is available here.)

uniform mat4 u_view_projection;
uniform mat4 u_model;

uniform float u_normal_scaling;
uniform float u_height_scaling;

uniform vec3 u_cam_pos;
uniform vec3 u_light_pos;
uniform vec3 u_light_intensity;

// Feel free to add your own textures. If you need more than 4,
// you will need to modify the skeleton.
uniform sampler2D u_texture_1;
uniform sampler2D u_texture_2;
uniform sampler2D u_texture_3;
uniform sampler2D u_texture_4;
uniform vec4 u_color;


// Environment map! Take a look at GLSL documentation to see how to
// sample from this.
uniform samplerCube u_texture_cubemap;

in vec4 v_position;
in vec4 v_normal;
in vec4 v_tangent;
in vec2 v_uv;

uniform bool toggleSpecularShading;
uniform bool toggleOutlines;
uniform float cutOff;
uniform float specularHighlight;
uniform vec4 outlineColor;
uniform float outlineSize;
uniform bool toggleDiffuseLighting;
uniform int diffuseLevel;
out vec4 out_color;

void main() {
  // Your awesome shader here!


    out_color = outlineColor;

    float reflectionCoef = dot(u_light_pos, v_normal.xyz);
    float diffuseArea = 2.0 / diffuseLevel;
    vec4 colorRange = (u_color - outlineColor) / diffuseLevel;
    if (toggleDiffuseLighting && diffuseLevel >= 2 && reflectionCoef < 1 - diffuseArea){
        for (int i = 0; i < diffuseLevel; i++){
            float startArea = float(i) * diffuseArea + -1.0;
            if (reflectionCoef >= startArea && reflectionCoef <= startArea + diffuseArea){
                out_color = outlineColor + float(i) * colorRange;
            }
        }
    } else{
        out_color = u_color;
    }
    
    vec3 lightDirection = normalize(u_light_pos - v_position.xyz);
    vec3 cameraDirection = normalize(u_cam_pos - v_position.xyz);
    vec3 h = normalize(lightDirection + cameraDirection/ length(lightDirection + cameraDirection));

    if (toggleSpecularShading){
        if (1 - cutOff < dot(vec4(h, 1.0), v_normal)){
            out_color = out_color + vec4(1, 1, 1, 1) * specularHighlight;
        }
    }


    if (toggleOutlines){
        if (dot(cameraDirection, v_normal.xyz)
        < mix(.1 + outlineSize, .1,
        max(0.0, dot(v_normal.xyz, lightDirection))))
        {
            out_color = outlineColor;
        }
    }


    out_color.a = 1;
}
