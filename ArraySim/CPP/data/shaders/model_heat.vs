#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aNormal;
layout (location = 2) in float heatMapValue;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

out float hValue;
void main()
{
	gl_Position = projection * view * model * vec4(aPos, 1.0); // add homogeneous w component
    hValue = heatMapValue;
    //FragPos = vec3(model * vec4(aPos, 1.0));
    //Normal = mat3(transpose(inverse(model))) * aNormal;
}