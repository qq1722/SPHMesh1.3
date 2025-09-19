#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in float aDensity; // <<-- 新增：接收密度属性

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

out float density; // <<-- 新增：将密度传递给片段着色器

void main()
{
    gl_Position = projection * view * model * vec4(aPos, 1.0);
    gl_PointSize = 8.0;
    density = aDensity; // <<-- 传递
}