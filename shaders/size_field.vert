#version 330 core
layout (location = 0) in vec2 aPos;
layout (location = 1) in float aValue; // 接收 h_t 值

uniform mat4 view;
uniform mat4 projection;

out float fValue; // 传递给片段着色器

void main()
{
    gl_Position = projection * view * vec4(aPos, 0.0, 1.0);
    fValue = aValue;
}