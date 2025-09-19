#version 330 core
layout (location = 0) in vec2 aPos;

uniform mat4 projection;
uniform mat4 view;

void main()
{
    // 将2D坐标转换为3D，并应用相机变换
    gl_Position = projection * view * vec4(aPos.x, aPos.y, 0.0, 1.0);
    gl_PointSize = 1.0; // 设置点的大小
}