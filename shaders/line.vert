#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aForce;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;
uniform float force_scale;

void main()
{
    // gl_VertexID 是一个内置变量，表示当前正在处理的顶点是第几个
    // 我们将为每个粒子发送两个顶点的数据，但位置和力是相同的
    // 我们用 gl_VertexID 的奇偶性来区分是画起点还是终点
    vec3 end_point;
    if (gl_VertexID % 2 == 0) {
        // 偶数顶点，是线段的起点 (粒子位置)
        end_point = aPos;
    } else {
        // 奇数顶点，是线段的终点 (粒子位置 + 缩放后的力)
        end_point = aPos + aForce * force_scale;
    }
    gl_Position = projection * view * model * vec4(end_point, 1.0);
}