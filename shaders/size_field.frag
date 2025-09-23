#version 330 core
out vec4 FragColor;
in float fValue; // 接收 h_t 值

uniform float vmin;
uniform float vmax;

// 将一个值从一个范围映射到另一个范围
float map(float value, float min1, float max1, float min2, float max2) {
    return min2 + (value - min1) * (max2 - min2) / (max1 - min1);
}

// 经典的 Jet colormap (蓝 -> 青 -> 黄 -> 红)
vec3 jet(float t) {
    t = clamp(t, 0.0, 1.0);
    vec3 c = vec3(1.0);
    if (t < 0.34) {
        c.r = 0.0;
        c.g = map(t, 0.0, 0.34, 0.0, 1.0);
    } else if (t < 0.64) {
        c.r = map(t, 0.34, 0.64, 0.0, 1.0);
        c.g = 1.0;
        c.b = map(t, 0.34, 0.64, 1.0, 0.0);
    } else {
        c.r = 1.0;
        c.g = map(t, 0.64, 1.0, 1.0, 0.0);
        c.b = 0.0;
    }
    return c;
}

void main()
{
    float t = (fValue - vmin) / (vmax - vmin);
    // 论文中的配色更像是反过来的 magma，我们用jet近似
    FragColor = vec4(jet(1.0-t), 1.0); 
}