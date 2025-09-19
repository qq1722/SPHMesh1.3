#version 330 core
out vec4 FragColor;

in float density; // <<-- 新增：接收密度

uniform float restDensity; // 从C++传入的目标密度

void main()
{
    // 根据当前密度与目标密度的比值，映射到一个蓝->绿->红的色带
    float ratio = density / restDensity;
    vec3 color = vec3(0.0);

    if (ratio < 1.0) {
        // 密度低：从蓝到绿
        color = mix(vec3(0.0, 0.0, 1.0), vec3(0.0, 1.0, 0.0), ratio);
    } else {
        // 密度高：从绿到红
        color = mix(vec3(0.0, 1.0, 0.0), vec3(1.0, 0.0, 0.0), min((ratio - 1.0), 1.0));
    }

    FragColor = vec4(color, 1.0);
}