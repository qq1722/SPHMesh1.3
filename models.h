#pragma once
#include <vector>
#include <glm/glm.hpp>
#include <algorithm> // for std::min/max
#include <iostream>

// ��������ȡһ�������ӵġ����ƺ����ı߽���״
inline std::vector<glm::vec2> get_lake_shape_vertices() {
    std::vector<glm::vec2> vertices = {
        {2.2f, 8.1f}, {4.5f, 9.2f}, {7.3f, 8.5f}, {9.1f, 6.7f},
        {10.5f, 4.2f}, {11.2f, 1.5f}, {10.1f, -1.1f}, {8.3f, -3.2f},
        {6.1f, -4.5f}, {3.5f, -5.1f}, {1.2f, -4.2f}, {-1.5f, -3.5f},
        {-3.1f, -2.1f}, {-4.5f, 0.5f}, {-5.2f, 2.8f}, {-4.1f, 5.3f},
        {-2.5f, 7.1f}, {0.1f, 6.5f}
    };

    // ��һ������ʹ��ߴ��ʺϵ�ǰ��ͼ
    if (vertices.empty()) return {};

    glm::vec2 min_coords = vertices[0];
    glm::vec2 max_coords = vertices[0];
    for (const auto& v : vertices) {
        min_coords.x = std::min(min_coords.x, v.x);
        min_coords.y = std::min(min_coords.y, v.y);
        max_coords.x = std::max(max_coords.x, v.x);
        max_coords.y = std::max(max_coords.y, v.y);
    }
    glm::vec2 center = (min_coords + max_coords) * 0.5f;
    glm::vec2 size = max_coords - min_coords;
    float max_dim = std::max(size.x, size.y);
    float target_size = 10.0f; // Ŀ��ߴ�
    float scale = (max_dim > 1e-6) ? (target_size / max_dim) : 1.0f;
    for (auto& v : vertices) {
        v = (v - center) * scale;
    }

    return vertices;
}