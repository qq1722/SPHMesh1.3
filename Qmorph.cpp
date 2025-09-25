#include "Qmorph.h"
#include "CGALMeshGenerator.h" 
#include <iostream>
#include <map>
#include <vector>
#include <numeric>

// Q-Morph 核心算法的实现
Qmorph::Result Qmorph::run(const CGALMeshGenerator& delaunay_mesh) {
    const auto& triangles = delaunay_mesh.get_triangles();
    const auto& vertices = delaunay_mesh.get_vertices();

    if (triangles.empty()) {
        std::cout << "No triangles to convert." << std::endl;
        return {};
    }

    Result result;
    std::vector<bool> triangle_merged(triangles.size(), false);

    // 1. 构建邻接关系
    std::map<std::pair<unsigned int, unsigned int>, std::vector<size_t>> edge_to_triangle_map;
    for (size_t i = 0; i < triangles.size(); ++i) {
        auto& t = triangles[i];
        unsigned int v[3] = { t.v0, t.v1, t.v2 };
        for (int j = 0; j < 3; ++j) {
            unsigned int v1 = v[j];
            unsigned int v2 = v[(j + 1) % 3];
            if (v1 > v2) std::swap(v1, v2);
            edge_to_triangle_map[{v1, v2}].push_back(i);
        }
    }

    // 2. 遍历所有内部边，尝试合并
    for (auto const& [edge, tri_indices] : edge_to_triangle_map) {
        if (tri_indices.size() == 2) {
            size_t tri1_idx = tri_indices[0];
            size_t tri2_idx = tri_indices[1];

            if (triangle_merged[tri1_idx] || triangle_merged[tri2_idx]) {
                continue;
            }

            const auto& t1 = triangles[tri1_idx];
            const auto& t2 = triangles[tri2_idx];

            unsigned int shared_v1 = edge.first;
            unsigned int shared_v2 = edge.second;

            unsigned int other_v1 = (t1.v0 != shared_v1 && t1.v0 != shared_v2) ? t1.v0 : ((t1.v1 != shared_v1 && t1.v1 != shared_v2) ? t1.v1 : t1.v2);
            unsigned int other_v2 = (t2.v0 != shared_v1 && t2.v0 != shared_v2) ? t2.v0 : ((t2.v1 != shared_v1 && t2.v1 != shared_v2) ? t2.v1 : t2.v2);

            // 3. 质量检查
            float quality = calculate_quad_quality(vertices[other_v1], vertices[shared_v1], vertices[other_v2], vertices[shared_v2]);
            if (quality > 0.3f) { // 设定一个质量阈值
                result.quads.push_back({ other_v1, shared_v1, other_v2, shared_v2 });
                triangle_merged[tri1_idx] = true;
                triangle_merged[tri2_idx] = true;
            }
        }
    }

    // 4. 收集未被合并的三角形
    for (size_t i = 0; i < triangles.size(); ++i) {
        if (!triangle_merged[i]) {
            result.remaining_triangles.push_back(triangles[i]);
        }
    }

    std::cout << "Q-Morph Conversion complete: " << result.quads.size() << " quads, " << result.remaining_triangles.size() << " triangles remaining." << std::endl;
    return result;
}

// 质量评估函数：这里使用一个简单的基于角度的方法
float Qmorph::calculate_quad_quality(const glm::vec2& p1, const glm::vec2& p2, const glm::vec2& p3, const glm::vec2& p4) {
    auto angle = [](const glm::vec2& a, const glm::vec2& b, const glm::vec2& c) {
        glm::vec2 v1 = a - b;
        glm::vec2 v2 = c - b;
        return std::acos(glm::dot(glm::normalize(v1), glm::normalize(v2)));
        };

    float angles[4];
    angles[0] = angle(p4, p1, p2);
    angles[1] = angle(p1, p2, p3);
    angles[2] = angle(p2, p3, p4);
    angles[3] = angle(p3, p4, p1);

    float min_angle_deg = 180.0f, max_angle_deg = 0.0f;
    for (int i = 0; i < 4; ++i) {
        float deg = glm::degrees(angles[i]);
        min_angle_deg = std::min(min_angle_deg, deg);
        max_angle_deg = std::max(max_angle_deg, deg);
    }

    // 如果最大角太大或最小角太小，质量就很差
    if (max_angle_deg > 165.0f || min_angle_deg < 25.0f) {
        return 0.0f;
    }

    // 一个简单的质量度量：越接近90度越好
    float quality = 1.0f - (max_angle_deg - 90.0f) / 75.0f;
    return quality;
}