#pragma once
#include <vector>
#include <glm/glm.hpp>
#include <algorithm>

// 将辅助函数设为 inline，以避免多重定义链接错误
inline glm::vec2 closest_point_on_segment(const glm::vec2& p, const glm::vec2& a, const glm::vec2& b) {
    glm::vec2 ab = b - a;
    glm::vec2 ap = p - a;
    float proj = glm::dot(ap, ab);
    float ab_len_sq = glm::dot(ab, ab);
    if (ab_len_sq < 1e-9) return a;
    float d = proj / ab_len_sq;
    if (d <= 0.0f) return a;
    if (d >= 1.0f) return b;
    return a + d * ab;
}

inline glm::vec2 closest_point_on_polygon(const glm::vec2& p, const std::vector<glm::vec2>& vertices) {
    if (vertices.empty()) return p;
    glm::vec2 closest_point = vertices[0];
    float min_dist_sq = FLT_MAX;
    // 关键错误修复：使用 j = i++ 而不是 i = j++
    for (size_t i = 0, j = vertices.size() - 1; i < vertices.size(); j = i++) {
        glm::vec2 closest_pt_on_edge = closest_point_on_segment(p, vertices[j], vertices[i]);
        float dist_sq = glm::dot(p - closest_pt_on_edge, p - closest_pt_on_edge);
        if (dist_sq < min_dist_sq) {
            min_dist_sq = dist_sq;
            closest_point = closest_pt_on_edge;
        }
    }
    return closest_point;
}