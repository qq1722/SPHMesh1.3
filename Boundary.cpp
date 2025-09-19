#include "Boundary.h"
#include <algorithm> // for std::min/max

Boundary::Boundary(const std::vector<glm::vec2>& vertices) : vertices_(vertices) 
{
    calculate_aabb();
}

const std::vector<glm::vec2>& Boundary::get_vertices() const 
{
    return vertices_;
}

const glm::vec4& Boundary::get_aabb() const 
{
    return aabb_;
}

void Boundary::calculate_aabb() 
{
    if (vertices_.empty()) {
        aabb_ = glm::vec4(0.0f, 0.0f, 0.0f, 0.0f);
        return;
    }

    glm::vec2 min_coords = vertices_[0];
    glm::vec2 max_coords = vertices_[0];

    for (size_t i = 1; i < vertices_.size(); ++i) 
    {
        min_coords.x = std::min(min_coords.x, vertices_[i].x);
        min_coords.y = std::min(min_coords.y, vertices_[i].y);
        max_coords.x = std::max(max_coords.x, vertices_[i].x);
        max_coords.y = std::max(max_coords.y, vertices_[i].y);
    }
    aabb_ = glm::vec4(min_coords.x, min_coords.y, max_coords.x, max_coords.y);
}

// 使用 Ray-Casting (射线投射) 算法判断点是否在多边形内部
bool Boundary::is_inside(const glm::vec2& point) const 
{
    if (vertices_.empty()) 
    {
        return false;
    }
    int num_vertices = vertices_.size();
    bool inside = false;

    // 遍历多边形的每一条边
    for (int i = 0, j = num_vertices - 1; i < num_vertices; j = i++) 
    {
        const auto& p1 = vertices_[i];
        const auto& p2 = vertices_[j];

        // 检查点是否在边的Y轴范围内
        if (((p1.y > point.y) != (p2.y > point.y)) &&
            // 检查点是否在射线(从点向右)的左侧
            (point.x < (p2.x - p1.x) * (point.y - p1.y) / (p2.y - p1.y) + p1.x)) 
        {
            inside = !inside; // 找到一个交点，内外状态反转
        }
    }
    return inside;
}