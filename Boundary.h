#pragma once
#include <vector>
#include <glm/glm.hpp>
#include <string>

class Boundary 
{
public:
    // 构造函数，接收一个定义边界形状的顶点列表
    Boundary(const std::vector<glm::vec2>& vertices);

    // 判断一个点是否在边界内部 (核心算法)
    bool is_inside(const glm::vec2& point) const;

    // 获取边界的所有顶点，用于渲染
    const std::vector<glm::vec2>& get_vertices() const;

    // 获取边界的轴对齐包围盒 (AABB)，方便后续计算
    const glm::vec4& get_aabb() const;

private:
    std::vector<glm::vec2> vertices_;
    glm::vec4 aabb_; // x_min, y_min, x_max, y_max

    void calculate_aabb(); // 私有辅助函数，用于计算包围盒
};