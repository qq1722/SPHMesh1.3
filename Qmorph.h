#pragma once
#include <vector>
#include <glm/glm.hpp>
#include "CGALMeshGenerator.h"

// 为了避免循环引用，我们只进行前向声明
class CGALMeshGenerator;

class Qmorph {
public:
    // 定义输出结果的结构体
    struct Result {
        std::vector<CGALMeshGenerator::Quad> quads;
        std::vector<CGALMeshGenerator::Triangle> remaining_triangles;
    };

    Qmorph() = default;

    // 核心函数：接收一个三角网格，返回一个四边形为主的网格
    Result run(const CGALMeshGenerator& delaunay_mesh);

private:
    // 私有辅助函数
    float calculate_quad_quality(const glm::vec2& p1, const glm::vec2& p2, const glm::vec2& p3, const glm::vec2& p4);
};