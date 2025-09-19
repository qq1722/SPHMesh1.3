#pragma once
#include <vector>
#include <glm/glm.hpp>

class Simulation2D; // 前向聲明

class MeshGeneratorDelaunay {
public:
    // 用於儲存一個三角形的三個頂點索引
    struct Triangle {
        unsigned int v0, v1, v2;
    };

    MeshGeneratorDelaunay() = default;

    // 核心功能：接收粒子點並執行德勞內三角化
    void triangulate1(const Simulation2D& sim);

    const std::vector<glm::vec2>& get_vertices() const { return vertices_; }
    const std::vector<Triangle>& get_triangles() const { return triangles_; }

private:
    std::vector<glm::vec2> vertices_;
    std::vector<Triangle> triangles_;
};