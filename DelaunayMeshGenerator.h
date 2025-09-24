#pragma once
#include <vector>
#include <glm/glm.hpp>
#include "Simulation2D.h"
#include "Boundary.h"
class Simulation2D;

class DelaunayMeshGenerator {
public:
    struct Triangle {
        unsigned int v0, v1, v2;
    };

    DelaunayMeshGenerator() = default;

    // 核心函数：接收粒子和边界，生成CDT网格
    void generate_mesh(const std::vector<Simulation2D::Particle>& particles, const Boundary& boundary, float min_particle_spacing);

    const std::vector<glm::vec2>& get_vertices() const { return vertices_; }
    const std::vector<Triangle>& get_triangles() const { return triangles_; }

private:
    std::vector<glm::vec2> vertices_;
    std::vector<Triangle> triangles_;
};