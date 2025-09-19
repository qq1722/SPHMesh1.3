#pragma once
#include <vector>
#include <glm/glm.hpp>

class Simulation2D;
class Boundary;

class MeshGenerator2D {
public:
    struct Quad {
        unsigned int v0, v1, v2, v3;
    };
    struct Triangle {
        unsigned int v0, v1, v2;
    };

    MeshGenerator2D() = default;

    void generate(const Simulation2D& sim, const Boundary& boundary, float grid_size);

    const std::vector<glm::vec2>& get_vertices() const { return vertices_; }
    const std::vector<Quad>& get_quads() const { return quads_; }
    const std::vector<Triangle>& get_triangles() const { return triangles_; }

private:
    std::vector<glm::vec2> vertices_;
    std::vector<Quad> quads_;
    std::vector<Triangle> triangles_;
};