#include "DelaunayMeshGenerator.h"
#include "Cdd.h"
#include <iostream>
#include <map>

void DelaunayMeshGenerator::generate_mesh(const std::vector<Simulation2D::Particle>& particles, const Boundary& boundary) {
    vertices_.clear();
    triangles_.clear();

    const auto& boundary_vertices = boundary.get_vertices();
    if (particles.empty() || boundary_vertices.empty()) return;

    CDT::Triangulation<float> cdt;

    // 1. 准备顶点：先放边界顶点，这样它们的索引就是固定的 0 到 N-1
    std::vector<CDT::V2d<float>> cdt_vertices;
    cdt_vertices.reserve(boundary_vertices.size() + particles.size());
    for (const auto& bv : boundary_vertices) {
        cdt_vertices.push_back({ bv.x, bv.y });
    }
    // 然后放入内部粒子
    for (const auto& p : particles) {
        if (!p.is_boundary) {
            cdt_vertices.push_back({ p.position.x, p.position.y });
        }
    }
    cdt.insertVertices(cdt_vertices);

    // 2. 准备约束边
    std::vector<CDT::Edge<float>> cdt_edges;
    cdt_edges.reserve(boundary_vertices.size());
    for (CDT::VertInd i = 0; i < boundary_vertices.size(); ++i) {
        cdt_edges.emplace_back(CDT::Edge<float>{i, (i + 1) % (CDT::VertInd)boundary_vertices.size()});
    }
    cdt.insertEdges(cdt_edges);

    // 3. 执行三角化并移除外部三角形
    cdt.eraseOuterTrianglesAndHoles();

    // 4. 提取结果
    vertices_.clear();
    const auto& result_vertices = cdt.vertices;
    vertices_.reserve(result_vertices.size());
    for (const auto& v : result_vertices) {
        vertices_.emplace_back(v.x, v.y);
    }

    triangles_.clear();
    const auto& result_triangles = cdt.triangles;
    triangles_.reserve(result_triangles.size());
    for (const auto& t : result_triangles) {
        triangles_.push_back({ t.vertices[0], t.vertices[1], t.vertices[2] });
    }

    std::cout << "CDT generated: " << vertices_.size() << " vertices, " << triangles_.size() << " triangles." << std::endl;
}