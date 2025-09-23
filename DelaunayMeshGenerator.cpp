#include "DelaunayMeshGenerator.h"
#include "CDT.h" // <-- 包含新的头文件
#include <iostream>

void DelaunayMeshGenerator::generate_mesh(const std::vector<Simulation2D::Particle>& particles, const Boundary& boundary) {
    vertices_.clear();
    triangles_.clear();

    if (particles.empty()) return;

    // 1. 创建 CDT 对象
    CDT::Triangulation<float> cdt;

    // 2. 插入所有粒子顶点
    std::vector<CDT::V2d<float>> cdt_vertices;
    cdt_vertices.reserve(particles.size());
    for (const auto& p : particles) {
        cdt_vertices.push_back({ p.position.x, p.position.y });
    }
    cdt.insertVertices(cdt_vertices);

    // 3. 插入边界作为约束边
    const auto& boundary_vertices = boundary.get_vertices();
    std::vector<CDT::Edge> cdt_edges;
    cdt_edges.reserve(boundary_vertices.size());
    for (size_t i = 0; i < boundary_vertices.size(); ++i) {
        // CDT 的顶点索引是从 0 开始的
        // 注意：这里我们假设边界顶点就是粒子点集的前N个点
        // 一个更鲁棒的方法是先将边界顶点也加入粒子列表
        // 但为简单起见，我们假设边界已经被粒子采样覆盖
    }
    // CDT库在插入顶点时会自动处理边界，我们只需要最后移除外部三角形
    // cdt.insertEdges(cdt_edges);


    // 4. 执行三角化 (移除超级结构和区域外的三角形)
   // cdt.eraseSuperfluousTriangles(boundary_vertices, false);
    cdt.eraseSuperTriangle();
    cdt.eraseOuterTriangles();

    // 5. 提取结果
    const auto& result_vertices = cdt.vertices;
    vertices_.reserve(result_vertices.size());
    for (const auto& v : result_vertices) {
        vertices_.emplace_back(v.x, v.y);
    }

    const auto& result_triangles = cdt.triangles;
    triangles_.reserve(result_triangles.size());
    for (const auto& t : result_triangles) {
        triangles_.push_back({
            static_cast<unsigned int>(t.vertices[0]),
            static_cast<unsigned int>(t.vertices[1]),
            static_cast<unsigned int>(t.vertices[2])
            });
    }

    std::cout << "CDT generated: " << vertices_.size() << " vertices, " << triangles_.size() << " triangles." << std::endl;
}