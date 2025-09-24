#include "CGALMeshGenerator.h"
#include <iostream>
#include <map>
#include <vector>

void CGALMeshGenerator::generate_mesh(const std::vector<Simulation2D::Particle>& particles, const Boundary& boundary) {
    vertices_.clear();
    triangles_.clear();

    const auto& boundary_vertices = boundary.get_vertices();
    if (particles.empty() || boundary_vertices.empty()) return;

    CDT cdt;

    // --- 步骤 1: 只使用内部粒子进行三角剖分 ---
    std::vector<Point> interior_points;
    for (const auto& p : particles) {
        // 我们只关心那些自由移动的粒子，因为它们的位置经过了优化
        if (!p.is_boundary) {
            interior_points.emplace_back(p.position.x, p.position.y);
        }
    }

    if (interior_points.empty()) {
        std::cerr << "Warning: No interior particles to generate mesh from." << std::endl;
        return;
    }

    cdt.insert(interior_points.begin(), interior_points.end());


    // --- 步骤 2 & 3: 筛选并提取内部三角形 (质心判断法) ---
    // 这个map将CGAL的内部顶点句柄映射到我们自己的连续索引
    std::map<CDT::Vertex_handle, unsigned int> vertex_map;

    for (auto face_it = cdt.finite_faces_begin(); face_it != cdt.finite_faces_end(); ++face_it) {
        // a) 计算三角形的质心
        Point p0 = face_it->vertex(0)->point();
        Point p1 = face_it->vertex(1)->point();
        Point p2 = face_it->vertex(2)->point();
        glm::vec2 centroid((p0.x() + p1.x() + p2.x()) / 3.0f, (p0.y() + p1.y() + p2.y()) / 3.0f);

        // b) 使用边界判断质心是否在区域内
        if (boundary.is_inside(centroid)) {
            unsigned int v_indices[3];
            for (int i = 0; i < 3; ++i) {
                CDT::Vertex_handle vh = face_it->vertex(i);

                // c) 如果是新顶点，添加到我们的顶点列表并记录其索引
                if (vertex_map.find(vh) == vertex_map.end()) {
                    vertex_map[vh] = (unsigned int)vertices_.size();
                    vertices_.emplace_back(vh->point().x(), vh->point().y());
                }
                v_indices[i] = vertex_map[vh];
            }
            // d) 添加这个合格的三角形
            triangles_.push_back({ v_indices[0], v_indices[1], v_indices[2] });
        }
    }

    std::cout << "CGAL generated (Clip Method): " << vertices_.size() << " vertices, " << triangles_.size() << " triangles." << std::endl;
}