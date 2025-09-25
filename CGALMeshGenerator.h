#pragma once
#include <vector>
#include <glm/glm.hpp>
#include "Simulation2D.h"
#include "Boundary.h"

// --- 关键修复：定义正确的CGAL类型 ---
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Triangulation_face_base_with_info_2.h>
#include <CGAL/Constrained_triangulation_face_base_2.h> // <-- 必须包含这个头文件

// 区域内/外标记
struct FaceInfo2 {
    bool in_domain() const { return nesting_level % 2 == 1; }
    int nesting_level = 0;
};

// CGAL 内核
using K = CGAL::Exact_predicates_inexact_constructions_kernel;

// 创建一个复合的面基类：它既有约束边的信息，也有我们自定义的信息
using Cfb = CGAL::Constrained_triangulation_face_base_2<K>;
using Fbb = CGAL::Triangulation_face_base_with_info_2<FaceInfo2, K, Cfb>;

// 顶点和数据结构
using Vb = CGAL::Triangulation_vertex_base_2<K>;
using Tds = CGAL::Triangulation_data_structure_2<Vb, Fbb>;

// 最终的约束德劳内三角剖分类型
using CDT = CGAL::Constrained_Delaunay_triangulation_2<K, Tds>;
using Point = CDT::Point;
// --- 结束 ---


class CGALMeshGenerator {
public:
    struct Triangle {
        unsigned int v0, v1, v2;
    };
    // Quad 结构体定义移到 Qmorph.h 中更合适，但为了 Viewer 能访问，暂时保留
    struct Quad {
        unsigned int v0, v1, v2, v3;
    };

    CGALMeshGenerator() = default;
    ~CGALMeshGenerator() = default;

    // 函数签名现在是正确的
    void generate_mesh(const std::vector<Simulation2D::Particle>& particles, const Boundary& boundary);

    const std::vector<glm::vec2>& get_vertices() const { return vertices_; }
    const std::vector<Triangle>& get_triangles() const { return triangles_; }
    const std::vector<Quad>& get_quads() const { return quads_; }

private:
    std::vector<glm::vec2> vertices_;
    std::vector<Triangle> triangles_;
    std::vector<Quad> quads_; // 新增

};