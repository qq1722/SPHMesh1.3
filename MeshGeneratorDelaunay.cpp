#include "MeshGeneratorDelaunay.h"
#include "Simulation2D.h"
#include "triangle.h"
#include <vector>

// *** 核心修復：在包含頭文件之前，定義所有必要的宏 ***
#define TRILIBRARY
#define REAL double
extern "C" {
#include "triangle.h"
}

void MeshGeneratorDelaunay::triangulate1(const Simulation2D& sim) {
    vertices_.clear();
    triangles_.clear();

    const auto& particles = sim.get_particle_positions();
    if (particles.empty()) return;

    vertices_ = particles;

    // --- 準備 Triangle 庫所需的輸入數據 ---
    std::vector<REAL> points_for_triangle; // 使用 REAL 而不是 double
    points_for_triangle.reserve(vertices_.size() * 2);
    for (const auto& v : vertices_) {
        points_for_triangle.push_back(v.x);
        points_for_triangle.push_back(v.y);
    }

    // --- 初始化輸入和輸出結構體 ---
    struct triangulateio in, out, vorout;

    // 初始化输入结构体
    in.numberofpoints = static_cast<int>(vertices_.size());
    in.pointlist = points_for_triangle.data();
    in.numberofpointattributes = 0;
    in.pointattributelist = nullptr;
    in.pointmarkerlist = nullptr;
    in.numberofsegments = 0;
    in.segmentlist = nullptr;
    in.segmentmarkerlist = nullptr;
    in.numberofholes = 0;
    in.holelist = nullptr;
    in.numberofregions = 0;
    in.regionlist = nullptr;

    // 初始化输出结构体
    out.pointlist = nullptr;
    out.pointattributelist = nullptr;
    out.pointmarkerlist = nullptr;
    out.trianglelist = nullptr;
    out.triangleattributelist = nullptr;
    out.neighborlist = nullptr;
    out.segmentlist = nullptr;
    out.segmentmarkerlist = nullptr;
    out.edgelist = nullptr;
    out.edgemarkerlist = nullptr;

    // 初始化Voronoi输出结构体（即使不使用）
    vorout.pointlist = nullptr;
    vorout.pointattributelist = nullptr;
    vorout.pointmarkerlist = nullptr;
    vorout.trianglelist = nullptr;
    vorout.triangleattributelist = nullptr;
    vorout.neighborlist = nullptr;
    vorout.segmentlist = nullptr;
    vorout.segmentmarkerlist = nullptr;
    vorout.edgelist = nullptr;
    vorout.edgemarkerlist = nullptr;
    char* zq = NULL;
    // --- 使用4個參數調用 triangulate ---
    ::triangulate(const_cast<char*>("zQ"), &in, &out, &vorout);

    // --- 從輸出結果中提取三角形索引 ---
    triangles_.reserve(out.numberoftriangles);
    for (int i = 0; i < out.numberoftriangles; ++i) {
        triangles_.push_back({
            static_cast<unsigned int>(out.trianglelist[i * 3 + 0]),
            static_cast<unsigned int>(out.trianglelist[i * 3 + 1]),
            static_cast<unsigned int>(out.trianglelist[i * 3 + 2])
            });
    }

    // --- 釋放內存 ---
    free(out.pointlist);
    free(out.pointattributelist);
    free(out.pointmarkerlist);
    free(out.trianglelist);
    free(out.triangleattributelist);
    free(out.neighborlist);
    free(out.segmentlist);
    free(out.segmentmarkerlist);
    free(out.edgelist);
    free(out.edgemarkerlist);

    // 釋放Voronoi輸出內存
    free(vorout.pointlist);
    free(vorout.pointattributelist);
    free(vorout.pointmarkerlist);
    free(vorout.trianglelist);
    free(vorout.triangleattributelist);
    free(vorout.neighborlist);
    free(vorout.segmentlist);
    free(vorout.segmentmarkerlist);
    free(vorout.edgelist);
    free(vorout.edgemarkerlist);
}