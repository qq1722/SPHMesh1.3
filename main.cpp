#include "Viewer.h"
#include "Boundary.h"
#include "Simulation2D.h"
#include "MeshGenerator2D.h"
#include <iostream>
#include <vector>
#include <glm/glm.hpp>
#include <algorithm>

int main() {
    // --- 1. 選擇並歸一化模型 ---
    std::vector<glm::vec2> active_shape_vertices = {
        { 0.0f,  5.0f}, { 1.1f,  1.5f}, { 4.5f,  1.5f},
        { 1.8f, -0.9f}, { 2.8f, -4.5f}, { 0.0f, -2.5f},
        {-2.8f, -4.5f}, {-1.8f, -0.9f}, {-4.5f,  1.5f},
        {-1.1f,  1.5f}
    };

  /*  if (active_shape_vertices.empty()) return -1;

    glm::vec2 min_coords = active_shape_vertices[0];
    glm::vec2 max_coords = active_shape_vertices[0];
    for (const auto& v : active_shape_vertices) {
        min_coords.x = std::min(min_coords.x, v.x);
        min_coords.y = std::min(min_coords.y, v.y);
        max_coords.x = std::max(max_coords.x, v.x);
        max_coords.y = std::max(max_coords.y, v.y);
    }
    glm::vec2 center = (min_coords + max_coords) * 0.5f;
    glm::vec2 size = max_coords - min_coords;
    float max_dim = std::max(size.x, size.y);
    float target_size = 10.0f;
    float scale = (max_dim > 1e-6) ? (target_size / max_dim) : 1.0f;
    for (auto& v : active_shape_vertices) {
        v = (v - center) * scale;
    }*/

    // --- 2. 創建所有需要的對象 ---
    Boundary boundary(active_shape_vertices);
    Simulation2D sim(boundary);
    MeshGenerator2D generator;

    // --- 3. 設置 Viewer ---
    Viewer viewer(1280, 720, "SPH Remeshing - Dynamic Mesh Generation");

    // 將所有對象都交給 Viewer 來管理
    viewer.set_boundary(&boundary);
    viewer.set_simulation2d(&sim);
   // viewer.set_mesh_generator2d(&generator);

    // --- 4. 啟動主循環 ---
    viewer.run();

    return 0;
}