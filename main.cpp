#include "Viewer.h"
#include "Boundary.h"
#include "Simulation2D.h"
#include "MeshGenerator2D.h"
#include "models.h"
#include "qmorph.h"
#include <iostream>
#include <vector>
#include <glm/glm.hpp>
#include <algorithm>

int main() {
    // --- 1. 選擇並歸一化模型 ---
    std::vector<glm::vec2> active_shape_vertices = get_lake_shape_vertices();

    if (active_shape_vertices.empty()) return -1;



    // --- 2. 創建所有需要的對象 ---
    Boundary boundary(active_shape_vertices);
    Simulation2D sim(boundary);
    CGALMeshGenerator  generator;
    Qmorph qmorph_converter;
    //MeshGenerator2D generator;

    // --- 3. 設置 Viewer ---
    Viewer viewer(1280, 720, "SPH Remeshing - Dynamic Mesh Generation");

    // 將所有對象都交給 Viewer 來管理
   viewer.set_boundary(&boundary);
    viewer.set_simulation2d(&sim);
   // viewer.set_mesh_generator2d(&generator);

   // 直接从 sim 对象获取 grid 指针，代码更清晰
    viewer.set_background_grid(sim.get_background_grid());

    viewer.set_cgal_generator(&generator);
    viewer.set_qmorph_generator(&qmorph_converter);


    // --- 4. 啟動主循環 ---
    viewer.run();

    return 0;
}