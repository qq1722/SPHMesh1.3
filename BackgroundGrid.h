#pragma once
#include <vector>
#include <glm/glm.hpp>
#include "Boundary.h"

class BackgroundGrid {
public:
    BackgroundGrid(const Boundary& boundary, float grid_cell_size);

    // 获取指定位置的目标尺寸 h_t
    float get_target_size(const glm::vec2& pos) const;

private:
    void compute_fields(const Boundary& boundary);

    glm::vec2 min_coords_; // 网格的起始坐标
    float cell_size_;
    int width_, height_;
    std::vector<float> target_size_field_; // 存储 h_t
};