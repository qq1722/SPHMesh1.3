#pragma once
#include <vector>
#include <glm/glm.hpp>
#include "Boundary.h"

class BackgroundGrid {
public:
    BackgroundGrid(const Boundary& boundary, float grid_cell_size);

    float get_target_size(const glm::vec2& pos) const;
    // ��������ȡָ��λ�õ�Ŀ�귽�� D_t
    glm::vec2 get_target_direction(const glm::vec2& pos) const;

private:
    void compute_fields(const Boundary& boundary);

    glm::vec2 min_coords_;
    float cell_size_;
    int width_, height_;
    std::vector<float> target_size_field_;     // �洢 h_t
    std::vector<glm::vec2> target_direction_field_; // �������洢 D_t
};