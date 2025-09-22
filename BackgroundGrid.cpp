#include "BackgroundGrid.h"
#include "Utils.h" // <-- 包含新的工具文件
#include <algorithm>
#include <vector>

BackgroundGrid::BackgroundGrid(const Boundary& boundary, float grid_cell_size) {
    cell_size_ = grid_cell_size;
    const auto& aabb = boundary.get_aabb();
    min_coords_ = { aabb.x, aabb.y };
    // 增加一点安全边界，防止浮点数误差
    width_ = static_cast<int>((aabb.z - aabb.x) / cell_size_) + 3;
    height_ = static_cast<int>((aabb.w - aabb.y) / cell_size_) + 3;
    target_size_field_.resize(width_ * height_);

    compute_fields(boundary);
}

void BackgroundGrid::compute_fields(const Boundary& boundary) {
    const auto& boundary_vertices = boundary.get_vertices();
    if (boundary_vertices.size() < 2) return;

    std::vector<float> boundary_target_sizes(boundary_vertices.size());
    float h_min = cell_size_ * 0.5f;
    float h_max = cell_size_ * 2.0f;
    for (size_t i = 0; i < boundary_vertices.size(); ++i) {
        const auto& p_prev = boundary_vertices[(i + boundary_vertices.size() - 1) % boundary_vertices.size()];
        const auto& p_curr = boundary_vertices[i];
        const auto& p_next = boundary_vertices[(i + 1) % boundary_vertices.size()];
        glm::vec2 v1 = glm::normalize(p_curr - p_prev);
        glm::vec2 v2 = glm::normalize(p_next - p_curr);
        float dot_product = glm::dot(v1, v2);
        dot_product = std::max(-1.0f, std::min(1.0f, dot_product));
        float angle = std::acos(dot_product);
        float t = angle / 3.1415926535f;
        boundary_target_sizes[i] = glm::mix(h_min, h_max, t * t);
    }

    std::vector<float> distance_field(width_ * height_, FLT_MAX);
    std::vector<glm::vec2> closest_boundary_point(width_ * height_);

    // 这个循环现在是安全的
    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            glm::vec2 grid_pos = min_coords_ + glm::vec2(x * cell_size_, y * cell_size_);
            // 检查网格点是否在边界内或非常靠近边界，以确保边界附近的场是平滑的
            glm::vec2 closest_pt = closest_point_on_polygon(grid_pos, boundary_vertices);
            float dist_to_boundary = glm::distance(grid_pos, closest_pt);

            // 只计算边界内部及附近一定范围内的场
            if (boundary.is_inside(grid_pos) || dist_to_boundary < h_max * 2.0f) {
                closest_boundary_point[y * width_ + x] = closest_pt;
                distance_field[y * width_ + x] = dist_to_boundary;
            }
        }
    }

    for (int i = 0; i < target_size_field_.size(); ++i) {
        if (distance_field[i] != FLT_MAX) {
            float influence_radius = h_max * 4.0f;
            float t = std::min(distance_field[i] / influence_radius, 1.0f);
            target_size_field_[i] = glm::mix(h_min, h_max, t);
        }
        else {
            target_size_field_[i] = h_max;
        }
    }
}

float BackgroundGrid::get_target_size(const glm::vec2& pos) const {
    glm::vec2 local_pos = (pos - min_coords_) / cell_size_;
    int x0 = static_cast<int>(local_pos.x);
    int y0 = static_cast<int>(local_pos.y);
    int x1 = x0 + 1;
    int y1 = y0 + 1;

    x0 = std::max(0, std::min(x0, width_ - 1));
    y0 = std::max(0, std::min(y0, height_ - 1));
    x1 = std::max(0, std::min(x1, width_ - 1));
    y1 = std::max(0, std::min(y1, height_ - 1));

    float tx = local_pos.x - x0;
    float ty = local_pos.y - y0;

    float s00 = target_size_field_[y0 * width_ + x0];
    float s10 = target_size_field_[y0 * width_ + x1];
    float s01 = target_size_field_[y1 * width_ + x0];
    float s11 = target_size_field_[y1 * width_ + x1];

    float s_y0 = glm::mix(s00, s10, tx);
    float s_y1 = glm::mix(s01, s11, tx);

    return glm::mix(s_y0, s_y1, ty);
}