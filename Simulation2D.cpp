#include "Simulation2D.h"
#include "Boundary.h"
#include "Utils.h"
#include <random>
#include <algorithm>
#include <iostream>

constexpr float PI = 3.1415926535f;

// --- 辅助函数：将向量转换到局部坐标系 ---
glm::vec2 Simulation2D::transform_to_local(const glm::vec2& vec, const glm::mat2& rot_matrix) const {
    // 旋转矩阵的逆是它的转置
    return glm::transpose(rot_matrix) * vec;
}



Simulation2D::Simulation2D(const Boundary& boundary) : boundary_(boundary) {
    const auto& aabb = boundary.get_aabb();
    float domain_width = aabb.z - aabb.x;
    float grid_cell_size = domain_width / 80.0f;
    grid_ = std::make_unique<BackgroundGrid>(boundary, grid_cell_size);
    initialize_particles(boundary);
}


// --- 核心修改：在力计算中引入局部坐标系 ---
void Simulation2D::compute_forces() {
    for (auto& p : particles_) { p.force = glm::vec2(0.0f); }

    for (int i = 0; i < num_particles_; ++i) {
        for (int j = i + 1; j < num_particles_; ++j) {
            glm::vec2 diff_global = particles_[i].position - particles_[j].position;
            float h_avg = (particles_[i].smoothing_h + particles_[j].smoothing_h) * 0.5f;

            // 关键：将全局位移向量转换到粒子i的局部坐标系
            glm::vec2 diff_local_i = transform_to_local(diff_global, particles_[i].rotation);
            float r_inf = l_inf_norm(diff_local_i);

            if (r_inf < 2.0f * h_avg) {
                float q = r_inf / h_avg;
                if (q > 1e-6) {
                    float rho_t_i = particles_[i].target_density;
                    float rho_t_j = particles_[j].target_density;
                    float P_term = (stiffness_ / (rho_t_i * rho_t_i)) + (stiffness_ / (rho_t_j * rho_t_j));
                    float W_grad_mag = wendland_c6_kernel_derivative(q, h_avg);

                    // L∞归一化向量（在局部坐标系下）
                    glm::vec2 normalized_diff_local = diff_local_i / r_inf;

                    // 计算局部坐标系下的力
                    glm::vec2 force_local = -mass_ * mass_ * P_term * W_grad_mag * normalized_diff_local;

                    // 关键：将局部力转换回全局坐标系
                    glm::vec2 force_global = particles_[i].rotation * force_local;

                    particles_[i].force += force_global;
                    particles_[j].force -= force_global;
                }
            }
        }
    }
}


// --- 核心修改：在位置更新后，更新粒子的方向 ---
void Simulation2D::update_positions() {
    for (auto& p : particles_) {
        p.velocity += (p.force / mass_) * time_step_;
        p.velocity *= damping_;
        p.position += p.velocity * time_step_;

        // 从背景网格更新每个粒子的目标参数
        p.smoothing_h = grid_->get_target_size(p.position);
        p.target_density = 1.0f / (p.smoothing_h * p.smoothing_h);

        // 关键：更新粒子的旋转矩阵以对齐方向场
        glm::vec2 target_dir = grid_->get_target_direction(p.position);
        glm::vec2 current_dir = p.rotation[0]; // 局部X轴

        // 使用少量插值平滑地转向目标方向，防止抖动
        glm::vec2 new_dir = glm::normalize(current_dir + (target_dir - current_dir) * 0.1f);

        p.rotation[0] = new_dir;
        p.rotation[1] = glm::vec2(-new_dir.y, new_dir.x); // 保持正交
    }
}




// ... (compute_forces, update_positions, step, get_particle_positions 保持不变) ...


void Simulation2D::initialize_particles(const Boundary& boundary) {
    particles_.clear();
    const glm::vec4& aabb = boundary.get_aabb();
    std::mt19937 rng(std::random_device{}());
    std::uniform_real_distribution<float> dist(-0.25f, 0.25f);

    // 使用背景网格的目标尺寸来决定播撒步长
    for (float y = aabb.y; y <= aabb.w; ) {
        float current_h_y = grid_->get_target_size({ aabb.x, y });
        for (float x = aabb.x; x <= aabb.z; ) {
            float current_h_x = grid_->get_target_size({ x, y });
            glm::vec2 pos = { x + dist(rng) * current_h_x, y + dist(rng) * current_h_y };
            if (boundary.is_inside(pos)) {
                float h_t = grid_->get_target_size(pos);
                particles_.emplace_back(Particle{ pos, {}, {}, h_t, 1.0f / (h_t * h_t) });
            }
            x += current_h_x;
        }
        y += current_h_y;
    }

    num_particles_ = particles_.size();
    positions_for_render_.resize(num_particles_);
    std::cout << "Generated " << num_particles_ << " adaptive particles." << std::endl;
}

float Simulation2D::wendland_c6_kernel(float q, float h) {
    if (q >= 0.0f && q < 2.0f) {
        float term = 1.0f - q / 2.0f;
        float term_sq = term * term;
        float term4 = term_sq * term_sq;
        float alpha_d = (78.0f / (28.0f * PI * h * h));
        return alpha_d * term4 * term4 * (4.0f * q * q * q + 6.25f * q * q + 4.0f * q + 1.0f);
    }
    return 0.0f;
}

float Simulation2D::wendland_c6_kernel_derivative(float q, float h) {
    if (q > 1e-6f && q < 2.0f) {
        float term = 1.0f - q / 2.0f;
        float term_sq = term * term;
        float term3 = term_sq * term;
        float term4 = term_sq * term_sq;
        float term7 = term3 * term4;
        float alpha_d = (78.0f / (28.0f * PI * h * h));
        return alpha_d * term7 * (-10.0f * q * q * q - 10.25f * q * q - 2.0f * q) / h;
    }
    return 0.0f;
}

float Simulation2D::l_inf_norm(const glm::vec2& v) const {
    return std::max(std::abs(v.x), std::abs(v.y));
}





void Simulation2D::handle_boundaries(const Boundary& boundary) {
    for (auto& p : particles_) {
        if (!boundary.is_inside(p.position)) {
            p.position = closest_point_on_polygon(p.position, boundary.get_vertices());
            p.velocity *= -0.5f;
        }
    }
}

void Simulation2D::step() {
    if (num_particles_ == 0) return;
    compute_forces();
    update_positions();
    handle_boundaries(boundary_);
    for (int i = 0; i < num_particles_; ++i) {
        positions_for_render_[i] = particles_[i].position;
    }
}

const std::vector<glm::vec2>& Simulation2D::get_particle_positions() const {
    return positions_for_render_;
}

// 新增函数实现
float Simulation2D::get_kinetic_energy() const {
    float total_energy = 0.0f;
    for (const auto& p : particles_) {
        total_energy += 0.5f * mass_ * glm::dot(p.velocity, p.velocity);
    }
    return total_energy;
}