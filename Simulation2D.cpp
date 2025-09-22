#include "Simulation2D.h"
#include "Boundary.h"
#include <random>
#include <algorithm>
#include <iostream>

constexpr float PI = 3.1415926535f;

// --- L-infinity Norm (Hu et al., 2025, Sec. 3.4.1) ---
float Simulation2D::l_inf_norm(const glm::vec2& v) const {
    return std::max(std::abs(v.x), std::abs(v.y));
}

// --- Wendland Quintic C6 Kernel & Derivative (Hu et al., 2025, Eq. 13) ---
// 修复了之前版本中的一个小错误 (term4 * term4)
float Simulation2D::wendland_c6_kernel(float q) {
    if (q >= 0.0f && q < 2.0f) {
        float term = 1.0f - q / 2.0f;
        float term_sq = term * term;
        float term4 = term_sq * term_sq;
        float alpha_d = (78.0f / (28.0f * PI * h_ * h_));
        return alpha_d * term4 * term4 * (4.0f * q * q * q + 6.25f * q * q + 4.0f * q + 1.0f);
    }
    return 0.0f;
}

float Simulation2D::wendland_c6_kernel_derivative(float q) {
    if (q > 1e-6f && q < 2.0f) {
        float term = 1.0f - q / 2.0f;
        float term_sq = term * term;
        float term3 = term_sq * term;
        float term4 = term_sq * term_sq;
        float term7 = term3 * term4;
        float alpha_d = (78.0f / (28.0f * PI * h_ * h_));
        // 使用链式法则求导 dW/dr = (dW/dq) * (dq/dr) = (dW/dq) / h
        return alpha_d * term7 * (-10.0f * q * q * q - 10.25f * q * q - 2.0f * q) / h_;
    }
    return 0.0f;
}

// --- 构造函数和初始化 ---
Simulation2D::Simulation2D(const Boundary& boundary) : boundary_(boundary) {
    // 动态计算参数
    const auto& aabb = boundary.get_aabb();
    float domain_width = aabb.z - aabb.x;
    h_ = domain_width / 50.0f; // 目标尺寸 h_t，大约在宽度上分布25个粒子

    mass_ = 1.0f;
    rest_density_ = 1.0f / (h_ * h_); // 目标密度 rho_t
    stiffness_ = 5.0f; // 压力常数 P0

    initialize_particles(boundary_);
}

void Simulation2D::initialize_particles(const Boundary& boundary) {
    particles_.clear();
    const glm::vec4& aabb = boundary.get_aabb();
    for (float x = aabb.x; x <= aabb.z; x += h_) {
        for (float y = aabb.y; y <= aabb.w; y += h_) {
            if (boundary.is_inside({ x, y })) {
                particles_.emplace_back(Particle{ {x, y} });
            }
        }
    }
    num_particles_ = particles_.size();
    positions_for_render_.resize(num_particles_);
    std::cout << "Generated " << num_particles_ << " particles." << std::endl;
}

// --- SPH 模拟核心步骤 ---
void Simulation2D::compute_forces() {
    for (auto& p : particles_) {
        p.force = glm::vec2(0.0f);
    }

    for (int i = 0; i < num_particles_; ++i) {
        for (int j = i + 1; j < num_particles_; ++j) {
            glm::vec2 diff = particles_[i].position - particles_[j].position;

            // 核心思想：使用 L∞ 范数来判断邻居关系
            float r_inf = l_inf_norm(diff);

            // Wendland 核函数的作用范围是 2h
            if (r_inf < 2.0f * h_) {
                float q = r_inf / h_;
                if (q > 1e-6) {
                    // --- 压力 (Hu et al., Eq. 9) ---
                    // 这个公式驱动粒子达到目标密度 rho_t (rest_density_)
                    float P_term = (stiffness_ / (rest_density_ * rest_density_)) * 2.0f;
                    float W_grad_mag = wendland_c6_kernel_derivative(q);

                    // L∞ 归一化向量 (Eq. 5)
                    glm::vec2 normalized_diff = diff / r_inf;

                    glm::vec2 force = -mass_ * mass_ * P_term * W_grad_mag * normalized_diff;

                    particles_[i].force += force;
                    particles_[j].force -= force;
                }
            }
        }
    }
}

void Simulation2D::update_positions() {
    for (auto& p : particles_) {
        // 使用质量 mass_ 而不是易出错的密度 density
        p.velocity += (p.force / mass_) * time_step_;
        p.velocity *= damping_;
        p.position += p.velocity * time_step_;
    }
}

// --- 边界处理 (与上一版相同，但更稳健) ---
glm::vec2 closest_point_on_segment(const glm::vec2& p, const glm::vec2& a, const glm::vec2& b) {
    glm::vec2 ab = b - a;
    glm::vec2 ap = p - a;
    float proj = glm::dot(ap, ab);
    float ab_len_sq = glm::dot(ab, ab);
    if (ab_len_sq < 1e-9) return a;
    float d = proj / ab_len_sq;
    if (d <= 0.0f) return a;
    if (d >= 1.0f) return b;
    return a + d * ab;
}

glm::vec2 closest_point_on_polygon(const glm::vec2& p, const std::vector<glm::vec2>& vertices) {
    if (vertices.empty()) return p;
    glm::vec2 closest_point = vertices[0];
    float min_dist_sq = FLT_MAX;
    for (size_t i = 0, j = vertices.size() - 1; i < vertices.size(); j = i++) {
        glm::vec2 closest_pt_on_edge = closest_point_on_segment(p, vertices[j], vertices[i]);
        float dist_sq = glm::dot(p - closest_pt_on_edge, p - closest_pt_on_edge);
        if (dist_sq < min_dist_sq) {
            min_dist_sq = dist_sq;
            closest_point = closest_pt_on_edge;
        }
    }
    return closest_point;
}

void Simulation2D::handle_boundaries(const Boundary& boundary) {
    for (auto& p : particles_) {
        if (!boundary.is_inside(p.position)) {
            p.position = closest_point_on_polygon(p.position, boundary.get_vertices());
            p.velocity *= -0.5f; // 带阻尼的速度反弹
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