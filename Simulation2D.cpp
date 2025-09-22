#include "Simulation2D.h"
#include "Boundary.h"
#include <random>
#include <algorithm>
#include <iostream>

constexpr float PI = 3.1415926535f;

// --- 辅助函数声明 (修复 C3861 "找不到标识符" 错误) ---
// 将函数声明放在文件顶部，这样后面的函数就可以调用它们了
glm::vec2 closest_point_on_segment(const glm::vec2& p, const glm::vec2& a, const glm::vec2& b);
glm::vec2 closest_point_on_polygon(const glm::vec2& p, const std::vector<glm::vec2>& vertices);


// --- SPH Kernels and L-inf Norm (与上一版相同) ---
float Simulation2D::l_inf_norm(const glm::vec2& v) const {
    return std::max(std::abs(v.x), std::abs(v.y));
}

float Simulation2D::wendland_c6_kernel(float q) {
    float h = h_max_;
    if (q >= 0.0f && q < 2.0f) {
        float term = 1.0f - q / 2.0f;
        float term_sq = term * term;
        float term4 = term_sq * term_sq;
        float alpha_d = (78.0f / (28.0f * PI * h * h));
        return alpha_d * term4 * term4 * (4.0f * q * q * q + 6.25f * q * q + 4.0f * q + 1.0f);
    }
    return 0.0f;
}

float Simulation2D::wendland_c6_kernel_derivative(float q) {
    if (q > 1e-6f && q < 2.0f) {
        float h = h_max_;
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


// --- 构造函数和初始化 ---
Simulation2D::Simulation2D(const Boundary& boundary) : boundary_(boundary) {
    const auto& aabb = boundary.get_aabb();
    float domain_width = aabb.z - aabb.x;
    h_max_ = domain_width / 50.0f;
    h_min_ = domain_width / 120.0f;
    mass_ = 1.0f;
    rest_density_ = 1.0f / (h_max_ * h_max_);
    stiffness_ = 5.0f;
    initialize_particles(boundary_);
}

void Simulation2D::compute_target_size_field(const Boundary& boundary, std::vector<float>& target_sizes) {
    const auto& vertices = boundary.get_vertices();
    size_t n = vertices.size();
    if (n < 3) return;
    target_sizes.resize(n);
    for (size_t i = 0; i < n; ++i) {
        const auto& p_prev = vertices[(i + n - 1) % n];
        const auto& p_curr = vertices[i];
        const auto& p_next = vertices[(i + 1) % n];
        glm::vec2 v1 = glm::normalize(p_curr - p_prev);
        glm::vec2 v2 = glm::normalize(p_next - p_curr);
        float dot_product = glm::dot(v1, v2);
        dot_product = std::max(-1.0f, std::min(1.0f, dot_product));
        float angle = std::acos(dot_product);
        float t = angle / PI;
        target_sizes[i] = glm::mix(h_min_, h_max_, t * t);
    }
}

// 修正后的边界粒子生成算法
void Simulation2D::initialize_boundary_particles(const Boundary& boundary, const std::vector<float>& target_sizes) {
    const auto& vertices = boundary.get_vertices();
    size_t n = vertices.size();
    if (n < 2) return;

    float dist_along_boundary = 0.0f;
    float next_particle_dist = 0.0f;

    for (size_t i = 0; i < n; ++i) {
        const auto& p0 = vertices[i];
        const auto& p1 = vertices[(i + 1) % n];
        glm::vec2 edge_vec = p1 - p0;
        float edge_length = glm::length(edge_vec);

        while (next_particle_dist <= dist_along_boundary + edge_length) {
            float t = (next_particle_dist - dist_along_boundary) / edge_length;
            if (t >= 0.0f && t <= 1.0f) {
                particles_.emplace_back(Particle{ p0 + t * edge_vec, glm::vec2(0.0f), glm::vec2(0.0f), true });
            }
            float h_local = glm::mix(target_sizes[i], target_sizes[(i + 1) % n], t);
            next_particle_dist += h_local;
        }
        dist_along_boundary += edge_length;
    }
}


void Simulation2D::initialize_in_domain_particles(const Boundary& boundary) {
    const glm::vec4& aabb = boundary.get_aabb();
    for (float x = aabb.x; x <= aabb.z; x += h_max_) {
        for (float y = aabb.y; y <= aabb.w; y += h_max_) {
            glm::vec2 pos = { x, y };
            if (boundary.is_inside(pos)) {
                bool too_close = false;
                for (const auto& p : particles_) {
                    if (p.is_boundary) {
                        if (glm::distance(pos, p.position) < h_max_ * 0.9f) { // 稍微增大排斥半径
                            too_close = true;
                            break;
                        }
                    }
                }
                if (!too_close) {
                    particles_.emplace_back(Particle{ pos, glm::vec2(0.0f), glm::vec2(0.0f), false });
                }
            }
        }
    }
}

void Simulation2D::initialize_particles(const Boundary& boundary) {
    particles_.clear();
    std::vector<float> target_sizes;
    compute_target_size_field(boundary, target_sizes);
    initialize_boundary_particles(boundary, target_sizes);
    initialize_in_domain_particles(boundary);
    num_particles_ = particles_.size();
    positions_for_render_.resize(num_particles_);
    std::cout << "Generated " << num_particles_ << " total particles." << std::endl;
}

// --- SPH 模拟核心步骤 (与上一版相同) ---
void Simulation2D::compute_forces() {
    for (auto& p : particles_) { p.force = glm::vec2(0.0f); }
    for (int i = 0; i < num_particles_; ++i) {
        if (particles_[i].is_boundary) continue;
        for (int j = 0; j < num_particles_; ++j) {
            if (i == j) continue;
            glm::vec2 diff = particles_[i].position - particles_[j].position;
            float r_inf = l_inf_norm(diff);
            if (r_inf < 2.0f * h_max_) {
                float q = r_inf / h_max_;
                if (q > 1e-6) {
                    float P_term = (stiffness_ / (rest_density_ * rest_density_)) * 2.0f;
                    float W_grad_mag = wendland_c6_kernel_derivative(q);
                    glm::vec2 normalized_diff = diff / r_inf;
                    glm::vec2 force = -mass_ * mass_ * P_term * W_grad_mag * normalized_diff;
                    particles_[i].force += force;
                }
            }
        }
    }
}

void Simulation2D::update_positions() {
    for (auto& p : particles_) {
        if (!p.is_boundary) {
            p.velocity += (p.force / mass_) * time_step_;
            p.velocity *= damping_;
            p.position += p.velocity * time_step_;
        }
    }
}

void Simulation2D::handle_boundaries(const Boundary& boundary) {
    for (auto& p : particles_) {
        if (!p.is_boundary && !boundary.is_inside(p.position)) {
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

// --- 辅助函数定义 (修复 C2084 "已有主体" 错误) ---
// 确保这些函数只被定义一次
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