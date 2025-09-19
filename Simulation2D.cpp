//#include "Simulation2D.h"
//#include "Boundary.h" // 包含Boundary头文件
//#include <random>
//#include <algorithm>
//
//// 构造函数实现
//Simulation2D::Simulation2D(const Boundary& boundary)
//    : boundary_(boundary) {
//    initialize_particles(boundary_);
//}
//
//// 核心修改：混合策略的粒子初始化
//void Simulation2D::initialize_particles(const Boundary& boundary) {
//    particles_.clear();
//
//    // 获取边界的包围盒
//    const glm::vec4& aabb = boundary.get_aabb();
//    float x_min = aabb.x;
//    float y_min = aabb.y;
//    float x_max = aabb.z;
//    float y_max = aabb.w;
//
//    // 在包围盒内遍历背景网格
//    for (float x = x_min; x <= x_max; x += h_) {
//        for (float y = y_min; y <= y_max; y += h_) {
//            glm::vec2 current_pos(x, y);
//
//            // 1. 如果网格点在边界内部，就创建一个粒子
//            if (boundary.is_inside(current_pos)) {
//                Particle p;
//                p.position = current_pos;
//                p.velocity = glm::vec2(0.0f);
//                p.force = glm::vec2(0.0f);
//
//                // 2. 判断该粒子是否为“安全的”内部粒子
//                // 检查它周围8个邻居是否也都在边界内部
//                bool is_safe =
//                    boundary.is_inside({ x + h_, y }) &&
//                    boundary.is_inside({ x - h_, y }) &&
//                    boundary.is_inside({ x,       y + h_ }) &&
//                    boundary.is_inside({ x,       y - h_ }) &&
//                    boundary.is_inside({ x + h_, y + h_ }) &&
//                    boundary.is_inside({ x - h_, y + h_ }) &&
//                    boundary.is_inside({ x + h_, y - h_ }) &&
//                    boundary.is_inside({ x - h_, y - h_ });
//
//                // 如果安全，就标记为固定粒子
//                p.is_fixed = is_safe;
//
//                particles_.push_back(p);
//            }
//        }
//    }
//    num_particles_ = particles_.size();
//    positions_for_render_.resize(num_particles_);
//}
//
//
//void Simulation2D::compute_forces() {
//    for (auto& p : particles_) {
//        p.force = glm::vec2(0.0f, 0.0f);
//    }
//
//    for (int i = 0; i < num_particles_; ++i) {
//        for (int j = i + 1; j < num_particles_; ++j) {
//            glm::vec2 diff = particles_[i].position - particles_[j].position;
//            float dist_sq = glm::dot(diff, diff);
//
//            // 作用半径比网格尺寸稍大一点，确保邻居间有作用力
//            float effective_h = h_ * 1.5f;
//
//            if (dist_sq < effective_h * effective_h && dist_sq > 1e-8) {
//                float dist = std::sqrt(dist_sq);
//                float force_magnitude = stiffness_ * (effective_h - dist) / dist; // 简化的SPH压力
//
//                glm::vec2 force = force_magnitude * diff;
//
//                particles_[i].force += force;
//                particles_[j].force -= force;
//            }
//        }
//    }
//}
//
//
//void Simulation2D::update_positions() {
//    float mass = 1.0f;
//    for (auto& p : particles_) {
//        // 核心修改：只更新非固定的粒子！
//        if (!p.is_fixed) {
//            p.velocity += (p.force / mass) * time_step_;
//            p.velocity *= damping_;
//            p.position += p.velocity * time_step_;
//        }
//    }
//}
//
//
//
//void Simulation2D::handle_boundaries(const Boundary& boundary) {
//    const auto& boundary_vertices = boundary.get_vertices();
//    // 如果边界没有顶点，直接返回，不做任何处理
//    if (boundary_vertices.empty()) {
//        return;
//    }
//
//    for (auto& p : particles_) {
//        // 只处理非固定的、且跑到了外面的粒子
//        if (!p.is_fixed && !boundary.is_inside(p.position)) {
//
//            float min_dist_sq = FLT_MAX;
//            // 正确的修复：在这里初始化 closest_vertex
//            glm::vec2 closest_vertex = boundary_vertices[0];
//
//            for (const auto& v : boundary_vertices) {
//                float dist_sq = glm::dot(v - p.position, v - p.position);
//                if (dist_sq < min_dist_sq) {
//                    min_dist_sq = dist_sq;
//                    closest_vertex = v;
//                }
//            }
//            p.position = closest_vertex; // 现在这里是绝对安全的
//            p.velocity *= -0.5f;
//        }
//    }
//}
//void Simulation2D::step() {
//    compute_forces();
//    update_positions();
//    handle_boundaries(boundary_);
//
//    // 更新用于渲染的位置数据
//    for (int i = 0; i < num_particles_; ++i) {
//        positions_for_render_[i] = particles_[i].position;
//    }
//}
//
//const std::vector<glm::vec2>& Simulation2D::get_particle_positions() const {
//    return positions_for_render_;
//}

#include "Simulation2D.h"
#include "Boundary.h"
#include <random>
#include <algorithm>
#include <glm/gtc/type_ptr.hpp> // for glm::dot

// 辅助函数：计算点 p 到线段 a-b 的最近点
glm::vec2 closest_point_on_segment(const glm::vec2& p, const glm::vec2& a, const glm::vec2& b) {
    glm::vec2 ab = b - a;
    glm::vec2 ap = p - a;

    float proj = glm::dot(ap, ab);
    float ab_len_sq = glm::dot(ab, ab);
    float d = proj / ab_len_sq;

    if (d <= 0.0f) {
        return a; // 最近点是 a
    }
    else if (d >= 1.0f) {
        return b; // 最近点是 b
    }
    else {
        return a + d * ab; // 最近点在线段中间
    }
}


// --- 构造函数和 initialize_particles, compute_forces, update_positions 保持不变 ---
Simulation2D::Simulation2D(const Boundary& boundary)
    : boundary_(boundary) {
    initialize_particles(boundary_);
}

void Simulation2D::initialize_particles(const Boundary& boundary) {
    particles_.clear();
    const glm::vec4& aabb = boundary.get_aabb();
    float x_min = aabb.x;
    float y_min = aabb.y;
    float x_max = aabb.z;
    float y_max = aabb.w;
    //// *** 在这里添加视觉补偿 ***
    //float padding = h_ * 3.0f; // 增加一个粒子多一点的宽度
    //x_min -= padding;
    //y_min -= padding;
    //x_max += padding;
    //y_max += padding;

    for (float x = x_min; x <= x_max; x += h_) {
        for (float y = y_min; y <= y_max; y += h_) {
            glm::vec2 current_pos(x, y);
            if (boundary.is_inside(current_pos)) {
                Particle p;
                p.position = current_pos;
                p.velocity = glm::vec2(0.0f);
                p.force = glm::vec2(0.0f);

                bool is_safe =
                    boundary.is_inside({ x + h_, y }) &&
                    boundary.is_inside({ x - h_, y }) &&
                    boundary.is_inside({ x,       y + h_ }) &&
                    boundary.is_inside({ x,       y - h_ }) &&
                    boundary.is_inside({ x + h_, y + h_ }) &&
                    boundary.is_inside({ x - h_, y + h_ }) &&
                    boundary.is_inside({ x + h_, y - h_ }) &&
                    boundary.is_inside({ x - h_, y - h_ });
                p.is_fixed = is_safe;
                particles_.push_back(p);
            }
        }
    }
    num_particles_ = particles_.size();
    positions_for_render_.resize(num_particles_);
}

void Simulation2D::compute_forces() {
    for (auto& p : particles_) {
        p.force = glm::vec2(0.0f, 0.0f);
    }
    for (int i = 0; i < num_particles_; ++i) {
        for (int j = i + 1; j < num_particles_; ++j) {
            glm::vec2 diff = particles_[i].position - particles_[j].position;
            float dist_sq = glm::dot(diff, diff);
            float effective_h = h_ * 1.5f;
            if (dist_sq < effective_h * effective_h && dist_sq > 1e-8) {
                float dist = std::sqrt(dist_sq);
                float force_magnitude = stiffness_ * (effective_h - dist) / dist;
                glm::vec2 force = force_magnitude * diff;
                particles_[i].force += force;
                particles_[j].force -= force;
            }
        }
    }
}

void Simulation2D::update_positions() {
    float mass = 1.0f;
    for (auto& p : particles_) {
        if (!p.is_fixed) {
            p.velocity += (p.force / mass) * time_step_;
            p.velocity *= damping_;
            p.position += p.velocity * time_step_;
        }
    }
}
// ---------------------------------------------------------------------------------


// *** 这是本次最核心的修改 ***
void Simulation2D::handle_boundaries(const Boundary& boundary) {
    const auto& boundary_vertices = boundary.get_vertices();
    if (boundary_vertices.size() < 2) {
        return;
    }

    for (auto& p : particles_) {
        // 只处理跑出去的动态粒子
        if (!p.is_fixed && !boundary.is_inside(p.position)) {

            glm::vec2 closest_point_on_boundary;
            float min_dist_sq = FLT_MAX;

            // 遍历边界的每一条“边”
            for (size_t i = 0, j = boundary_vertices.size() - 1; i < boundary_vertices.size(); j = i++) {
                const auto& v1 = boundary_vertices[j];
                const auto& v2 = boundary_vertices[i];

                // 计算粒子到这条边的最近点
                glm::vec2 closest_pt_on_edge = closest_point_on_segment(p.position, v1, v2);
                float dist_sq = glm::dot(p.position - closest_pt_on_edge, p.position - closest_pt_on_edge);

                // 如果这个点比之前找到的更近，就更新它
                if (dist_sq < min_dist_sq) {
                    min_dist_sq = dist_sq;
                    closest_point_on_boundary = closest_pt_on_edge;
                }
            }

            // 将粒子位置修正到边界上最近的点，并反转速度
            p.position = closest_point_on_boundary;
            p.velocity *= -0.5f;
        }
    }
}

void Simulation2D::step() {
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