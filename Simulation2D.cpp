//#include "Simulation2D.h"
//#include "Boundary.h" // ����Boundaryͷ�ļ�
//#include <random>
//#include <algorithm>
//
//// ���캯��ʵ��
//Simulation2D::Simulation2D(const Boundary& boundary)
//    : boundary_(boundary) {
//    initialize_particles(boundary_);
//}
//
//// �����޸ģ���ϲ��Ե����ӳ�ʼ��
//void Simulation2D::initialize_particles(const Boundary& boundary) {
//    particles_.clear();
//
//    // ��ȡ�߽�İ�Χ��
//    const glm::vec4& aabb = boundary.get_aabb();
//    float x_min = aabb.x;
//    float y_min = aabb.y;
//    float x_max = aabb.z;
//    float y_max = aabb.w;
//
//    // �ڰ�Χ���ڱ�����������
//    for (float x = x_min; x <= x_max; x += h_) {
//        for (float y = y_min; y <= y_max; y += h_) {
//            glm::vec2 current_pos(x, y);
//
//            // 1. ���������ڱ߽��ڲ����ʹ���һ������
//            if (boundary.is_inside(current_pos)) {
//                Particle p;
//                p.position = current_pos;
//                p.velocity = glm::vec2(0.0f);
//                p.force = glm::vec2(0.0f);
//
//                // 2. �жϸ������Ƿ�Ϊ����ȫ�ġ��ڲ�����
//                // �������Χ8���ھ��Ƿ�Ҳ���ڱ߽��ڲ�
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
//                // �����ȫ���ͱ��Ϊ�̶�����
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
//            // ���ð뾶������ߴ��Դ�һ�㣬ȷ���ھӼ���������
//            float effective_h = h_ * 1.5f;
//
//            if (dist_sq < effective_h * effective_h && dist_sq > 1e-8) {
//                float dist = std::sqrt(dist_sq);
//                float force_magnitude = stiffness_ * (effective_h - dist) / dist; // �򻯵�SPHѹ��
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
//        // �����޸ģ�ֻ���·ǹ̶������ӣ�
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
//    // ����߽�û�ж��㣬ֱ�ӷ��أ������κδ���
//    if (boundary_vertices.empty()) {
//        return;
//    }
//
//    for (auto& p : particles_) {
//        // ֻ����ǹ̶��ġ����ܵ������������
//        if (!p.is_fixed && !boundary.is_inside(p.position)) {
//
//            float min_dist_sq = FLT_MAX;
//            // ��ȷ���޸����������ʼ�� closest_vertex
//            glm::vec2 closest_vertex = boundary_vertices[0];
//
//            for (const auto& v : boundary_vertices) {
//                float dist_sq = glm::dot(v - p.position, v - p.position);
//                if (dist_sq < min_dist_sq) {
//                    min_dist_sq = dist_sq;
//                    closest_vertex = v;
//                }
//            }
//            p.position = closest_vertex; // ���������Ǿ��԰�ȫ��
//            p.velocity *= -0.5f;
//        }
//    }
//}
//void Simulation2D::step() {
//    compute_forces();
//    update_positions();
//    handle_boundaries(boundary_);
//
//    // ����������Ⱦ��λ������
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

// ��������������� p ���߶� a-b �������
glm::vec2 closest_point_on_segment(const glm::vec2& p, const glm::vec2& a, const glm::vec2& b) {
    glm::vec2 ab = b - a;
    glm::vec2 ap = p - a;

    float proj = glm::dot(ap, ab);
    float ab_len_sq = glm::dot(ab, ab);
    float d = proj / ab_len_sq;

    if (d <= 0.0f) {
        return a; // ������� a
    }
    else if (d >= 1.0f) {
        return b; // ������� b
    }
    else {
        return a + d * ab; // ��������߶��м�
    }
}


// --- ���캯���� initialize_particles, compute_forces, update_positions ���ֲ��� ---
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
    //// *** ����������Ӿ����� ***
    //float padding = h_ * 3.0f; // ����һ�����Ӷ�һ��Ŀ��
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


// *** ���Ǳ�������ĵ��޸� ***
void Simulation2D::handle_boundaries(const Boundary& boundary) {
    const auto& boundary_vertices = boundary.get_vertices();
    if (boundary_vertices.size() < 2) {
        return;
    }

    for (auto& p : particles_) {
        // ֻ�����ܳ�ȥ�Ķ�̬����
        if (!p.is_fixed && !boundary.is_inside(p.position)) {

            glm::vec2 closest_point_on_boundary;
            float min_dist_sq = FLT_MAX;

            // �����߽��ÿһ�����ߡ�
            for (size_t i = 0, j = boundary_vertices.size() - 1; i < boundary_vertices.size(); j = i++) {
                const auto& v1 = boundary_vertices[j];
                const auto& v2 = boundary_vertices[i];

                // �������ӵ������ߵ������
                glm::vec2 closest_pt_on_edge = closest_point_on_segment(p.position, v1, v2);
                float dist_sq = glm::dot(p.position - closest_pt_on_edge, p.position - closest_pt_on_edge);

                // ���������֮ǰ�ҵ��ĸ������͸�����
                if (dist_sq < min_dist_sq) {
                    min_dist_sq = dist_sq;
                    closest_point_on_boundary = closest_pt_on_edge;
                }
            }

            // ������λ���������߽�������ĵ㣬����ת�ٶ�
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