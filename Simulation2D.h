#pragma once
#include <vector>
#include <glm/glm.hpp>
#include "BackgroundGrid.h"
//#include "DelaunayMeshGenerator.h"
#include <memory>

class Boundary;

class Simulation2D {
public:
    struct Particle {
        glm::vec2 position;
        glm::vec2 velocity = glm::vec2(0.0f);
        glm::vec2 force = glm::vec2(0.0f);
        float smoothing_h = 0.0f;
        float target_density = 0.0f;
        glm::mat2 rotation = glm::mat2(1.0f); // 新增：局部坐标系的旋转矩阵
		bool is_boundary = false;
    };

    Simulation2D(const Boundary& boundary);
    void step();
    const std::vector<glm::vec2>& get_particle_positions() const;
    const std::vector<Particle>& get_particles() const { return particles_; }
    // 新增：计算系统总动能，用于收敛判断
    float get_kinetic_energy() const;
    // 新增：提供对背景网格的访问
    BackgroundGrid* get_background_grid() const { return grid_.get(); }
    float get_min_target_size() const { return h_min_; } // <-- 新增

private:
    void initialize_particles(const Boundary& boundary);
    void compute_forces();
    void update_positions();
    void handle_boundaries(const Boundary& boundary);

    // 辅助函数
    glm::vec2 transform_to_local(const glm::vec2& vec, const glm::mat2& rot_matrix) const;
    float l_inf_norm(const glm::vec2& v) const;
    float wendland_c6_kernel(float q, float h);
    float wendland_c6_kernel_derivative(float q, float h);

    std::vector<Particle> particles_;
    std::vector<glm::vec2> positions_for_render_;
    const Boundary& boundary_;
    std::unique_ptr<BackgroundGrid> grid_;
    int num_particles_ = 0;

    // SPH 模拟参数
    float time_step_ = 0.005f;
    float mass_ = 1.0f;
    float stiffness_ = 1.0f;//releace模式1.0f好用
    float damping_ = 0.998f;
    float h_max_;             // 最大目标尺寸
    float h_min_;             // <-- 新增：补上这个缺失的声明
};