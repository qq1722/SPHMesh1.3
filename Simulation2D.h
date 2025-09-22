#pragma once
#include <vector>
#include <glm/glm.hpp>

class Boundary;

class Simulation2D {
public:
    Simulation2D(const Boundary& boundary);
    void step();
    const std::vector<glm::vec2>& get_particle_positions() const;

    // 内部结构体的前向声明
    struct Particle;

    // 修复 C2440 错误：使用完整的作用域名 Simulation2D::Particle
    const std::vector<Simulation2D::Particle>& get_particles() const { return particles_; }

private:
    void initialize_particles(const Boundary& boundary);
    void compute_forces();
    void update_positions();
    void handle_boundaries(const Boundary& boundary);

    // L-infinity norm as per Hu et al., 2025
    float l_inf_norm(const glm::vec2& v) const;

    // SPH kernel functions from Hu et al., 2025
    float wendland_c6_kernel(float q);
    float wendland_c6_kernel_derivative(float q);

    // 结构体定义移到 private 区域
    struct Particle {
        glm::vec2 position;
        glm::vec2 velocity = glm::vec2(0.0f);
        glm::vec2 force = glm::vec2(0.0f);
        float density = 0.0f; // 实际密度
        float pressure = 0.0f;
    };

    std::vector<Particle> particles_;
    std::vector<glm::vec2> positions_for_render_;
    const Boundary& boundary_;
    int num_particles_ = 0;

    // --- 经过重新校准的SPH模拟参数 ---
    float time_step_ = 0.005f;
    float h_;               // 目标尺寸 h_t, 也是平滑半径
    float mass_;              // 粒子质量
    float rest_density_;      // 目标密度 rho_t
    float stiffness_;         // 压力常数 P0
    float damping_ = 0.998f;   // 阻尼
};