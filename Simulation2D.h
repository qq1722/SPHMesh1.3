#pragma once
#include <vector>
#include <glm/glm.hpp>

class Boundary;

class Simulation2D {
public: // <-- 将 Particle 结构体移至 public 区域
    struct Particle {
        glm::vec2 position;
        glm::vec2 velocity = glm::vec2(0.0f);
        glm::vec2 force = glm::vec2(0.0f);
        bool is_boundary = false;
    };

    Simulation2D(const Boundary& boundary);
    void step();
    const std::vector<glm::vec2>& get_particle_positions() const;
    const std::vector<Particle>& get_particles() const { return particles_; }

private:
    void initialize_particles(const Boundary& boundary);
    void compute_target_size_field(const Boundary& boundary, std::vector<float>& target_sizes);
    void initialize_boundary_particles(const Boundary& boundary, const std::vector<float>& target_sizes);
    void initialize_in_domain_particles(const Boundary& boundary);
    void handle_boundaries(const Boundary& boundary);

    void compute_forces();
    void update_positions();

    // SPH kernel functions and helpers
    float wendland_c6_kernel(float q);
    float wendland_c6_kernel_derivative(float q);
    float l_inf_norm(const glm::vec2& v) const;

    std::vector<Particle> particles_;
    std::vector<glm::vec2> positions_for_render_;
    const Boundary& boundary_;
    int num_particles_ = 0;

    // SPH 模拟参数
    float time_step_ = 0.005f;
    float h_min_;
    float h_max_;
    float mass_ = 1.0f;
    float rest_density_;
    float stiffness_;
    float damping_ = 0.998f;
};