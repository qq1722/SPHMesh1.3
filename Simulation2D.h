#pragma once
#include <vector>
#include <glm/glm.hpp>
#include "BackgroundGrid.h" // ������ͷ�ļ�
#include <memory> // for std::unique_ptr

class Boundary;

class Simulation2D {
public:
    struct Particle {
        glm::vec2 position;
        glm::vec2 velocity = glm::vec2(0.0f);
        glm::vec2 force = glm::vec2(0.0f);
        float smoothing_h = 0.0f; // ÿ���������Լ������ð뾶h_t
        float target_density = 0.0f; // ÿ���������Լ���Ŀ���ܶ�rho_t
    };

    Simulation2D(const Boundary& boundary);
    void step();
    const std::vector<glm::vec2>& get_particle_positions() const;
    const std::vector<Particle>& get_particles() const { return particles_; }

private:
    void initialize_particles(const Boundary& boundary);
    void compute_forces();
    void update_positions();
    void handle_boundaries(const Boundary& boundary);

    // SPH kernel functions
    float wendland_c6_kernel(float q, float h);
    float wendland_c6_kernel_derivative(float q, float h);
    float l_inf_norm(const glm::vec2& v) const;

    std::vector<Particle> particles_;
    std::vector<glm::vec2> positions_for_render_;
    const Boundary& boundary_;
    std::unique_ptr<BackgroundGrid> grid_; // ʹ������ָ�����������
    int num_particles_ = 0;

    // SPH ģ�����
    float time_step_ = 0.005f;
    float mass_ = 1.0f;
    float stiffness_ = 80.0f;
    float damping_ = 0.998f;
};