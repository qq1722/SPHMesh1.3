#pragma once
#include <vector>
#include <glm/glm.hpp>

class Boundary;

class Simulation2D {
public:
    Simulation2D(const Boundary& boundary);
    void step();
    const std::vector<glm::vec2>& get_particle_positions() const;

    // �ڲ��ṹ���ǰ������
    struct Particle;

    // �޸� C2440 ����ʹ���������������� Simulation2D::Particle
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

    // �ṹ�嶨���Ƶ� private ����
    struct Particle {
        glm::vec2 position;
        glm::vec2 velocity = glm::vec2(0.0f);
        glm::vec2 force = glm::vec2(0.0f);
        float density = 0.0f; // ʵ���ܶ�
        float pressure = 0.0f;
    };

    std::vector<Particle> particles_;
    std::vector<glm::vec2> positions_for_render_;
    const Boundary& boundary_;
    int num_particles_ = 0;

    // --- ��������У׼��SPHģ����� ---
    float time_step_ = 0.005f;
    float h_;               // Ŀ��ߴ� h_t, Ҳ��ƽ���뾶
    float mass_;              // ��������
    float rest_density_;      // Ŀ���ܶ� rho_t
    float stiffness_;         // ѹ������ P0
    float damping_ = 0.998f;   // ����
};