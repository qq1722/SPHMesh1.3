#pragma once
#include <vector>
#include <glm/glm.hpp>

// ǰ������������ѭ������
class Boundary;

class Simulation2D {
public:
    // ���캯�������ڽ���һ��Boundary���������
    Simulation2D(const Boundary& boundary);

    // ִ��һ��ʱ�䲽��ģ��
    void step();

    // ��ȡ�������ӵ�λ�ã�������Ⱦ
    const std::vector<glm::vec2>& get_particle_positions() const;

private:
    // ��ʼ��ʱ�����ݱ߽�ͱ�����������������
    void initialize_particles(const Boundary& boundary);

    // �������������ܵ����� (���ģ�)
    void compute_forces();

    // ���������������ӵ��ٶȺ�λ��
    void update_positions();

    // ����߽磬��ֹ�����ܳ�����
    void handle_boundaries(const Boundary& boundary);

    struct Particle {
        glm::vec2 position;
        glm::vec2 velocity;
        glm::vec2 force;
        bool is_fixed = false; // �������������̶ֹ����ӺͶ�̬����
    };

    std::vector<Particle> particles_;
    std::vector<glm::vec2> positions_for_render_; // �����洢λ�ã����㴫���GPU
    const Boundary& boundary_; // ���жԱ߽������

    int num_particles_;

    // ģ�����
    float time_step_ = 0.005f;
    float h_ = 0.10f;           // ���Ӽ�࣬Ҳ������������ߴ�
    float stiffness_ = 100.0f;
    float damping_ = 0.99f;
};