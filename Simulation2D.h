#pragma once
#include <vector>
#include <glm/glm.hpp>

// 前向声明，避免循环引用
class Boundary;

class Simulation2D {
public:
    // 构造函数，现在接收一个Boundary对象的引用
    Simulation2D(const Boundary& boundary);

    // 执行一个时间步的模拟
    void step();

    // 获取所有粒子的位置，用于渲染
    const std::vector<glm::vec2>& get_particle_positions() const;

private:
    // 初始化时，根据边界和背景网格来放置粒子
    void initialize_particles(const Boundary& boundary);

    // 计算所有粒子受到的力 (核心！)
    void compute_forces();

    // 根据力来更新粒子的速度和位置
    void update_positions();

    // 处理边界，防止粒子跑出区域
    void handle_boundaries(const Boundary& boundary);

    struct Particle {
        glm::vec2 position;
        glm::vec2 velocity;
        glm::vec2 force;
        bool is_fixed = false; // 新增！用于区分固定粒子和动态粒子
    };

    std::vector<Particle> particles_;
    std::vector<glm::vec2> positions_for_render_; // 单独存储位置，方便传输给GPU
    const Boundary& boundary_; // 持有对边界的引用

    int num_particles_;

    // 模拟参数
    float time_step_ = 0.005f;
    float h_ = 0.10f;           // 粒子间距，也用作背景网格尺寸
    float stiffness_ = 100.0f;
    float damping_ = 0.99f;
};